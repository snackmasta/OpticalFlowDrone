# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink, smbus2

import cv2
import numpy as np
import threading
import subprocess
import math
import struct
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
from smbus2 import SMBus
from picamera2 import Picamera2
import time
from datetime import datetime
from pathlib import Path
from pymavlink import mavutil

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS
RECORDINGS_DIR = Path("recordings")
PROJECT_ROOT = Path(__file__).resolve().parent
MEDIAMTX_BIN = PROJECT_ROOT / ".tools" / "mediamtx" / "mediamtx"
MAVLINK_CONNECTION_STRING = "udp:127.0.0.1:14551"
IMU_I2C_BUS = 1
IMU_I2C_ADDR = 0x68
IMU_PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
ACCEL_LSB_PER_G = 16384.0
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
GYRO_LSB_PER_DPS = 131.0
COMPLEMENTARY_FILTER_ALPHA = 0.96
COMPASS_SHM_NAME = "compass_heading_stream"
COMPASS_SHM_MAGIC = b"CHDG"
COMPASS_SHM_HEADER_FORMAT = "<4sII"
COMPASS_SHM_RECORD_FORMAT = "<6d"
COMPASS_SHM_HEADER_SIZE = struct.calcsize(COMPASS_SHM_HEADER_FORMAT)
COMPASS_SHM_RECORD_SIZE = struct.calcsize(COMPASS_SHM_RECORD_FORMAT)
COMPASS_MAX_SAMPLES = 120
COMPASS_FRESHNESS_THRESHOLD_S = 0.75

gyro_bias = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
}

distance_state = {
    "current_distance": None,
    "last_update": 0.0,
}
distance_lock = threading.Lock()
attitude_state = {
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
    "xgyro_dps": 0.0,
    "ygyro_dps": 0.0,
    "zgyro_dps": 0.0,
    "last_update": 0.0,
}
attitude_lock = threading.Lock()
compass_state = {
    "heading_deg": None,
    "timestamp": 0.0,
    "last_update": 0.0,
}
compass_lock = threading.Lock()
compass_stream_shm = None
accel_state = {
    "x_g": 0.0,
    "y_g": 0.0,
    "z_g": 0.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "tilt_deg": 0.0,
    "last_update": 0.0,
}
accel_lock = threading.Lock()
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
try:
    frame_duration_us = int(1_000_000 / TARGET_FPS)
    camera_config['controls']['FrameDurationLimits'] = (frame_duration_us, frame_duration_us)
except Exception:
    pass
picam2.configure(camera_config)
picam2.start()
time.sleep(2)

# Optical flow parameters
TRACK_FEATURE_COUNT = 10
FLOW_SCALE = 0.5
CAMERA_HORIZONTAL_FOV_DEG = 62.2
MAX_FLOW_STEP_PX = 80.0
MIN_INLIERS_FOR_VELOCITY = 3
RETICLE_COLOR = (0, 220, 220)
feature_params = dict(maxCorners=TRACK_FEATURE_COUNT, qualityLevel=0.3, minDistance=5, blockSize=5)
lk_params = dict(
    winSize=(9, 9),
    maxLevel=0,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 8, 0.03),
)

velocity_state = {
    "vx_mps": 0.0,
    "vy_mps": 0.0,
    "speed_mps": 0.0,
    "inliers": 0,
    "last_update": 0.0,
}


def to_small_gray(frame):
    small = cv2.resize(frame, None, fx=FLOW_SCALE, fy=FLOW_SCALE, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)


def ensure_bgr(frame):
    if frame.ndim == 3 and frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def read_i2c_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def normalize_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def angular_error_deg(target_deg, current_deg):
    return normalize_angle_deg(target_deg - current_deg)


def blend_angle_deg(current_deg, target_deg, blend):
    return normalize_angle_deg(current_deg + (blend * angular_error_deg(target_deg, current_deg)))


def invert_compass_heading_deg(heading_deg):
    return normalize_angle_deg(-heading_deg)


def compass_cardinal_from_heading_deg(heading_deg):
    directions = ["N", "NW", "W", "SW", "S", "SE", "E", "NE"]   
    normalized_heading_deg = heading_deg % 360.0
    index = int((normalized_heading_deg + 22.5) // 45.0) % 8
    return directions[index]


def draw_compass_widget(frame, heading_deg, age_s):
    h, w = frame.shape[:2]
    radius_outer = max(22, min(w, h) // 12)
    radius_inner = max(8, radius_outer // 2)
    center = (w - radius_outer - 22, radius_outer + 22)

    compass_overlay = frame.copy()
    cv2.circle(compass_overlay, center, radius_outer, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.circle(compass_overlay, center, radius_inner, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] - radius_outer, center[1]), (center[0] + radius_outer, center[1]), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0], center[1] - radius_outer), (center[0], center[1] + radius_outer), (255, 255, 255), 1, cv2.LINE_AA)

    tick = max(4, radius_outer // 5)
    cv2.line(compass_overlay, (center[0], center[1] - radius_outer), (center[0], center[1] - radius_outer - tick), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] + radius_outer, center[1]), (center[0] + radius_outer + tick, center[1]), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0], center[1] + radius_outer), (center[0], center[1] + radius_outer + tick), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] - radius_outer, center[1]), (center[0] - radius_outer - tick, center[1]), (255, 255, 255), 1, cv2.LINE_AA)

    if heading_deg is not None:
        heading_rad = math.radians(-heading_deg)
        needle_length = radius_outer - 4
        needle_end = (
            int(center[0] + math.sin(heading_rad) * needle_length),
            int(center[1] - math.cos(heading_rad) * needle_length),
        )
        cv2.line(compass_overlay, center, needle_end, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(compass_overlay, center, 3, (0, 0, 255), -1, cv2.LINE_AA)

    font_scale = 0.45
    cv2.putText(compass_overlay, "N", (center[0] - 6, center[1] - radius_outer - 6), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "E", (center[0] + radius_outer + 4, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "S", (center[0] - 6, center[1] + radius_outer + 14), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "W", (center[0] - radius_outer - 14, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)

    if heading_deg is not None:
        heading_360 = heading_deg % 360.0
        cardinal = compass_cardinal_from_heading_deg(heading_360)
        label = f"{cardinal} {heading_360:05.1f}"
    else:
        label = "COMPASS"
    if age_s is not None:
        label = f"{label} {age_s:.1f}s"
    cv2.putText(compass_overlay, label, (center[0] - radius_outer, center[1] + radius_outer + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)

    return cv2.addWeighted(frame, 1.0, compass_overlay, 0.85, 0)


def draw_imu_analysis_widget(frame, imu_roll_deg, imu_pitch_deg, imu_yaw_deg, accel_roll_deg, accel_pitch_deg, accel_tilt_deg):
    h, w = frame.shape[:2]
    box_w = min(300, max(220, int(w * 0.34)))
    box_h = 166
    x0 = w - box_w - 14
    y0 = min(max(130, h // 5), h - box_h - 14)

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    cv2.putText(frame, "IMU / ACCEL ANALYSIS", (x0 + 10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

    def draw_signed_bar(label, value_deg, row, color):
        y = y0 + 38 + (row * 22)
        bar_x0 = x0 + 104
        bar_x1 = x0 + box_w - 12
        center_x = (bar_x0 + bar_x1) // 2
        span = (bar_x1 - bar_x0) // 2
        v = float(np.clip(value_deg, -180.0, 180.0))
        end_x = int(center_x + (v / 180.0) * span)

        cv2.putText(frame, f"{label} {value_deg:+6.1f}", (x0 + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, y), (bar_x1, y), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y - 4), (center_x, y + 4), (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y), (end_x, y), color, 2, cv2.LINE_AA)

    draw_signed_bar("IMU R", imu_roll_deg, 0, (0, 180, 255))
    draw_signed_bar("IMU P", imu_pitch_deg, 1, (0, 180, 255))
    draw_signed_bar("IMU Y", imu_yaw_deg, 2, (0, 180, 255))
    draw_signed_bar("ACC R", accel_roll_deg, 3, (0, 255, 255))
    draw_signed_bar("ACC P", accel_pitch_deg, 4, (0, 255, 255))
    draw_signed_bar("A TLT", accel_tilt_deg, 5, (0, 255, 140))

    return frame


def blend_value(current_value, target_value, blend):
    return current_value + ((target_value - current_value) * blend)


def accel_to_roll_pitch(ax_g, ay_g, az_g):
    magnitude = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
    if magnitude < 0.1:
        return 0.0, 0.0

    ax_g /= magnitude
    ay_g /= magnitude
    az_g /= magnitude

    roll_deg = math.degrees(math.atan2(ay_g, az_g))
    pitch_deg = math.degrees(math.atan2(-ax_g, math.sqrt((ay_g * ay_g) + (az_g * az_g))))
    return normalize_angle_deg(roll_deg), normalize_angle_deg(pitch_deg)


def calibrate_gyro_bias(bus, sample_count=200):
    gx_total = 0.0
    gy_total = 0.0
    gz_total = 0.0

    for _ in range(sample_count):
        gx_total += read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H) / GYRO_LSB_PER_DPS
        gy_total += read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H) / GYRO_LSB_PER_DPS
        gz_total += read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H) / GYRO_LSB_PER_DPS
        time.sleep(0.01)

    gyro_bias["x"] = gx_total / sample_count
    gyro_bias["y"] = gy_total / sample_count
    gyro_bias["z"] = gz_total / sample_count
    print(
        "Calibrated gyro bias: "
        f"x={gyro_bias['x']:.4f} dps, "
        f"y={gyro_bias['y']:.4f} dps, "
        f"z={gyro_bias['z']:.4f} dps"
    )


def open_compass_shared_memory(wait_interval=0.5):
    while True:
        try:
            return shared_memory.SharedMemory(name=COMPASS_SHM_NAME)
        except FileNotFoundError:
            time.sleep(wait_interval)


def release_compass_shared_memory(shm):
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass

    try:
        shm.close()
    except Exception:
        pass


def read_latest_compass_sample(shm):
    magic, write_index, sample_count = struct.unpack_from(COMPASS_SHM_HEADER_FORMAT, shm.buf, 0)
    if magic != COMPASS_SHM_MAGIC or sample_count == 0:
        return None

    latest_index = (write_index - 1) % COMPASS_MAX_SAMPLES
    record_offset = COMPASS_SHM_HEADER_SIZE + (latest_index * COMPASS_SHM_RECORD_SIZE)
    timestamp, raw_heading, heading, x, y, z = struct.unpack_from(COMPASS_SHM_RECORD_FORMAT, shm.buf, record_offset)
    return {
        "timestamp": timestamp,
        "raw_heading": raw_heading,
        "heading": heading,
        "x": x,
        "y": y,
        "z": z,
        "sample_count": sample_count,
        "latest_index": latest_index,
    }


def sample_is_fresh(sample, freshness_threshold):
    return sample is not None and (time.time() - sample["timestamp"]) <= freshness_threshold


def compass_reader_thread():
    global compass_stream_shm

    last_seen = None
    while True:
        if compass_stream_shm is None:
            compass_stream_shm = open_compass_shared_memory()
            last_seen = None

        try:
            sample = read_latest_compass_sample(compass_stream_shm)
            if not sample_is_fresh(sample, COMPASS_FRESHNESS_THRESHOLD_S):
                with compass_lock:
                    compass_state["heading_deg"] = None
                    compass_state["timestamp"] = 0.0
                    compass_state["last_update"] = time.time()

                last_seen = None
                release_compass_shared_memory(compass_stream_shm)
                compass_stream_shm = None
                time.sleep(0.1)
                continue

            snapshot = (
                sample["timestamp"],
                sample["raw_heading"],
                sample["heading"],
                sample["x"],
                sample["y"],
                sample["z"],
                sample["sample_count"],
            )
            if snapshot != last_seen:
                with compass_lock:
                    compass_state["heading_deg"] = invert_compass_heading_deg(sample["heading"])
                    compass_state["timestamp"] = sample["timestamp"]
                    compass_state["last_update"] = time.time()
                last_seen = snapshot

            time.sleep(0.02)
        except (FileNotFoundError, OSError):
            last_seen = None
            release_compass_shared_memory(compass_stream_shm)
            compass_stream_shm = None
            time.sleep(0.1)
        except Exception as e:
            print(f"Compass reader error: {e}")
            last_seen = None
            release_compass_shared_memory(compass_stream_shm)
            compass_stream_shm = None
            time.sleep(0.1)


def start_distance_sensor_reader():
    def mavlink_reader():
        try:
            master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)
            master.wait_heartbeat()
            print(f"Connected to MAVLink on {MAVLINK_CONNECTION_STRING}")

            while True:
                msg = master.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue

                msg_type = msg.get_type()
                now = time.time()

                if msg_type == "DISTANCE_SENSOR":
                    with distance_lock:
                        distance_state["current_distance"] = msg.current_distance
                        distance_state["last_update"] = now
        except Exception as e:
            print(f"MAVLink reader error: {e}")

    def imu_reader():
        try:
            bus = SMBus(IMU_I2C_BUS)
            bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
            time.sleep(0.2)
            print(f"Connected to I2C MPU6050 on bus {IMU_I2C_BUS} address 0x{IMU_I2C_ADDR:02X}")

            calibrate_gyro_bias(bus)

            last_imu_ts = None
            while True:
                now = time.time()
                ax_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_XOUT_H)
                ay_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_YOUT_H)
                az_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_ZOUT_H)
                gx_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H)
                gy_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H)
                gz_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H)

                xaccel_g = ax_raw / ACCEL_LSB_PER_G
                yaccel_g = ay_raw / ACCEL_LSB_PER_G
                zaccel_g = az_raw / ACCEL_LSB_PER_G
                xgyro_dps = (gx_raw / GYRO_LSB_PER_DPS) - gyro_bias["x"]
                ygyro_dps = (gy_raw / GYRO_LSB_PER_DPS) - gyro_bias["y"]
                zgyro_dps = (gz_raw / GYRO_LSB_PER_DPS) - gyro_bias["z"]

                roll_accel_deg, pitch_accel_deg = accel_to_roll_pitch(xaccel_g, yaccel_g, zaccel_g)

                with accel_lock:
                    accel_state["x_g"] = blend_value(accel_state["x_g"], xaccel_g, 0.2)
                    accel_state["y_g"] = blend_value(accel_state["y_g"], yaccel_g, 0.2)
                    accel_state["z_g"] = blend_value(accel_state["z_g"], zaccel_g, 0.2)
                    accel_state["roll_deg"] = roll_accel_deg
                    accel_state["pitch_deg"] = pitch_accel_deg
                    accel_state["tilt_deg"] = math.degrees(
                        math.atan2(
                            math.sqrt((xaccel_g * xaccel_g) + (yaccel_g * yaccel_g)),
                            max(1e-6, abs(zaccel_g)),
                        )
                    )
                    accel_state["last_update"] = now

                with attitude_lock:
                    attitude_state["xgyro_dps"] = xgyro_dps
                    attitude_state["ygyro_dps"] = ygyro_dps
                    attitude_state["zgyro_dps"] = zgyro_dps

                    if last_imu_ts is not None:
                        dt = now - last_imu_ts
                        if 0 < dt < 0.1:
                            roll_gyro_deg = attitude_state["roll_deg"] + (xgyro_dps * dt)
                            pitch_gyro_deg = attitude_state["pitch_deg"] + (ygyro_dps * dt)
                            yaw_gyro_deg = attitude_state["yaw_deg"] + (zgyro_dps * dt)

                            compass_heading_deg = None
                            compass_age_s = None
                            with compass_lock:
                                if compass_state["heading_deg"] is not None:
                                    compass_heading_deg = compass_state["heading_deg"]
                                    compass_age_s = time.time() - compass_state["timestamp"]

                            attitude_state["roll_deg"] = normalize_angle_deg(
                                (COMPLEMENTARY_FILTER_ALPHA * roll_gyro_deg)
                                + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel_deg)
                            )
                            attitude_state["pitch_deg"] = normalize_angle_deg(
                                (COMPLEMENTARY_FILTER_ALPHA * pitch_gyro_deg)
                                + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel_deg)
                            )
                            if compass_heading_deg is not None and compass_age_s is not None and compass_age_s <= COMPASS_FRESHNESS_THRESHOLD_S:
                                attitude_state["yaw_deg"] = blend_angle_deg(
                                    yaw_gyro_deg,
                                    compass_heading_deg,
                                    1.0 - COMPLEMENTARY_FILTER_ALPHA,
                                )
                            else:
                                attitude_state["yaw_deg"] = normalize_angle_deg(yaw_gyro_deg)

                    attitude_state["last_update"] = now

                last_imu_ts = now
                time.sleep(0.02)
        except Exception as e:
            print(f"I2C IMU reader error: {e}")

    distance_thread = threading.Thread(target=mavlink_reader, daemon=True)
    distance_thread.start()

    imu_thread = threading.Thread(target=imu_reader, daemon=True)
    imu_thread.start()

    compass_thread = threading.Thread(target=compass_reader_thread, daemon=True)
    compass_thread.start()

    return distance_thread, imu_thread, compass_thread


def draw_osd(frame):
    with distance_lock:
        current_distance = distance_state["current_distance"]
    with attitude_lock:
        roll_deg = attitude_state["roll_deg"]
        pitch_deg = attitude_state["pitch_deg"]
        yaw_deg = attitude_state["yaw_deg"]
        xgyro_dps = attitude_state["xgyro_dps"]
        ygyro_dps = attitude_state["ygyro_dps"]
        zgyro_dps = attitude_state["zgyro_dps"]
    with accel_lock:
        xaccel_g = accel_state["x_g"]
        yaccel_g = accel_state["y_g"]
        zaccel_g = accel_state["z_g"]
        accel_roll_deg = accel_state["roll_deg"]
        accel_pitch_deg = accel_state["pitch_deg"]
        accel_tilt_deg = accel_state["tilt_deg"]
    with compass_lock:
        compass_heading_deg = compass_state["heading_deg"]
        compass_timestamp = compass_state["timestamp"]

    vx_mps = velocity_state["vx_mps"]
    vy_mps = velocity_state["vy_mps"]
    speed_mps = velocity_state["speed_mps"]
    inliers = velocity_state["inliers"]

    lines = [
        f"DIST: {current_distance if current_distance is not None else 'N/A'} cm",
        # f"VX: {vx_mps:+.3f} m/s",
        # f"VY: {vy_mps:+.3f} m/s",
        # f"SPD: {speed_mps:.3f} m/s ({inliers} inliers)",
        f"GYRO: X:{xgyro_dps:+.1f} Y:{ygyro_dps:+.1f} Z:{zgyro_dps:+.1f} dps",
        f"ACCEL: X:{xaccel_g:+.2f}g Y:{yaccel_g:+.2f}g Z:{zaccel_g:+.2f}g",
        f"ROLL: {roll_deg:+.1f} deg",
        f"PITCH: {pitch_deg:+.1f} deg",
        f"YAW: {yaw_deg:+.1f} deg",
    ]

    overlay = frame.copy()
    x, y = 12, 28
    box_w = 0
    for line in lines:
        (text_w, text_h), baseline = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        box_w = max(box_w, text_w)

    box_h = 18 * len(lines) + 14
    cv2.rectangle(overlay, (8, 8), (20 + box_w, 12 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    for line in lines:
        # highlight attitude values with brighter color for quick visual cue
        color = (0, 255, 0)
        if line.startswith("ROLL") or line.startswith("PITCH") or line.startswith("YAW"):
            color = (0, 200, 255)
        elif line.startswith("ACCEL"):
            color = (255, 200, 0)
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        y += 18

    compass_age_s = None
    if compass_timestamp:
        compass_age_s = max(0.0, time.time() - compass_timestamp)
    frame = draw_compass_widget(frame, compass_heading_deg, compass_age_s)
    frame = draw_imu_analysis_widget(
        frame,
        roll_deg,
        pitch_deg,
        yaw_deg,
        accel_roll_deg,
        accel_pitch_deg,
        accel_tilt_deg,
    )

    return frame


def draw_ground_reticle(frame):
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    radius_outer = max(24, min(w, h) // 7)
    radius_inner = max(12, radius_outer // 2)
    arm = max(16, radius_outer // 2)

    with attitude_lock:
        roll_deg = attitude_state["roll_deg"]
        pitch_deg = attitude_state["pitch_deg"]
        yaw_deg = attitude_state["yaw_deg"]
    with accel_lock:
        xaccel_g = accel_state["x_g"]
        yaccel_g = accel_state["y_g"]

    # increase sensitivity: amplify pitch and roll movement, and allow larger clamp
    pitch_px = int(np.clip(-pitch_deg * 3.0, -h * 0.3, h * 0.3))
    roll_px = int(np.clip(roll_deg * 2.0, -w * 0.3, w * 0.3))
    center = (cx + roll_px, cy + pitch_px)

    overlay = frame.copy()
    rotation_matrix = cv2.getRotationMatrix2D(center, -yaw_deg, 1.0)

    reticle = np.zeros_like(frame)
    cv2.circle(reticle, center, radius_outer, RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.circle(reticle, center, radius_inner, RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] - radius_outer - arm, center[1]), (center[0] + radius_outer + arm, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0], center[1] - radius_outer - arm), (center[0], center[1] + radius_outer + arm), RETICLE_COLOR, 1, cv2.LINE_AA)

    tick = max(6, radius_outer // 5)
    cv2.line(reticle, (center[0], center[1] - radius_outer), (center[0], center[1] - radius_outer - tick), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] + radius_outer, center[1]), (center[0] + radius_outer + tick, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0], center[1] + radius_outer), (center[0], center[1] + radius_outer + tick), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] - radius_outer, center[1]), (center[0] - radius_outer - tick, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)

    reticle = cv2.warpAffine(reticle, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    frame = cv2.addWeighted(frame, 1.0, reticle, 0.9, 0)

    arrow_scale = 45.0
    arrow_dx = int(np.clip(xaccel_g * arrow_scale, -w * 0.25, w * 0.25))
    arrow_dy = int(np.clip(-yaccel_g * arrow_scale, -h * 0.25, h * 0.25))
    arrow_end = (cx + arrow_dx, cy + arrow_dy)
    cv2.arrowedLine(frame, (cx, cy), arrow_end, (255, 200, 0), 3, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(frame, (cx, cy), 4, (255, 200, 0), -1, cv2.LINE_AA)

    return frame


def choose_output_mode():
    default_mode = "record"
    prompt = "Choose output mode: [r]ecord locally or [s]tream via RTSP? [r/s] "
    try:
        choice = input(prompt).strip().lower()
    except EOFError:
        choice = default_mode

    if choice in ("s", "stream", "rtsp"):
        return "rtsp"
    return default_mode


def choose_rtsp_url():
    default_url = "drone"
    try:
        value = input(f"RTSP stream name [{default_url}]: ").strip()
    except EOFError:
        value = ""
    return value or default_url


class FileSink:
    def __init__(self, frame_size, fps):
        RECORDINGS_DIR.mkdir(exist_ok=True)
        self.output_path = RECORDINGS_DIR / f"optical_flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, fps, frame_size)

        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.output_path}")

        print(f"Recording optical flow to {self.output_path}")

    def write(self, frame):
        self.writer.write(frame)

    def release(self):
        self.writer.release()
        print(f"Saved recording: {self.output_path}")


class RtspServer:
    def __init__(self):
        if not MEDIAMTX_BIN.exists():
            raise RuntimeError(f"MediaMTX binary not found at {MEDIAMTX_BIN}")

        self.process = subprocess.Popen(
            [str(MEDIAMTX_BIN)],
            cwd=str(MEDIAMTX_BIN.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        time.sleep(1)
        if self.process.poll() is not None:
            raise RuntimeError("MediaMTX failed to start")

    def release(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except Exception:
                self.process.kill()


class RtspSink:
    def __init__(self, frame_size, fps, rtsp_url):
        frame_width, frame_height = frame_size
        self.rtsp_url = rtsp_url
        self.failed = False
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{frame_width}x{frame_height}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-pix_fmt",
                "yuv420p",
                "-f",
                "rtsp",
                "-rtsp_transport",
                "tcp",
                rtsp_url,
            ],
            stdin=subprocess.PIPE,
        )

        print(f"Streaming optical flow to {rtsp_url}")

    def write(self, frame):
        if self.failed:
            return
        if self.process.stdin is None:
            self.failed = True
            raise RuntimeError("RTSP stream is not writable")

        if self.process.poll() is not None:
            self.failed = True
            raise RuntimeError(f"RTSP server stopped for {self.rtsp_url}")

        try:
            self.process.stdin.write(frame.tobytes())
        except BrokenPipeError as exc:
            self.failed = True
            raise RuntimeError(f"RTSP stream dropped for {self.rtsp_url}") from exc
        except Exception:
            self.failed = True
            raise

    def is_failed(self):
        return self.failed

    def release(self):
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except Exception:
                pass

        try:
            self.process.wait(timeout=5)
        except Exception:
            self.process.kill()


def create_output_sink(frame_size, fps):
    mode = choose_output_mode()
    if mode == "rtsp":
        stream_name = choose_rtsp_url()
        server = RtspServer()
        publish_url = f"rtsp://127.0.0.1:8554/{stream_name}"
        print(f"Open this on Windows: rtsp://<drone-ip>:8554/{stream_name}")
        return RtspSink(frame_size, fps, publish_url), True, server
    return FileSink(frame_size, fps), False, None


def focal_length_px(frame_width):
    return frame_width / (2.0 * math.tan(math.radians(CAMERA_HORIZONTAL_FOV_DEG / 2.0)))


def reject_outlier_tracks(good_old, good_new):
    good_old = np.asarray(good_old, dtype=np.float32).reshape(-1, 2)
    good_new = np.asarray(good_new, dtype=np.float32).reshape(-1, 2)

    if len(good_old) != len(good_new):
        pair_count = min(len(good_old), len(good_new))
        good_old = good_old[:pair_count]
        good_new = good_new[:pair_count]

    if len(good_new) < MIN_INLIERS_FOR_VELOCITY:
        return good_old, good_new

    motion = (good_new - good_old).astype(np.float32)
    magnitudes = np.linalg.norm(motion, axis=1)
    basic_mask = magnitudes < MAX_FLOW_STEP_PX
    if np.count_nonzero(basic_mask) < MIN_INLIERS_FOR_VELOCITY:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    old_filtered = good_old[basic_mask]
    new_filtered = good_new[basic_mask]
    affine_result = cv2.estimateAffinePartial2D(old_filtered, new_filtered, method=cv2.RANSAC, ransacReprojThreshold=2.0)
    if affine_result is None:
        return old_filtered, new_filtered

    _, inlier_mask = affine_result
    if inlier_mask is None:
        return old_filtered, new_filtered

    inlier_mask = inlier_mask.ravel().astype(bool)
    if np.count_nonzero(inlier_mask) < MIN_INLIERS_FOR_VELOCITY:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    return old_filtered[inlier_mask], new_filtered[inlier_mask]


def estimate_body_velocity_mps(good_old, good_new, altitude_cm, dt_s, fx_px, fy_px):
    if altitude_cm is None or altitude_cm <= 0 or dt_s <= 0:
        return None
    if len(good_new) < MIN_INLIERS_FOR_VELOCITY:
        return None

    displacement = good_new - good_old
    median_dx_px = float(np.median(displacement[:, 0]))
    median_dy_px = float(np.median(displacement[:, 1]))

    altitude_m = altitude_cm / 100.0
    vx_mps = (median_dx_px * altitude_m) / (fx_px * dt_s)
    vy_mps = (median_dy_px * altitude_m) / (fy_px * dt_s)
    return vx_mps, vy_mps

old_frame = ensure_bgr(picam2.capture_array())
old_gray = to_small_gray(old_frame)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

start_distance_sensor_reader()

frame_height, frame_width = old_frame.shape[:2]
focal_length_x_px = focal_length_px(frame_width)
focal_length_y_px = focal_length_x_px
output_sink, rtsp_selected, rtsp_server = create_output_sink((frame_width, frame_height), TARGET_FPS)
fallback_sink = None
print("Press Ctrl+C to stop")


def switch_to_file_sink(frame_size, fps, reason):
    global fallback_sink, output_sink, rtsp_selected, rtsp_server
    if fallback_sink is None:
        print(reason)
        fallback_sink = FileSink(frame_size, fps)
    output_sink = fallback_sink
    rtsp_selected = False

def record_optical_flow():
    global old_gray, p0, output_sink
    next_frame_time = time.perf_counter()
    previous_frame_ts = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now < next_frame_time:
            time.sleep(next_frame_time - now)
        frame_ts = time.perf_counter()
        dt_s = frame_ts - previous_frame_ts
        previous_frame_ts = frame_ts
        next_frame_time = frame_ts + FRAME_INTERVAL_S

        frame = ensure_bgr(picam2.capture_array())
        frame_gray = to_small_gray(frame)
        img = frame.copy()
        draw_scale = 1.0 / FLOW_SCALE

        if p0 is not None and len(p0) > 0:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            if p1 is not None and st is not None:
                good_new = p1[st.flatten() == 1]
                good_old = p0[st.flatten() == 1]
                inlier_old, inlier_new = reject_outlier_tracks(good_old, good_new)

                with distance_lock:
                    altitude_cm = distance_state["current_distance"]

                velocity_estimate = estimate_body_velocity_mps(
                    inlier_old,
                    inlier_new,
                    altitude_cm,
                    dt_s,
                    focal_length_x_px,
                    focal_length_y_px,
                )

                if velocity_estimate is not None:
                    vx_mps, vy_mps = velocity_estimate
                    velocity_state["vx_mps"] = vx_mps
                    velocity_state["vy_mps"] = vy_mps
                    velocity_state["speed_mps"] = float(math.hypot(vx_mps, vy_mps))
                    velocity_state["inliers"] = len(inlier_new)
                    velocity_state["last_update"] = frame_ts
                else:
                    velocity_state["vx_mps"] = 0.0
                    velocity_state["vy_mps"] = 0.0
                    velocity_state["speed_mps"] = 0.0
                    velocity_state["inliers"] = 0
                    velocity_state["last_update"] = frame_ts

                for new, old in zip(inlier_new, inlier_old):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    ax, by = int(a * draw_scale), int(b * draw_scale)
                    cx, dy = int(c * draw_scale), int(d * draw_scale)
                    img = cv2.line(img, (cx, dy), (ax, by), (0, 255, 0), 1)
                    img = cv2.circle(img, (ax, by), 3, (0, 0, 255), -1)

                if len(good_new) < TRACK_FEATURE_COUNT:
                    new_features = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                    if new_features is not None:
                        good_new = np.concatenate((good_new.reshape(-1, 2), new_features.reshape(-1, 2)), axis=0)

                if len(good_new) > TRACK_FEATURE_COUNT:
                    good_new = good_new[:TRACK_FEATURE_COUNT]

                if len(good_new) > 0:
                    p0 = good_new.reshape(-1, 1, 2)
                else:
                    p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
            else:
                p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        else:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)

        old_gray = frame_gray.copy()
        img = draw_ground_reticle(img)
        img = draw_osd(img)
        try:
            output_sink.write(img)
        except RuntimeError as exc:
            if rtsp_selected:
                switch_to_file_sink((frame_width, frame_height), TARGET_FPS, f"RTSP failed, falling back to local recording: {exc}")
                output_sink.write(img)
            else:
                raise

if __name__ == '__main__':
    try:
        record_optical_flow()
    except KeyboardInterrupt:
        print("\nStopping output...")
    finally:
        output_sink.release()
        if rtsp_server is not None:
            rtsp_server.release()
        picam2.stop()
