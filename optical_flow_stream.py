# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink, smbus2

import cv2
import numpy as np
import threading
import subprocess
import math
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
GYRO_I2C_BUS = 1
GYRO_I2C_ADDR = 0x68
GYRO_PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
GYRO_LSB_PER_DPS = 131.0

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


def start_distance_sensor_reader():
    def distance_reader():
        try:
            master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)
            master.wait_heartbeat()
            print(f"Connected to MAVLink on {MAVLINK_CONNECTION_STRING}")

            while True:
                msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)
                if msg is None:
                    continue

                now = time.time()

                with distance_lock:
                    distance_state["current_distance"] = msg.current_distance
                    distance_state["last_update"] = now
        except Exception as e:
            print(f"MAVLink distance reader error: {e}")

    def gyro_reader():
        try:
            bus = SMBus(GYRO_I2C_BUS)
            bus.write_byte_data(GYRO_I2C_ADDR, GYRO_PWR_MGMT_1, 0)
            time.sleep(0.2)
            print(f"Connected to I2C gyro on bus {GYRO_I2C_BUS} address 0x{GYRO_I2C_ADDR:02X}")

            last_imu_ts = None
            while True:
                now = time.time()
                gx_raw = read_i2c_word(bus, GYRO_I2C_ADDR, GYRO_XOUT_H)
                gy_raw = read_i2c_word(bus, GYRO_I2C_ADDR, GYRO_YOUT_H)
                gz_raw = read_i2c_word(bus, GYRO_I2C_ADDR, GYRO_ZOUT_H)

                xgyro_dps = gx_raw / GYRO_LSB_PER_DPS
                ygyro_dps = gy_raw / GYRO_LSB_PER_DPS
                zgyro_dps = gz_raw / GYRO_LSB_PER_DPS

                with attitude_lock:
                    attitude_state["xgyro_dps"] = xgyro_dps
                    attitude_state["ygyro_dps"] = ygyro_dps
                    attitude_state["zgyro_dps"] = zgyro_dps

                    if last_imu_ts is not None:
                        dt = now - last_imu_ts
                        if dt > 0:
                            attitude_state["roll_deg"] += xgyro_dps * dt
                            attitude_state["pitch_deg"] += ygyro_dps * dt
                            attitude_state["yaw_deg"] += zgyro_dps * dt

                            attitude_state["roll_deg"] = ((attitude_state["roll_deg"] + 180.0) % 360.0) - 180.0
                            attitude_state["pitch_deg"] = ((attitude_state["pitch_deg"] + 180.0) % 360.0) - 180.0
                            attitude_state["yaw_deg"] = ((attitude_state["yaw_deg"] + 180.0) % 360.0) - 180.0

                    attitude_state["last_update"] = now

                last_imu_ts = now
                time.sleep(0.02)
        except Exception as e:
            print(f"I2C gyro reader error: {e}")

    distance_thread = threading.Thread(target=distance_reader, daemon=True)
    distance_thread.start()

    gyro_thread = threading.Thread(target=gyro_reader, daemon=True)
    gyro_thread.start()

    return distance_thread, gyro_thread


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

    vx_mps = velocity_state["vx_mps"]
    vy_mps = velocity_state["vy_mps"]
    speed_mps = velocity_state["speed_mps"]
    inliers = velocity_state["inliers"]

    lines = [
        f"DIST: {current_distance if current_distance is not None else 'N/A'} cm",
        f"VX: {vx_mps:+.3f} m/s",
        f"VY: {vy_mps:+.3f} m/s",
        f"SPD: {speed_mps:.3f} m/s ({inliers} inliers)",
        f"GYRO: {xgyro_dps:+.1f} {ygyro_dps:+.1f} {zgyro_dps:+.1f} dps",
        f"ATT: {roll_deg:+.1f} {pitch_deg:+.1f} {yaw_deg:+.1f} deg",
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
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        y += 18

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

    pitch_px = int(np.clip(-pitch_deg * 2.0, -h * 0.2, h * 0.2))
    roll_px = int(np.clip(roll_deg * 1.2, -w * 0.2, w * 0.2))
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
    return cv2.addWeighted(frame, 1.0, reticle, 0.9, 0)


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
