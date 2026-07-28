import time
import math
import struct
import threading
import numpy as np
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
try:
    from smbus2 import SMBus
except (ImportError, ModuleNotFoundError):
    SMBus = None


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

ATTITUDE_SHM_NAME = "drone_attitude_stream"
ATTITUDE_SHM_MAGIC = b"ATT "
ATTITUDE_SHM_HEADER_FORMAT = "<4sII"
ATTITUDE_SHM_RECORD_FORMAT = "<7d"
ATTITUDE_SHM_HEADER_SIZE = struct.calcsize(ATTITUDE_SHM_HEADER_FORMAT)
ATTITUDE_SHM_RECORD_SIZE = struct.calcsize(ATTITUDE_SHM_RECORD_FORMAT)
ATTITUDE_MAX_SAMPLES = 120
ATTITUDE_SHM_SIZE = ATTITUDE_SHM_HEADER_SIZE + (ATTITUDE_MAX_SAMPLES * ATTITUDE_SHM_RECORD_SIZE)

attitude_shm = None
attitude_shm_lock = threading.Lock()


def attach_attitude_shm():
    """
    Attaches to or creates the shared memory block for drone attitude stream data.
    """
    global attitude_shm
    with attitude_shm_lock:
        if attitude_shm is not None:
            return attitude_shm
        try:
            attitude_shm = shared_memory.SharedMemory(name=ATTITUDE_SHM_NAME, create=True, size=ATTITUDE_SHM_SIZE)
        except FileExistsError:
            attitude_shm = shared_memory.SharedMemory(name=ATTITUDE_SHM_NAME, create=False)
            if attitude_shm.size < ATTITUDE_SHM_SIZE:
                attitude_shm.close()
                try:
                    attitude_shm.unlink()
                except FileNotFoundError:
                    pass
                attitude_shm = shared_memory.SharedMemory(name=ATTITUDE_SHM_NAME, create=True, size=ATTITUDE_SHM_SIZE)

        try:
            resource_tracker.unregister(attitude_shm._name, "shared_memory")
        except Exception:
            pass

        struct.pack_into(ATTITUDE_SHM_HEADER_FORMAT, attitude_shm.buf, 0, ATTITUDE_SHM_MAGIC, 0, 0)
        return attitude_shm


def write_attitude_sample(timestamp, roll_deg, pitch_deg, yaw_deg, xgyro_dps, ygyro_dps, zgyro_dps):
    """
    Writes a single attitude sample (roll, pitch, yaw, gyro rates) into drone_attitude_stream SHM.
    """
    try:
        shm = attach_attitude_shm()
        with attitude_shm_lock:
            _, write_index, sample_count = struct.unpack_from(ATTITUDE_SHM_HEADER_FORMAT, shm.buf, 0)
            record_offset = ATTITUDE_SHM_HEADER_SIZE + (write_index * ATTITUDE_SHM_RECORD_SIZE)
            struct.pack_into(
                ATTITUDE_SHM_RECORD_FORMAT,
                shm.buf,
                record_offset,
                float(timestamp),
                float(roll_deg),
                float(pitch_deg),
                float(yaw_deg),
                float(xgyro_dps),
                float(ygyro_dps),
                float(zgyro_dps),
            )
            write_index = (write_index + 1) % ATTITUDE_MAX_SAMPLES
            sample_count = min(sample_count + 1, ATTITUDE_MAX_SAMPLES)
            struct.pack_into(ATTITUDE_SHM_HEADER_FORMAT, shm.buf, 0, ATTITUDE_SHM_MAGIC, write_index, sample_count)
    except Exception as exc:
        pass

gyro_calibrated = False

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

gyro_integrated_state = {
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
}
gyro_integrated_lock = threading.Lock()

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


def read_i2c_word(bus, addr, reg):
    """
    Reads a signed 16-bit word from the I2C register.
    """
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def normalize_angle_deg(angle_deg):
    """
    Normalizes a given angle in degrees to the range [-180, 180].
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0


def angular_error_deg(target_deg, current_deg):
    """
    Calculates the shortest angular difference in degrees between two angles.
    """
    return normalize_angle_deg(target_deg - current_deg)


def blend_angle_deg(current_deg, target_deg, blend):
    """
    Blends/interpolates between two angles in degrees using a blend factor [0, 1].
    """
    return normalize_angle_deg(current_deg + (blend * angular_error_deg(target_deg, current_deg)))


def invert_compass_heading_deg(heading_deg):
    """
    Inverts the sign of a compass heading and normalizes it to [-180, 180].
    """
    return normalize_angle_deg(-heading_deg)


def blend_value(current_value, target_value, blend):
    """
    Linearly interpolates between two numeric values using a blend factor.
    """
    return current_value + ((target_value - current_value) * blend)


def accel_to_roll_pitch(ax_g, ay_g, az_g):
    """
    Calculates roll and pitch angles in degrees directly from accelerometer gravity vectors.
    """
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
    """
    Calculates the gyroscope zero-rate offset by averaging samples while the drone/sensor is stationary.
    """
    global gyro_calibrated
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
    gyro_calibrated = True
    print(
        "Calibrated gyro bias: "
        f"x={gyro_bias['x']:.4f} dps, "
        f"y={gyro_bias['y']:.4f} dps, "
        f"z={gyro_bias['z']:.4f} dps"
    )


def open_compass_shared_memory(wait_interval=0.5):
    """
    Tries to connect to the compass shared memory block. Retries periodically until successful.
    """
    while True:
        try:
            shm = shared_memory.SharedMemory(name=COMPASS_SHM_NAME)
            try:
                from multiprocessing import resource_tracker
                resource_tracker.unregister(shm._name, "shared_memory")
            except Exception:
                pass
            return shm
        except FileNotFoundError:
            time.sleep(wait_interval)


def release_compass_shared_memory(shm):
    """
    Closes the compass shared memory segment.
    """
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass

    try:
        shm.close()
    except Exception:
        pass


def read_latest_compass_sample(shm):
    """
    Reads the latest sample records (timestamp, coordinates, heading) from compass shared memory.
    """
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
    """
    Checks if a sample timestamp is within the freshness threshold.
    """
    return sample is not None and (time.time() - sample["timestamp"]) <= freshness_threshold


def compass_reader_thread():
    """
    Thread runner that continuously monitors the compass shared memory segment and copies values to local states.
    """
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
    """
    Starts concurrent threads to read IMU data from the I2C MPU6050,
    and compass data from shared memory.
    """

    def imu_reader():
        """
        Background reader that polls the MPU6050, integrates gyro angular rates,
        and applies a complementary filter.
        """
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
                            # 3D Angular Kinematics: Convert body gyro rates (p, q, r) to Euler angle rates (phi_dot, theta_dot, psi_dot)
                            curr_roll_rad = math.radians(attitude_state["roll_deg"])
                            curr_pitch_rad = math.radians(attitude_state["pitch_deg"])
                            cos_phi = math.cos(curr_roll_rad)
                            sin_phi = math.sin(curr_roll_rad)
                            cos_theta = max(0.01, math.cos(curr_pitch_rad))
                            tan_theta = math.tan(curr_pitch_rad)

                            roll_rate_dps = xgyro_dps + ((ygyro_dps * sin_phi + zgyro_dps * cos_phi) * tan_theta)
                            pitch_rate_dps = (ygyro_dps * cos_phi) - (zgyro_dps * sin_phi)
                            yaw_rate_dps = (ygyro_dps * sin_phi + zgyro_dps * cos_phi) / cos_theta

                            with gyro_integrated_lock:
                                gyro_integrated_state["roll_deg"] = normalize_angle_deg(
                                    gyro_integrated_state["roll_deg"] + (roll_rate_dps * dt)
                                )
                                gyro_integrated_state["pitch_deg"] = normalize_angle_deg(
                                    gyro_integrated_state["pitch_deg"] + (pitch_rate_dps * dt)
                                )
                                gyro_integrated_state["yaw_deg"] = normalize_angle_deg(
                                    gyro_integrated_state["yaw_deg"] + (yaw_rate_dps * dt)
                                )

                            roll_gyro_deg = attitude_state["roll_deg"] + (roll_rate_dps * dt)
                            pitch_gyro_deg = attitude_state["pitch_deg"] + (pitch_rate_dps * dt)
                            yaw_gyro_deg = attitude_state["yaw_deg"] + (yaw_rate_dps * dt)

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

                    # Stream live roll, pitch, yaw and rates to shared memory
                    write_attitude_sample(
                        now,
                        attitude_state["roll_deg"],
                        attitude_state["pitch_deg"],
                        attitude_state["yaw_deg"],
                        attitude_state["xgyro_dps"],
                        attitude_state["ygyro_dps"],
                        attitude_state["zgyro_dps"],
                    )

                last_imu_ts = now
                time.sleep(0.02)
        except Exception as e:
            print(f"I2C IMU reader error: {e}")

    imu_thread = threading.Thread(target=imu_reader, daemon=True)
    imu_thread.start()

    compass_thread = threading.Thread(target=compass_reader_thread, daemon=True)
    compass_thread.start()

    return imu_thread, compass_thread


def is_gyro_calibrated():
    """
    Returns True if the gyroscope bias calibration has completed.
    """
    return gyro_calibrated
