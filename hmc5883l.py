#!/usr/bin/env python3
"""
HMC5883L Compass Dashboard
Real-time magnetic heading visualization via web interface.
"""

import json
import math
import struct
from multiprocessing import shared_memory
from pathlib import Path
import threading
import time
from collections import deque

from flask import Flask, jsonify, render_template
try:
    from smbus2 import SMBus
except (ImportError, ModuleNotFoundError):
    SMBus = None


HMC5883L_ADDR = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
DATA_X_MSB = 0x03
DATA_Z_MSB = 0x05
DATA_Y_MSB = 0x07

DECLINATION_DEGREES = 0.0
MAX_SAMPLES = 120
LOW_PASS_ALPHA = 0.85 # to toggle between more responsive (higher alpha) or smoother (lower alpha) heading readings
SHM_NAME = "compass_heading_stream" # Shared memory name for compass data stream
SHM_MAGIC = b"CHDG" # Magic bytes to identify valid shared memory segment
SHM_HEADER_FORMAT = "<4sII" # Header: magic (4s), write_index (I), sample_count (I)
SHM_RECORD_FORMAT = "<6d" # Record: timestamp (d), raw_heading (d), heading (d), x (d), y (d), z (d)
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT) # Size of the header in bytes
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT) # Size of each record in bytes
SHM_SIZE = SHM_HEADER_SIZE + (MAX_SAMPLES * SHM_RECORD_SIZE) # Total shared memory size to hold header and max samples

heading_zero_offset = 0.0
ZERO_OFFSET_FILE = Path(__file__).resolve().with_name("compass_zero_offset.json")

compass_data = {
    "timestamps": deque(maxlen=MAX_SAMPLES),
    "x": deque(maxlen=MAX_SAMPLES),
    "y": deque(maxlen=MAX_SAMPLES),
    "z": deque(maxlen=MAX_SAMPLES),
    "heading": deque(maxlen=MAX_SAMPLES),
}

latest_state = {
    "connected": False,
    "heading": None,
    "raw_heading": None,
    "x": None,
    "y": None,
    "z": None,
    "cardinal": "N",
    "zero_offset": 0.0,
    "error": None,
}

app = Flask(__name__)
data_lock = threading.Lock()
shared_memory_lock = threading.Lock()
heading_stream_shm = None


def normalize_heading(heading):
    """
    Normalizes a heading to the range [0, 360) degrees.
    """
    return heading % 360


def load_heading_zero_offset():
    """
    Loads the heading zero offset from the local configuration file (compass_zero_offset.json).
    Falls back to 0.0 if not found or invalid.
    """
    global heading_zero_offset

    try:
        with ZERO_OFFSET_FILE.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)

        offset = float(payload.get("zero_offset", 0.0))
    except FileNotFoundError:
        offset = 0.0
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        print(f"Compass zero offset load failed: {exc}")
        offset = 0.0

    with data_lock:
        heading_zero_offset = normalize_heading(offset)
        latest_state["zero_offset"] = round(heading_zero_offset, 2)


def save_heading_zero_offset():
    """
    Persists the current heading zero offset to compass_zero_offset.json.
    """
    with data_lock:
        payload = {"zero_offset": round(heading_zero_offset, 2)}

    tmp_file = ZERO_OFFSET_FILE.with_suffix(".tmp")
    try:
        with tmp_file.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle)
            handle.flush()
        tmp_file.replace(ZERO_OFFSET_FILE)
    except OSError as exc:
        print(f"Compass zero offset save failed: {exc}")


def attach_heading_stream_shm():
    """
    Attaches to or creates the shared memory segment for the compass heading stream.
    Initializes the header block if newly created.
    """
    global heading_stream_shm

    with shared_memory_lock:
        if heading_stream_shm is not None:
            return heading_stream_shm

        try:
            heading_stream_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        except FileExistsError:
            heading_stream_shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
            if heading_stream_shm.size < SHM_SIZE:
                heading_stream_shm.close()
                try:
                    heading_stream_shm.unlink()
                except FileNotFoundError:
                    pass
                heading_stream_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)

        try:
            from multiprocessing import resource_tracker
            resource_tracker.unregister(heading_stream_shm._name, "shared_memory")
        except Exception:
            pass

        struct.pack_into(SHM_HEADER_FORMAT, heading_stream_shm.buf, 0, SHM_MAGIC, 0, 0)
        return heading_stream_shm


def write_heading_stream_sample(timestamp, raw_heading, heading, x, y, z):
    """
    Writes a compass data sample (timestamp, raw heading, offset-corrected heading, and raw coordinates)
    into the circular buffer of the shared memory segment.
    """
    shm = attach_heading_stream_shm()

    with shared_memory_lock:
        _, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
        record_offset = SHM_HEADER_SIZE + (write_index * SHM_RECORD_SIZE)
        struct.pack_into(
            SHM_RECORD_FORMAT,
            shm.buf,
            record_offset,
            float(timestamp),
            float(raw_heading),
            float(heading),
            float(x),
            float(y),
            float(z),
        )

        write_index = (write_index + 1) % MAX_SAMPLES
        sample_count = min(sample_count + 1, MAX_SAMPLES)
        struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)


def close_heading_stream_shm():
    """
    Closes and unlinks the shared memory segment for the compass heading stream.
    """
    global heading_stream_shm

    with shared_memory_lock:
        if heading_stream_shm is None:
            return

        try:
            heading_stream_shm.close()
        finally:
            try:
                heading_stream_shm.unlink()
            except FileNotFoundError:
                pass
            heading_stream_shm = None


def read_word(bus, reg):
    """
    Reads a 16-bit signed word from the specified register of the HMC5883L magnetometer sensor.
    """
    high = bus.read_byte_data(HMC5883L_ADDR, reg)
    low = bus.read_byte_data(HMC5883L_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def get_current_attitude(bus=None):
    """
    Tries to retrieve the current roll and pitch attitude angles in degrees.
    First checks the 'drone_attitude_stream' shared memory segment.
    If unavailable or stale, attempts a direct MPU6050 accelerometer read via I2C (0x68).
    Falls back to (0.0, 0.0, 'none') if unreadable.
    """
    try:
        shm = shared_memory.SharedMemory(name="drone_attitude_stream")
        try:
            magic, write_index, sample_count = struct.unpack_from("<4sII", shm.buf, 0)
            if magic == b"ATT " and sample_count > 0:
                latest_index = (write_index - 1) % 120
                offset = struct.calcsize("<4sII") + (latest_index * struct.calcsize("<7d"))
                timestamp, roll_deg, pitch_deg, yaw_deg, gx, gy, gz = struct.unpack_from("<7d", shm.buf, offset)
                if (time.time() - timestamp) <= 1.0:
                    return roll_deg, pitch_deg, "shared_memory"
        finally:
            shm.close()
    except Exception:
        pass

    if bus is not None:
        try:
            high_x = bus.read_byte_data(0x68, 0x3B)
            low_x = bus.read_byte_data(0x68, 0x3C)
            ax_raw = (high_x << 8) | low_x
            if ax_raw >= 0x8000:
                ax_raw -= 65536

            high_y = bus.read_byte_data(0x68, 0x3D)
            low_y = bus.read_byte_data(0x68, 0x3E)
            ay_raw = (high_y << 8) | low_y
            if ay_raw >= 0x8000:
                ay_raw -= 65536

            high_z = bus.read_byte_data(0x68, 0x3F)
            low_z = bus.read_byte_data(0x68, 0x40)
            az_raw = (high_z << 8) | low_z
            if az_raw >= 0x8000:
                az_raw -= 65536

            ax = ax_raw / 16384.0
            ay = ay_raw / 16384.0
            az = az_raw / 16384.0
            mag = math.sqrt(ax * ax + ay * ay + az * az)
            if mag >= 0.1:
                roll_deg = math.degrees(math.atan2(ay, az))
                pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                return roll_deg, pitch_deg, "mpu6050_accel"
        except Exception:
            pass

    return 0.0, 0.0, "none"


def compute_heading(x, y, z=0.0, roll_deg=0.0, pitch_deg=0.0):
    """
    Calculates 3D tilt-compensated magnetic heading in degrees using 3D magnetic (x, y, z)
    coordinates and current pitch/roll attitude angles.
    Adjusts with DECLINATION_DEGREES and normalizes output to [0, 360).
    """
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    cos_roll = math.cos(roll_rad)
    sin_roll = math.sin(roll_rad)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)

    xh = (x * cos_pitch) + (y * sin_roll * sin_pitch) + (z * cos_roll * sin_pitch)
    yh = (y * cos_roll) - (z * sin_roll)

    heading = math.degrees(math.atan2(yh, xh))
    heading += DECLINATION_DEGREES
    return normalize_heading(heading)


def apply_zero_offset(heading):
    """
    Applies the calibrated zero offset correction to the given raw heading.
    """
    with data_lock:
        offset = heading_zero_offset
    return normalize_heading(heading - offset)


def low_pass_filter(previous_value, current_value, alpha=LOW_PASS_ALPHA):
    """
    Applies a simple first-order low-pass filter to smooth sensor readings.
    """
    if previous_value is None:
        return current_value
    return previous_value + (alpha * (current_value - previous_value))


def cardinal_from_heading(heading):
    """
    Converts a numerical heading in degrees to a cardinal/ordinal direction string (e.g., 'N', 'NE', 'E').
    """
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    index = int((heading + 22.5) // 45) % 8
    return directions[index]


def compass_thread_worker():
    """
    Background worker thread that connects to the HMC5883L magnetometer,
    periodically reads coordinates, applies filters, computes heading,
    and updates the shared memory stream.
    """
    bus = None
    filtered_x = None
    filtered_y = None
    filtered_z = None
    while True:
        try:
            if bus is None:
                bus = SMBus(1)
                bus.write_byte_data(HMC5883L_ADDR, CONFIG_A, 0x70)
                bus.write_byte_data(HMC5883L_ADDR, CONFIG_B, 0x20)
                bus.write_byte_data(HMC5883L_ADDR, MODE, 0x00)
                time.sleep(0.2)

            x = read_word(bus, DATA_X_MSB)
            z = read_word(bus, DATA_Z_MSB)
            y = read_word(bus, DATA_Y_MSB)

            filtered_x = low_pass_filter(filtered_x, x)
            filtered_y = low_pass_filter(filtered_y, y)
            filtered_z = low_pass_filter(filtered_z, z)

            roll_deg, pitch_deg, att_src = get_current_attitude(bus)
            raw_heading = compute_heading(filtered_x, filtered_y, filtered_z, roll_deg, pitch_deg)
            heading = apply_zero_offset(raw_heading)
            cardinal = cardinal_from_heading(heading)

            timestamp = round(time.time(), 2)
            with data_lock:
                compass_data["timestamps"].append(timestamp)
                compass_data["x"].append(round(filtered_x, 2))
                compass_data["y"].append(round(filtered_y, 2))
                compass_data["z"].append(round(filtered_z, 2))
                compass_data["heading"].append(round(heading, 2))

                latest_state.update({
                    "connected": True,
                    "heading": round(heading, 2),
                    "raw_heading": round(raw_heading, 2),
                    "x": round(filtered_x, 2),
                    "y": round(filtered_y, 2),
                    "z": round(filtered_z, 2),
                    "cardinal": cardinal,
                    "roll_deg": round(roll_deg, 2),
                    "pitch_deg": round(pitch_deg, 2),
                    "attitude_source": att_src,
                    "zero_offset": round(heading_zero_offset, 2),
                    "error": None,
                })

            write_heading_stream_sample(timestamp, raw_heading, heading, filtered_x, filtered_y, filtered_z)

            time.sleep(0.02)
        except Exception as exc:
            with data_lock:
                latest_state["connected"] = False
                latest_state["error"] = str(exc)

            print(f"Compass read error: {exc}")
            time.sleep(1)
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass
                bus = None
                filtered_x = None
                filtered_y = None
                filtered_z = None


@app.route("/")
def index():
    """
    Renders the HTML template for the main dashboard view.
    """
    return render_template("compass_dashboard.html")


@app.route("/api/compass-data")
def get_compass_data():
    """
    Flask API endpoint that returns historical and latest compass state in JSON format.
    """
    with data_lock:
        return jsonify({
            "timestamps": list(compass_data["timestamps"]),
            "x": list(compass_data["x"]),
            "y": list(compass_data["y"]),
            "z": list(compass_data["z"]),
            "heading": list(compass_data["heading"]),
            "latest": latest_state.copy(),
        })


@app.route("/api/compass/zero", methods=["POST"])
def set_compass_zero():
    """
    Flask API endpoint that sets the current raw heading as the zero offset reference.
    """
    global heading_zero_offset

    with data_lock:
        latest_heading = latest_state.get("raw_heading")
        if latest_heading is None:
            return jsonify({"error": "No compass reading available yet"}), 409

        heading_zero_offset = normalize_heading(latest_heading)
        latest_state["zero_offset"] = round(heading_zero_offset, 2)

    save_heading_zero_offset()

    return jsonify({
        "success": True,
        "zero_offset": round(heading_zero_offset, 2),
    })


@app.route("/api/compass/zero/reset", methods=["POST"])
def reset_compass_zero():
    """
    Flask API endpoint that resets the zero offset calibration to 0.0.
    """
    global heading_zero_offset

    with data_lock:
        heading_zero_offset = 0.0
        latest_state["zero_offset"] = 0.0

    save_heading_zero_offset()

    return jsonify({
        "success": True,
        "zero_offset": 0.0,
    })


def main():
    """
    Main entry point for starting the Flask web dashboard server
    and initiating the background compass reader thread.
    """
    load_heading_zero_offset()
    attach_heading_stream_shm()

    sensor_thread = threading.Thread(target=compass_thread_worker, daemon=True)
    sensor_thread.start()

    print("\n" + "=" * 60)
    print("HMC5883L Compass Dashboard")
    print("=" * 60)
    print("Starting Flask dashboard on http://0.0.0.0:5002")
    print("Open your browser to: http://localhost:5002")
    print("=" * 60 + "\n")

    try:
        app.run(host="0.0.0.0", port=5002, debug=False, threaded=True)
    finally:
        close_heading_stream_shm()


if __name__ == "__main__":
    main()