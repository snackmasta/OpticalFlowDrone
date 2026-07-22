#!/usr/bin/env python3
"""
Bridge script to read Roll, Pitch, Yaw (and optical flow positions) from Shared Memory
and forward UDP telemetry packets to web_server.py (default target IP: 192.168.137.1, Port: 5005).
"""

import argparse
import json
import math
import socket
import struct
import time
from multiprocessing import shared_memory
from multiprocessing import resource_tracker

DEFAULT_TARGET_IP = "192.168.137.1"
DEFAULT_TARGET_PORT = 5005

ATTITUDE_SHM_NAME = "drone_attitude_stream"
ATTITUDE_SHM_MAGIC = b"ATT "
ATTITUDE_SHM_HEADER_FORMAT = "<4sII"
ATTITUDE_SHM_RECORD_FORMAT = "<7d"
ATTITUDE_SHM_HEADER_SIZE = struct.calcsize(ATTITUDE_SHM_HEADER_FORMAT)
ATTITUDE_SHM_RECORD_SIZE = struct.calcsize(ATTITUDE_SHM_RECORD_FORMAT)
ATTITUDE_MAX_SAMPLES = 120

FLOW_SHM_NAME = "optical_flow_stream"
FLOW_SHM_MAGIC = b"FLOW"
FLOW_SHM_HEADER_FORMAT = "<4sII"
FLOW_SHM_RECORD_FORMAT = "<11d"
FLOW_SHM_HEADER_SIZE = struct.calcsize(FLOW_SHM_HEADER_FORMAT)
FLOW_SHM_RECORD_SIZE = struct.calcsize(FLOW_SHM_RECORD_FORMAT)
FLOW_MAX_SAMPLES = 120


def safe_unregister_shm(shm):
    """Unregisters shared memory from Python's resource_tracker."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """Converts Euler angles in degrees to Quaternion (w, x, y, z)."""
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return {"w": round(w, 6), "x": round(x, 6), "y": round(y, 6), "z": round(z, 6)}


def read_latest_attitude():
    """Reads the latest Roll, Pitch, Yaw record from drone_attitude_stream SHM."""
    try:
        shm = shared_memory.SharedMemory(name=ATTITUDE_SHM_NAME)
        safe_unregister_shm(shm)
        buf = bytes(shm.buf)
        shm.close()

        magic, write_index, sample_count = struct.unpack_from(ATTITUDE_SHM_HEADER_FORMAT, buf, 0)
        if magic != ATTITUDE_SHM_MAGIC or sample_count == 0:
            return None

        latest_index = (write_index - 1) % ATTITUDE_MAX_SAMPLES
        record_offset = ATTITUDE_SHM_HEADER_SIZE + (latest_index * ATTITUDE_SHM_RECORD_SIZE)
        ts, roll, pitch, yaw, gx, gy, gz = struct.unpack_from(ATTITUDE_SHM_RECORD_FORMAT, buf, record_offset)
        return {
            "timestamp": ts,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "gx": gx,
            "gy": gy,
            "gz": gz,
        }
    except Exception:
        return None


def read_latest_flow():
    """Reads the latest Optical Flow sample from optical_flow_stream SHM."""
    try:
        shm = shared_memory.SharedMemory(name=FLOW_SHM_NAME)
        safe_unregister_shm(shm)
        buf = bytes(shm.buf)
        shm.close()

        magic, write_index, sample_count = struct.unpack_from(FLOW_SHM_HEADER_FORMAT, buf, 0)
        if magic != FLOW_SHM_MAGIC or sample_count == 0:
            return None

        latest_index = (write_index - 1) % FLOW_MAX_SAMPLES
        record_offset = FLOW_SHM_HEADER_SIZE + (latest_index * FLOW_SHM_RECORD_SIZE)
        values = struct.unpack_from(FLOW_SHM_RECORD_FORMAT, buf, record_offset)
        ts, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading = values
        return {
            "x_m": x_cm / 100.0,
            "y_m": y_cm / 100.0,
            "z_m": alt,
            "vx": vx,
            "vy": vy,
        }
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description="Send Shared Memory Roll Pitch Yaw to web_server.py via UDP.")
    parser.add_argument("--ip", type=str, default=DEFAULT_TARGET_IP, help=f"Target UDP IP (default: {DEFAULT_TARGET_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_TARGET_PORT, help=f"Target UDP Port (default: {DEFAULT_TARGET_PORT})")
    parser.add_argument("--rate", type=float, default=50.0, help="Transmission rate in Hz (default: 50.0)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate

    print(f"[SHM UDP Sender] Streaming RPY SHM to UDP {args.ip}:{args.port} @ {args.rate} Hz...")

    try:
        while True:
            start_ts = time.time()
            att = read_latest_attitude()
            flow = read_latest_flow()

            if att is not None:
                roll = att["roll"]
                pitch = att["pitch"]
                yaw = att["yaw"]
                quat = euler_to_quaternion(roll, pitch, yaw)

                pos_x = flow["x_m"] if flow else 0.0
                pos_y = flow["y_m"] if flow else 0.0
                pos_z = flow["z_m"] if flow else 0.0
                vel_x = flow["vx"] if flow else 0.0
                vel_y = flow["vy"] if flow else 0.0

                telemetry_packet = {
                    "timestamp": att["timestamp"],
                    "rotation": {
                        "quaternion": quat,
                        "euler": {
                            "roll": round(roll, 2),
                            "pitch": round(pitch, 2),
                            "yaw": round(yaw, 2),
                        },
                    },
                    "translation": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "velocity": {"x": round(vel_x, 3), "y": round(vel_y, 3), "z": round(0.0, 3)},
                        "linear_accel": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                    "heading": round(yaw, 2),
                    "status": "connected",
                }

                payload = json.dumps(telemetry_packet).encode("utf-8")
                sock.sendto(payload, (args.ip, args.port))

            elapsed = time.time() - start_ts
            time.sleep(max(0.001, interval - elapsed))
    except KeyboardInterrupt:
        print("\n[SHM UDP Sender] Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
