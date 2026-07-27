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
import threading
from multiprocessing import shared_memory
from multiprocessing import resource_tracker

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

DEFAULT_TARGET_IP = "192.168.137.1"
DEFAULT_TARGET_PORT = 5005

GPS_SERIAL_PORT = "/dev/ttyAMA2"
GPS_BAUD_RATE = 9600

latest_gps_data = {
    "lat": None,
    "lon": None,
    "alt_m": 0.0,
    "speed_kmh": 0.0,
    "satellites": 0,
    "fix_status": "SEARCHING FOR SATELLITES...",
    "fix_code": "0"
}
gps_lock = threading.Lock()

def nmea_to_decimal(raw_val, direction, is_lon=False):
    if not raw_val or '.' not in str(raw_val):
        return None
    try:
        raw_str = str(raw_val)
        dot_idx = raw_str.find('.')
        deg_len = 3 if is_lon else 2
        if dot_idx > deg_len:
            deg_len = dot_idx - 2
        if deg_len <= 0:
            return None
        degrees = float(raw_str[:deg_len])
        minutes = float(raw_str[deg_len:])
        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return round(decimal, 6)
    except ValueError:
        return None

def gps_hardware_thread():
    """Reads GPS NMEA stream from /dev/ttyAMA2 on the Pi drone side."""
    if not SERIAL_AVAILABLE:
        return
    try:
        ser = serial.Serial(GPS_SERIAL_PORT, GPS_BAUD_RATE, timeout=1)
        print(f"[GPS Thread] Hardware GPS listener active on {GPS_SERIAL_PORT} @ {GPS_BAUD_RATE} baud.")
    except Exception as e:
        print(f"[GPS Thread] Serial port {GPS_SERIAL_PORT} unavailable on this system ({e}).")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('$'):
                continue
            parts = line.split(',')
            sentence = parts[0]
            if sentence in ['$GNGGA', '$GPGGA'] and len(parts) >= 10:
                raw_lat, lat_dir = parts[2], parts[3]
                raw_lon, lon_dir = parts[4], parts[5]
                fix_code = parts[6] if len(parts) > 6 else '0'
                sats = int(parts[7]) if len(parts) > 7 and parts[7].isdigit() else 0
                alt = float(parts[9]) if len(parts) > 9 and parts[9] else 0.0
                lat = nmea_to_decimal(raw_lat, lat_dir)
                lon = nmea_to_decimal(raw_lon, lon_dir, is_lon=True)
                fix_str = "3D FIX ACQUIRED" if fix_code in ['1', '2', '4', '5'] else "SEARCHING FOR SATELLITES..."
                with gps_lock:
                    if lat is not None: latest_gps_data["lat"] = lat
                    if lon is not None: latest_gps_data["lon"] = lon
                    latest_gps_data["alt_m"] = alt
                    latest_gps_data["satellites"] = sats
                    latest_gps_data["fix_status"] = fix_str
                    latest_gps_data["fix_code"] = fix_code
            elif sentence in ['$GNRMC', '$GPRMC'] and len(parts) >= 9:
                status = parts[2]
                if status == 'A':
                    lat = nmea_to_decimal(parts[3], parts[4])
                    lon = nmea_to_decimal(parts[5], parts[6], is_lon=True)
                    spd_knots = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
                    with gps_lock:
                        if lat is not None: latest_gps_data["lat"] = lat
                        if lon is not None: latest_gps_data["lon"] = lon
                        latest_gps_data["speed_kmh"] = round(spd_knots * 1.852, 1)
                        latest_gps_data["fix_status"] = "3D FIX ACQUIRED"
        except Exception:
            time.sleep(0.1)

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


from madgwick_ahrs import MadgwickPositionEstimator

def main():
    parser = argparse.ArgumentParser(description="Send Shared Memory Roll Pitch Yaw to web_server.py via UDP.")
    parser.add_argument("--ip", type=str, default=DEFAULT_TARGET_IP, help=f"Target UDP IP (default: {DEFAULT_TARGET_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_TARGET_PORT, help=f"Target UDP Port (default: {DEFAULT_TARGET_PORT})")
    parser.add_argument("--rate", type=float, default=50.0, help="Transmission rate in Hz (default: 50.0)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate

    # Instantiate Madgwick AHRS Position Estimator (6DoF IMU mode, no magnetometer)
    madgwick_estimator = MadgwickPositionEstimator(beta=0.1, sample_freq=args.rate)

    # Start Hardware GPS thread (reading /dev/ttyAMA2 on drone side)
    gps_thread = threading.Thread(target=gps_hardware_thread, daemon=True)
    gps_thread.start()

    print(f"[SHM UDP Sender] Streaming Madgwick AHRS, Optical Flow & GPS Telemetry to UDP {args.ip}:{args.port} @ {args.rate} Hz...")

    start_time = time.time()

    try:
        while True:
            loop_start = time.time()
            att = read_latest_attitude()
            flow = read_latest_flow()

            if att is not None:
                roll = att["roll"]
                pitch = att["pitch"]
                yaw = att["yaw"]
                gx = att["gx"]
                gy = att["gy"]
                gz = att["gz"]
                ts = att["timestamp"]
            else:
                # Fallback: Synthesize smooth dynamic 3D motion for demonstration
                t = loop_start - start_time
                roll = 18.0 * math.sin(t * 1.8)
                pitch = 12.0 * math.cos(t * 1.4)
                yaw = math.degrees(math.atan2(math.sin(t * 0.8), math.cos(t * 0.8)))
                gx = 18.0 * 1.8 * math.cos(t * 1.8)
                gy = -12.0 * 1.4 * math.sin(t * 1.4)
                gz = 0.8 * (180.0 / math.pi)
                ts = loop_start

            # Calculate gravity projections for accelerometer input
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            ax_g = -math.sin(pitch_rad)
            ay_g = math.sin(roll_rad) * math.cos(pitch_rad)
            az_g = math.cos(roll_rad) * math.cos(pitch_rad)

            # Update Madgwick AHRS & Position state
            m_state = madgwick_estimator.update(
                gx_dps=gx, gy_dps=gy, gz_dps=gz,
                ax_g=ax_g, ay_g=ay_g, az_g=az_g
            )

            # Use optical flow displacement for xy position if available, fallback to Madgwick 3D position
            pos_x = flow["x_m"] if flow else m_state["position"]["x"]
            pos_y = flow["y_m"] if flow else m_state["position"]["y"]
            pos_z = -flow["z_m"] if flow else -m_state["position"]["z"]

            vel_x = flow["vx"] if flow else m_state["velocity"]["x"]
            vel_y = flow["vy"] if flow else m_state["velocity"]["y"]
            vel_z = -m_state["velocity"]["z"]

            with gps_lock:
                gps_snapshot = dict(latest_gps_data)

            telemetry_packet = {
                "timestamp": ts,
                "rotation": {
                    "quaternion": m_state["quaternion"],
                    "euler": {
                        "roll": round(roll, 2),
                        "pitch": round(pitch, 2),
                        "yaw": round(yaw, 2),
                    },
                },
                "translation": {
                    "position": {"x": round(pos_x, 3), "y": round(pos_z, 3), "z": round(pos_y, 3)},
                    "velocity": {"x": round(vel_x, 3), "y": round(vel_z, 3), "z": round(vel_y, 3)},
                    "linear_accel": m_state["linear_accel"],
                },
                "heading": round(yaw, 2),
                "status": "connected",
                "gps": gps_snapshot
            }

            payload = json.dumps(telemetry_packet).encode("utf-8")
            sock.sendto(payload, (args.ip, args.port))

            elapsed = time.time() - loop_start
            time.sleep(max(0.001, interval - elapsed))
    except KeyboardInterrupt:
        print("\n[SHM UDP Sender] Stopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
