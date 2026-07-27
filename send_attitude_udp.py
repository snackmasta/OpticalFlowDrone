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

try:
    import pynmea2
    PYNMEA2_AVAILABLE = True
except ImportError:
    PYNMEA2_AVAILABLE = False

BUZZER_PIN = 12
buzzer_device = None
buzzer_breached_state = False

try:
    from gpiozero import PWMOutputDevice
    buzzer_device = PWMOutputDevice(BUZZER_PIN, frequency=2300)
    buzzer_device.off()
    print(f"[Buzzer] Hardware PWMOutputDevice initialized on GPIO {BUZZER_PIN} @ 2300 Hz.")
except Exception as e:
    print(f"[Buzzer] gpiozero PWMOutputDevice disabled or non-Pi system ({e}).")

def _buzzer_loop_thread():
    """Background thread looping 2300Hz buzzer beep (0.15s ON, 0.15s OFF) while outside geofence."""
    global buzzer_breached_state
    while True:
        if buzzer_breached_state and buzzer_device is not None:
            try:
                buzzer_device.value = 0.5  # 2300Hz tone ON (50% duty cycle)
                time.sleep(0.15)
                buzzer_device.off()       # Tone OFF
                time.sleep(0.15)
            except Exception:
                time.sleep(0.1)
        else:
            if buzzer_device is not None:
                try:
                    buzzer_device.off()
                except Exception:
                    pass
            time.sleep(0.05)

if buzzer_device is not None:
    buzzer_thread = threading.Thread(target=_buzzer_loop_thread, daemon=True)
    buzzer_thread.start()

def trigger_geofence_buzzer(is_breached):
    """Triggers the looping 2300Hz GPIO 12 buzzer alarm when outside the 1m x 1m geofence."""
    global buzzer_breached_state
    buzzer_breached_state = bool(is_breached)

def convert_latitude(lat_str, lat_dir):
    if not lat_str or not lat_dir:
        return None
    try:
        deg = float(lat_str[:2])
        minutes = float(lat_str[2:])
        dec_deg = deg + (minutes / 60.0)
        if lat_dir == 'S':
            dec_deg = -dec_deg
        return dec_deg
    except ValueError:
        return None

def convert_longitude(lon_str, lon_dir):
    if not lon_str or not lon_dir:
        return None
    try:
        deg = float(lon_str[:3])
        minutes = float(lon_str[3:])
        dec_deg = deg + (minutes / 60.0)
        if lon_dir == 'W':
            dec_deg = -dec_deg
        return dec_deg
    except ValueError:
        return None

def gps_hardware_thread():
    """Continuously reads NMEA sentences from /dev/ttyAMA2 serial port."""
    global latest_gps_data
    if not SERIAL_AVAILABLE or not PYNMEA2_AVAILABLE:
        print("[GPS Thread] serial or pynmea2 not installed. GPS hardware reader disabled.")
        return

    print(f"[GPS Thread] Connecting to GPS module on {GPS_SERIAL_PORT} @ {GPS_BAUD_RATE} baud...")
    while True:
        ser = None
        try:
            ser = serial.Serial(GPS_SERIAL_PORT, baudrate=GPS_BAUD_RATE, timeout=1)
            print(f"[GPS Thread] Serial port {GPS_SERIAL_PORT} opened successfully.")

            while True:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$'):
                    try:
                        msg = pynmea2.parse(line)
                    except pynmea2.ParseError:
                        continue

                    if getattr(msg, 'sentence_type', None) in ['GGA', 'RMC']:
                        lat = getattr(msg, 'latitude', None)
                        lon = getattr(msg, 'longitude', None)

                        if (lat is None or lat == 0.0) and hasattr(msg, 'lat') and hasattr(msg, 'lat_dir'):
                            lat = convert_latitude(msg.lat, msg.lat_dir)
                        if (lon is None or lon == 0.0) and hasattr(msg, 'lon') and hasattr(msg, 'lon_dir'):
                            lon = convert_longitude(msg.lon, msg.lon_dir)

                        spd_knots = float(getattr(msg, 'spd_over_grnd', 0.0) or 0.0)
                        num_sats = int(getattr(msg, 'num_sats', 0) or 0)
                        fix_code = str(getattr(msg, 'gps_qual', '0') or '0')
                        status = getattr(msg, 'status', 'V')

                        with gps_lock:
                            if num_sats > 0 or latest_gps_data["satellites"] == 0:
                                latest_gps_data["satellites"] = num_sats
                            if lat is not None: latest_gps_data["lat"] = lat
                            if lon is not None: latest_gps_data["lon"] = lon
                            latest_gps_data["speed_kmh"] = round(spd_knots * 1.852, 1)
                            if status == 'A' or fix_code in ['1', '2']:
                                latest_gps_data["fix_status"] = "3D FIX ACQUIRED"
                            latest_gps_data["last_update"] = time.time()

        except Exception as e:
            print(f"[GPS Thread] Serial error on {GPS_SERIAL_PORT}: {e}. Retrying in 2s...")
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            time.sleep(2.0)

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
from sensor_fusion import SensorFusionEngine

def main():
    parser = argparse.ArgumentParser(description="Send Shared Memory Roll Pitch Yaw to web_server.py via UDP.")
    parser.add_argument("--ip", type=str, default=DEFAULT_TARGET_IP, help=f"Target UDP IP (default: {DEFAULT_TARGET_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_TARGET_PORT, help=f"Target UDP Port (default: {DEFAULT_TARGET_PORT})")
    parser.add_argument("--rate", type=float, default=50.0, help="Transmission rate in Hz (default: 50.0)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate

    madgwick_estimator = MadgwickPositionEstimator(beta=0.1, sample_freq=args.rate)
    fusion_engine = SensorFusionEngine(gps_gain=0.15, flow_weight=0.85)

    gps_thread = threading.Thread(target=gps_hardware_thread, daemon=True)
    gps_thread.start()

    print(f"[SHM UDP Sender] Streaming Madgwick AHRS, Optical Flow & Sensor-Fused GPS Telemetry to UDP {args.ip}:{args.port} @ {args.rate} Hz...")

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
                t = loop_start - start_time
                roll = 18.0 * math.sin(t * 1.8)
                pitch = 12.0 * math.cos(t * 1.4)
                yaw = math.degrees(math.atan2(math.sin(t * 0.8), math.cos(t * 0.8)))
                gx = 18.0 * 1.8 * math.cos(t * 1.8)
                gy = -12.0 * 1.4 * math.sin(t * 1.4)
                gz = 0.8 * (180.0 / math.pi)
                ts = loop_start

            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            ax_g = -math.sin(pitch_rad)
            ay_g = math.sin(roll_rad) * math.cos(pitch_rad)
            az_g = math.cos(roll_rad) * math.cos(pitch_rad)

            m_state = madgwick_estimator.update(
                gx_dps=gx, gy_dps=gy, gz_dps=gz,
                ax_g=ax_g, ay_g=ay_g, az_g=az_g
            )

            pos_x = flow["x_m"] if flow else m_state["position"]["x"]
            pos_y = flow["y_m"] if flow else m_state["position"]["y"]
            pos_z = -flow["z_m"] if flow else -m_state["position"]["z"]

            vel_x = flow["vx"] if flow else m_state["velocity"]["x"]
            vel_y = flow["vy"] if flow else m_state["velocity"]["y"]
            vel_z = -m_state["velocity"]["z"]

            fusion_engine.update_attitude(roll_deg=roll, pitch_deg=pitch, heading_deg=yaw)
            fusion_engine.predict_flow_step(flow_vx_m_s=vel_x, flow_vy_m_s=vel_y, dt=interval)

            with gps_lock:
                gps_snapshot = dict(latest_gps_data)

            if gps_snapshot.get("lat") is not None and gps_snapshot.get("lon") is not None:
                fusion_engine.update_gps(
                    raw_lat=gps_snapshot["lat"],
                    raw_lon=gps_snapshot["lon"],
                    alt_m=gps_snapshot.get("alt_m", 0.0),
                    speed_kmh=gps_snapshot.get("speed_kmh", 0.0),
                    sats=gps_snapshot.get("satellites", 0),
                    fix_status=gps_snapshot.get("fix_status", "")
                )

            fused_gps_state = fusion_engine.get_fused_state()
            fused_x = fused_gps_state.get("fused_x_m", 0.0)
            fused_y = fused_gps_state.get("fused_y_m", 0.0)

            chk_x = fused_x if (fused_x != 0.0 or fused_y != 0.0) else pos_x
            chk_y = fused_y if (fused_x != 0.0 or fused_y != 0.0) else pos_y
            is_breached = (abs(chk_x) > 0.5) or (abs(chk_y) > 0.5)

            trigger_geofence_buzzer(is_breached)

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
                "geofence_breach": is_breached,
                "gps": gps_snapshot,
                "fused_gps": fused_gps_state
            }

            payload = json.dumps(telemetry_packet).encode("utf-8")
            sock.sendto(payload, (args.ip, args.port))

            elapsed = time.time() - loop_start
            time.sleep(max(0.001, interval - elapsed))
    except KeyboardInterrupt:
        print("\n[SHM UDP Sender] Stopped.")
    finally:
        global buzzer_breached_state
        buzzer_breached_state = False
        if buzzer_device is not None:
            try:
                buzzer_device.off()
                buzzer_device.close()
            except Exception:
                pass
        print("[SHM UDP Sender] Hardware resources cleaned up.")
        sock.close()

if __name__ == "__main__":
    main()
