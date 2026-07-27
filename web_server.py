import http.server
import socketserver
import socket
import json
import threading
import time
import os
import sys
import csv
import math
from urllib.parse import parse_qs, urlparse, unquote
from geofence_engine import send_geofence_status_udp

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import pynmea2
    PYNMEA2_AVAILABLE = True
except ImportError:
    PYNMEA2_AVAILABLE = False

# Server configuration
HTTP_PORT = 8000
UDP_PORT = 5005
UDP_IP = "0.0.0.0"

# Directories to search for session logs
LOG_DIRECTORIES = ["recordings", "hasil_dan_pembahasan", "."]

# GPS and 2D Cartesian Plane projection configuration (origin initialized dynamically from hardware GPS)
DEFAULT_GPS_ORIGIN = {"lat": None, "lon": None}
EARTH_RADIUS_M = 6378137.0
GPS_SERIAL_PORT = os.environ.get("GPS_SERIAL_PORT", "/dev/ttyAMA2")
GPS_BAUD_RATE = 9600

gps_data_state = {
    "lat": None,
    "lon": None,
    "alt_m": 0.0,
    "speed_kmh": 0.0,
    "satellites": 0,
    "fix_status": "SEARCHING FOR SATELLITES...",
    "fix_code": "0",
    "projected_x_m": 0.0,
    "projected_y_m": 0.0,
    "origin": {"lat": None, "lon": None},
    "last_update": time.time()
}

def geo_to_2d_plane(lat, lon, origin_lat, origin_lon):
    """
    Projects WGS-84 Geographic (Lat, Lon) to 2D Local Cartesian Plane (X, Y) in meters.
    X: East (meters), Y: North (meters)
    """
    if lat is None or lon is None or origin_lat is None or origin_lon is None:
        return 0.0, 0.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)

    dlat = lat_rad - origin_lat_rad
    dlon = lon_rad - origin_lon_rad

    avg_lat = (lat_rad + origin_lat_rad) / 2.0
    x_m = dlon * math.cos(avg_lat) * EARTH_RADIUS_M
    y_m = dlat * EARTH_RADIUS_M
    return round(x_m, 3), round(y_m, 3)

def plane_2d_to_geo(x_m, y_m, origin_lat, origin_lon):
    """
    Inverse projection: Converts 2D Cartesian Plane (X meters East, Y meters North)
    to Geographic (Lat, Lon).
    """
    if origin_lat is None or origin_lon is None:
        return 0.0, 0.0
    dlat = y_m / EARTH_RADIUS_M
    lat_rad = math.radians(origin_lat) + dlat
    lat = math.degrees(lat_rad)
    
    dlon = x_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    lon = origin_lon + math.degrees(dlon)
    return round(lat, 6), round(lon, 6)

def nmea_to_decimal(raw_val, direction, is_lon=False):
    """Converts NMEA raw coordinate format (DDMM.MMMM / DDDMM.MMMM) to Decimal Degrees."""
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

origin_initialized = False

def update_gps_state(lat=None, lon=None, alt_m=None, speed_kmh=None, satellites=None, fix_status=None, fix_code=None, origin=None):
    """Updates global gps_data_state and recalculates 2D planar projection."""
    global gps_data_state, latest_telemetry, origin_initialized
    with clients_lock:
        if origin is not None and isinstance(origin, dict):
            if "lat" in origin and origin["lat"] is not None:
                gps_data_state["origin"]["lat"] = float(origin["lat"])
            if "lon" in origin and origin["lon"] is not None:
                gps_data_state["origin"]["lon"] = float(origin["lon"])
            origin_initialized = True

        if lat is not None: gps_data_state["lat"] = float(lat)
        if lon is not None: gps_data_state["lon"] = float(lon)
        if alt_m is not None: gps_data_state["alt_m"] = float(alt_m)
        if speed_kmh is not None: gps_data_state["speed_kmh"] = float(speed_kmh)
        if satellites is not None: gps_data_state["satellites"] = int(satellites)
        if fix_status is not None: gps_data_state["fix_status"] = str(fix_status)
        if fix_code is not None: gps_data_state["fix_code"] = str(fix_code)

        # Set reference origin to first valid GPS fix coordinate automatically from hardware GPS
        if (not origin_initialized or gps_data_state["origin"]["lat"] is None) and lat is not None and lon is not None:
            gps_data_state["origin"]["lat"] = float(lat)
            gps_data_state["origin"]["lon"] = float(lon)
            origin_initialized = True

        orig_lat = gps_data_state["origin"]["lat"]
        orig_lon = gps_data_state["origin"]["lon"]
        curr_lat = gps_data_state["lat"]
        curr_lon = gps_data_state["lon"]

        if orig_lat is not None and orig_lon is not None and curr_lat is not None and curr_lon is not None:
            x_m, y_m = geo_to_2d_plane(curr_lat, curr_lon, orig_lat, orig_lon)
        else:
            x_m, y_m = 0.0, 0.0

        gps_data_state["projected_x_m"] = x_m
        gps_data_state["projected_y_m"] = y_m
        gps_data_state["last_update"] = time.time()

        # Update latest_telemetry dictionary
        latest_telemetry["gps"] = dict(gps_data_state)

# Global state to store latest telemetry packet and connected client queues
latest_telemetry = {
    "timestamp": time.time(),
    "rotation": {"quaternion": {"w": 0.7071, "x": 0.7071, "y": 0, "z": 0}, "euler": {"roll": 90.0, "pitch": 0.0, "yaw": 0.0}},
    "translation": {"position": {"x": 0, "y": 0, "z": 0}, "velocity": {"x": 0, "y": 0, "z": 0}, "linear_accel": {"x": 0, "y": 0, "z": 0}},
    "heading": 0.0,
    "status": "waiting",
    "gps": dict(gps_data_state)
}

connected_sse_clients = []
clients_lock = threading.Lock()


def get_available_logs():
    """
    Scans recordings, hasil_dan_pembahasan, and current root directories
    to discover all recorded CSV session logs.
    """
    logs = []
    base_dir = os.path.dirname(os.path.abspath(__file__))
    seen_paths = set()

    for rel_dir in LOG_DIRECTORIES:
        target_dir = os.path.normpath(os.path.join(base_dir, rel_dir))
        if not os.path.exists(target_dir):
            continue
        try:
            for item in os.listdir(target_dir):
                if item.lower().endswith(".csv"):
                    full_path = os.path.join(target_dir, item)
                    rel_path = os.path.relpath(full_path, base_dir).replace("\\", "/")
                    if rel_path in seen_paths:
                        continue
                    seen_paths.add(rel_path)

                    stat = os.stat(full_path)
                    sample_count = 0
                    try:
                        with open(full_path, "r", encoding="utf-8", errors="ignore") as f:
                            sample_count = max(0, sum(1 for row in f if row.strip()) - 1)
                    except Exception:
                        pass

                    logs.append({
                        "filename": item,
                        "relative_path": rel_path,
                        "size_bytes": stat.st_size,
                        "sample_count": sample_count,
                        "modified_timestamp": stat.st_mtime,
                        "modified_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime)),
                        "folder": rel_dir
                    })
        except Exception as e:
            print(f"[LOG DISCOVERY ERROR] {e}")

    logs.sort(key=lambda x: x["modified_timestamp"], reverse=True)
    return logs


def parse_csv_log(log_target):
    """
    Reads a CSV session log file and returns parsed structured data & metadata.
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    clean_target = unquote(log_target).strip().lstrip("/")

    candidate = os.path.normpath(os.path.join(base_dir, clean_target))

    # Prevent directory traversal outside base workspace
    if not candidate.startswith(base_dir):
        return None, "Access denied: Path outside workspace directory"

    if not (os.path.exists(candidate) and os.path.isfile(candidate)):
        filename = os.path.basename(clean_target)
        found_path = None
        for rel_dir in LOG_DIRECTORIES:
            possible = os.path.normpath(os.path.join(base_dir, rel_dir, filename))
            if os.path.exists(possible) and os.path.isfile(possible):
                found_path = possible
                break
        if found_path:
            candidate = found_path
        else:
            return None, f"Log file '{clean_target}' not found"

    try:
        records = []
        headers = []
        with open(candidate, "r", encoding="utf-8", errors="ignore") as f:
            reader = csv.reader(f)
            headers = next(reader, None)
            if not headers:
                return {
                    "filename": os.path.basename(candidate),
                    "relative_path": os.path.relpath(candidate, base_dir).replace("\\", "/"),
                    "headers": [],
                    "total_samples": 0,
                    "records": [],
                    "summary": {}
                }, None

            headers = [h.strip() for h in headers]
            for row in reader:
                if not row or not any(field.strip() for field in row):
                    continue
                record = {}
                for idx, val in enumerate(row):
                    if idx < len(headers):
                        key = headers[idx]
                        v = val.strip()
                        try:
                            if "." in v or "e" in v.lower():
                                record[key] = float(v)
                            else:
                                record[key] = int(v)
                        except ValueError:
                            record[key] = v
                records.append(record)

        stat = os.stat(candidate)
        rel_path = os.path.relpath(candidate, base_dir).replace("\\", "/")

        summary = {"sample_count": len(records)}
        if records:
            if "Timestamp (s)" in records[0]:
                t0 = records[0]["Timestamp (s)"]
                t1 = records[-1]["Timestamp (s)"]
                if isinstance(t0, (int, float)) and isinstance(t1, (int, float)):
                    summary["duration_s"] = round(t1 - t0, 2)
            if "Altitude (m)" in records[0]:
                alts = [r["Altitude (m)"] for r in records if isinstance(r.get("Altitude (m)"), (int, float))]
                if alts:
                    summary["max_altitude_m"] = round(max(alts), 2)
                    summary["min_altitude_m"] = round(min(alts), 2)
            if "Speed (m/s)" in records[0]:
                speeds = [r["Speed (m/s)"] for r in records if isinstance(r.get("Speed (m/s)"), (int, float))]
                if speeds:
                    summary["max_speed_mps"] = round(max(speeds), 2)
                    summary["avg_speed_mps"] = round(sum(speeds) / len(speeds), 2)

        return {
            "filename": os.path.basename(candidate),
            "relative_path": rel_path,
            "size_bytes": stat.st_size,
            "modified_time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(stat.st_mtime)),
            "headers": headers,
            "total_samples": len(records),
            "summary": summary,
            "records": records
        }, None
    except Exception as exc:
        return None, f"Failed to read CSV log: {exc}"


class TelemetryHTTPServer(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve files from the ./web directory
        web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
        super().__init__(*args, directory=web_dir, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed_url = urlparse(self.path)
        path = parsed_url.path

        if path == '/stream':
            # Server-Sent Events (SSE) streaming endpoint
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            client_queue = []
            with clients_lock:
                connected_sse_clients.append(client_queue)

            try:
                # Send initial state
                with clients_lock:
                    initial_data = json.dumps(latest_telemetry)
                self.wfile.write(f"data: {initial_data}\n\n".encode('utf-8'))
                self.wfile.flush()

                while True:
                    time.sleep(0.02)  # 50Hz streaming
                    with clients_lock:
                        data_str = json.dumps(latest_telemetry)
                    self.wfile.write(f"data: {data_str}\n\n".encode('utf-8'))
                    self.wfile.flush()
            except Exception:
                pass
            finally:
                with clients_lock:
                    if client_queue in connected_sse_clients:
                        connected_sse_clients.remove(client_queue)

        elif path == '/api/gps':
            with clients_lock:
                gps_res = dict(gps_data_state)
            response_data = json.dumps({"status": "success", "gps": gps_res}, indent=2).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Length', str(len(response_data)))
            self.end_headers()
            self.wfile.write(response_data)

        elif path in ('/api/logs', '/logs', '/api/recordings'):
            # List available session logs
            logs = get_available_logs()
            response_data = json.dumps({"status": "success", "count": len(logs), "logs": logs}, indent=2).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Content-Length', str(len(response_data)))
            self.end_headers()
            self.wfile.write(response_data)

        elif path.startswith('/api/logs/') or path.startswith('/api/recordings/') or path.startswith('/logs/') or path in ('/api/log', '/log'):
            # Fetch specific log data
            log_target = None
            if path in ('/api/log', '/log'):
                qs = parse_qs(parsed_url.query)
                log_target = qs.get('file', [None])[0] or qs.get('name', [None])[0] or qs.get('path', [None])[0]
            elif path.startswith('/api/logs/'):
                log_target = path[len('/api/logs/'):]
            elif path.startswith('/api/recordings/'):
                log_target = path[len('/api/recordings/'):]
            elif path.startswith('/logs/'):
                log_target = path[len('/logs/'):]

            if not log_target:
                err_body = json.dumps({"status": "error", "message": "Missing log filename or path parameter"}, indent=2).encode('utf-8')
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(err_body)))
                self.end_headers()
                self.wfile.write(err_body)
                return

            log_data, err = parse_csv_log(log_target)
            if err:
                err_body = json.dumps({"status": "error", "message": err}, indent=2).encode('utf-8')
                self.send_response(404)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(err_body)))
                self.end_headers()
                self.wfile.write(err_body)
            else:
                body = json.dumps({"status": "success", "log": log_data}, indent=2).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        elif path.startswith('/recordings/') or path.startswith('/hasil_dan_pembahasan/'):
            # Static log file serving from workspace root
            base_dir = os.path.dirname(os.path.abspath(__file__))
            clean_path = unquote(path.lstrip('/'))
            file_path = os.path.normpath(os.path.join(base_dir, clean_path))
            if file_path.startswith(base_dir) and os.path.exists(file_path) and os.path.isfile(file_path):
                try:
                    with open(file_path, 'rb') as f:
                        content = f.read()
                    self.send_response(200)
                    if file_path.endswith('.csv'):
                        self.send_header('Content-Type', 'text/csv; charset=utf-8')
                    else:
                        self.send_header('Content-Type', 'application/octet-stream')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Content-Length', str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                    return
                except Exception as e:
                    pass
            super().do_GET()
        else:
            super().do_GET()

    def do_POST(self):
        parsed_url = urlparse(self.path)
        path = parsed_url.path

        if path in ('/api/gps/origin', '/api/gps'):
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else b'{}'
            try:
                data = json.loads(body.decode('utf-8'))
                if path == '/api/gps/origin':
                    origin = {"lat": data.get("lat"), "lon": data.get("lon")}
                    update_gps_state(origin=origin)
                else:
                    update_gps_state(
                        lat=data.get("lat"),
                        lon=data.get("lon"),
                        alt_m=data.get("alt_m"),
                        speed_kmh=data.get("speed_kmh"),
                        satellites=data.get("satellites"),
                        fix_status=data.get("fix_status"),
                        origin=data.get("origin")
                    )
                resp = json.dumps({"status": "success", "gps": gps_data_state}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)
                self.wfile.flush()
            except Exception as e:
                err_body = json.dumps({"status": "error", "message": str(e)}).encode('utf-8')
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(err_body)))
                self.end_headers()
                self.wfile.write(err_body)
                self.wfile.flush()
        elif path in ('/api/geofence/status', '/api/geofence'):
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length) if content_length > 0 else b'{}'
            try:
                data = json.loads(body.decode('utf-8'))
                is_breached = bool(data.get("breached", False))
                send_geofence_buzzer_udp(is_breached)
                resp = json.dumps({"status": "success", "breached": is_breached}).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(resp)))
                self.end_headers()
                self.wfile.write(resp)
                self.wfile.flush()
            except Exception as e:
                err_body = json.dumps({"status": "error", "message": str(e)}).encode('utf-8')
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Content-Length', str(len(err_body)))
                self.end_headers()
                self.wfile.write(err_body)
                self.wfile.flush()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress routine GET logging for clean console output
        try:
            if len(args) > 0 and isinstance(args[0], str) and ("GET /stream" in args[0] or "GET /api/" in args[0]):
                return
        except Exception:
            pass
        super().log_message(format, *args)


# Arm destination UDP settings
ARM_UDP_IP = "192.168.137.54"
ARM_UDP_PORT = 8888

# Buzzer destination UDP settings
BUZZER_UDP_IP = os.getenv("BUZZER_UDP_IP", "127.0.0.1")
BUZZER_UDP_PORT = int(os.getenv("BUZZER_UDP_PORT", "5006"))

def send_geofence_buzzer_udp(is_breached):
    """Delegates geofence status dispatch to Geofence Engine."""
    send_geofence_status_udp(is_breached, target_ip=BUZZER_UDP_IP, target_port=BUZZER_UDP_PORT)

def send_arm_angles(roll_deg, pitch_deg):
    """
    Calculates arm joint angles (90 - roll° for Shoulder J2, 90 + pitch° for Wrist J4 clamped 0..180)
    and streams to 4-DOF Robotic Arm via UDP.
    Payload format: "90,<shoulder_angle>,90,<wrist_angle>"
    """
    try:
        shoulder_angle = int(round(90.0 - roll_deg))
        shoulder_angle = max(0, min(180, shoulder_angle))

        wrist_angle = int(round(90.0 + pitch_deg))
        wrist_angle = max(0, min(180, wrist_angle))

        payload = f"90,{shoulder_angle},90,{wrist_angle}".encode('ascii')
        arm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        arm_sock.sendto(payload, (ARM_UDP_IP, ARM_UDP_PORT))
        arm_sock.close()
    except Exception as e:
        pass

def start_udp_listener():
    """
    Listens for incoming UDP telemetry packets from main.py
    and updates global latest_telemetry state.
    """
    global latest_telemetry
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except Exception:
            pass
        sock.bind((UDP_IP, UDP_PORT))
        print(f"[UDP Listener] Bound to {UDP_IP}:{UDP_PORT}")
    except Exception as e:
        print(f"[ERROR] Failed to bind UDP listener: {e}")
        return

    while True:
        try:
            data, _ = sock.recvfrom(4096)
            payload = json.loads(data.decode('utf-8'))
            payload["status"] = "connected"
            
            # Extract & process GPS telemetry if present in UDP payload
            if "fused_gps" in payload and isinstance(payload["fused_gps"], dict):
                fg = payload["fused_gps"]
                if fg.get("fused_lat") is not None and fg.get("fused_lon") is not None:
                    orig_l = gps_data_state["origin"]["lat"]
                    orig_lo = gps_data_state["origin"]["lon"]
                    fx_m, fy_m = geo_to_2d_plane(fg["fused_lat"], fg["fused_lon"], orig_l, orig_lo)
                    fg["fused_x_m"] = fx_m
                    fg["fused_y_m"] = fy_m

            if "gps" in payload and isinstance(payload["gps"], dict):
                g = payload["gps"]
                update_gps_state(
                    lat=g.get("lat"), lon=g.get("lon"), alt_m=g.get("alt_m"),
                    speed_kmh=g.get("speed_kmh"), satellites=g.get("satellites"),
                    fix_status=g.get("fix_status"), fix_code=g.get("fix_code"),
                    origin=g.get("origin")
                )
            elif "lat" in payload or "latitude" in payload:
                lat_v = payload.get("lat") or payload.get("latitude")
                lon_v = payload.get("lon") or payload.get("longitude")
                alt_v = payload.get("alt") or payload.get("altitude")
                update_gps_state(lat=lat_v, lon=lon_v, alt_m=alt_v)

            payload["gps"] = dict(gps_data_state)

            with clients_lock:
                latest_telemetry = payload

            # Extract roll & pitch angles and stream to 4-DOF Arm
            euler = payload.get("rotation", {}).get("euler", {})
            roll_val = euler.get("roll", 0.0)
            pitch_val = euler.get("pitch", 0.0)
            send_arm_angles(roll_val, pitch_val)
        except Exception:
            pass


class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = os.name != 'nt'  # Prevent socket hijacking on Windows by zombie background processes

    def handle_error(self, request, client_address):
        """Silently ignore routine client connection aborts/resets during browser refresh/close."""
        exc_type, exc_val, _ = sys.exc_info()
        if exc_type in (ConnectionAbortedError, ConnectionResetError, BrokenPipeError) or (exc_type and issubclass(exc_type, OSError)):
            return
        super().handle_error(request, client_address)


def main():
    print("=" * 60)
    print("      3D Optical Flow Geofence Visualizer Server")
    print("=" * 60)

    # Start UDP listener in background thread
    udp_thread = threading.Thread(target=start_udp_listener, daemon=True)
    udp_thread.start()

    # Start Multi-Threaded HTTP + SSE Server
    try:
        handler = TelemetryHTTPServer
        httpd = ThreadedHTTPServer(("", HTTP_PORT), handler)
        print(f"[HTTP + Stream Server] Running on http://localhost:{HTTP_PORT}")
        print(f"[GPS API] GET http://localhost:{HTTP_PORT}/api/gps | POST http://localhost:{HTTP_PORT}/api/gps/origin")
        print(f"[Session Logs API] GET http://localhost:{HTTP_PORT}/api/logs")
        print(f"[Dashboard] Open http://localhost:{HTTP_PORT} in your web browser.\n")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Web Server.")
    except Exception as e:
        print(f"[ERROR] Server error: {e}")

if __name__ == "__main__":
    main()

