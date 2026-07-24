import http.server
import socketserver
import socket
import json
import threading
import time
import os
import sys
import csv
from urllib.parse import parse_qs, urlparse, unquote

# Server configuration
HTTP_PORT = 8000
UDP_PORT = 5005
UDP_IP = "0.0.0.0"

# Directories to search for session logs
LOG_DIRECTORIES = ["recordings", "hasil_dan_pembahasan", "."]

# Global state to store latest telemetry packet and connected client queues
latest_telemetry = {
    "timestamp": time.time(),
    "rotation": {"quaternion": {"w": 0.7071, "x": 0.7071, "y": 0, "z": 0}, "euler": {"roll": 90.0, "pitch": 0.0, "yaw": 0.0}},
    "translation": {"position": {"x": 0, "y": 0, "z": 0}, "velocity": {"x": 0, "y": 0, "z": 0}, "linear_accel": {"x": 0, "y": 0, "z": 0}},
    "heading": 0.0,
    "status": "waiting"
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
            except (ConnectionResetError, BrokenPipeError):
                pass
            finally:
                with clients_lock:
                    if client_queue in connected_sse_clients:
                        connected_sse_clients.remove(client_queue)

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

    def log_message(self, format, *args):
        # Suppress routine GET logging for clean console output
        try:
            if len(args) > 0 and isinstance(args[0], str) and ("GET /stream" in args[0] or "GET /api/" in args[0]):
                return
        except Exception:
            pass
        super().log_message(format, *args)


# Arm destination UDP settings
ARM_UDP_IP = "192.168.137.229"
ARM_UDP_PORT = 8888

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

        # Servo 1 (Elbow)=90, Servo 2 (Shoulder Pitch)=shoulder_angle, Servo 3 (Base Yaw)=90, Servo 4 (Wrist Pitch)=wrist_angle
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
    allow_reuse_address = True


def main():
    print("=" * 60)
    print("      3D VR Controller Trajectory Web Server")
    print("=" * 60)

    # Start UDP listener in background thread
    udp_thread = threading.Thread(target=start_udp_listener, daemon=True)
    udp_thread.start()

    # Start Multi-Threaded HTTP + SSE Server
    try:
        handler = TelemetryHTTPServer
        httpd = ThreadedHTTPServer(("", HTTP_PORT), handler)
        print(f"[HTTP + Stream Server] Running on http://localhost:{HTTP_PORT}")
        print(f"[Session Logs API] GET http://localhost:{HTTP_PORT}/api/logs")
        print(f"[Dashboard] Open http://localhost:{HTTP_PORT} in your web browser.\n")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Web Server.")
    except Exception as e:
        print(f"[ERROR] Server error: {e}")

if __name__ == "__main__":
    main()
