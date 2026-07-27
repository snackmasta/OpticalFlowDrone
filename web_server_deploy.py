import http.server
import socketserver
import socket
import json
import threading
import time
import os
import sys
import math
from urllib.parse import parse_qs, urlparse

# Deployment Server Configuration
HTTP_PORT = int(os.getenv("DEPLOYMENT_HTTP_PORT", "8000"))
UDP_PORT = int(os.getenv("DEPLOYMENT_UDP_PORT", "5005"))
UDP_IP = "0.0.0.0"

# Earth radius for GPS 2D planar projection
EARTH_RADIUS_M = 6378137.0

# GPS and 2D Cartesian Plane state
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

origin_initialized = False

def geo_to_2d_plane(lat, lon, origin_lat, origin_lon):
    """Projects WGS-84 Geographic (Lat, Lon) to 2D Local Cartesian Plane (X, Y) in meters."""
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

        # Set reference origin to first valid GPS fix coordinate automatically
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

# Global telemetry state and SSE clients list
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

# Buzzer destination UDP settings
BUZZER_UDP_IP = os.getenv("BUZZER_UDP_IP", "127.0.0.1")
BUZZER_UDP_PORT = int(os.getenv("BUZZER_UDP_PORT", "5006"))

def send_geofence_buzzer_udp(is_breached):
    """
    Sends UDP packet to the Geofence Buzzer Listener on Raspberry Pi.
    Payload: "BREACH" when breached, "SAFE" when inside geofence.
    """
    try:
        payload = b"BREACH" if is_breached else b"SAFE"
        buzzer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            buzzer_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass

        buzzer_sock.sendto(payload, (BUZZER_UDP_IP, BUZZER_UDP_PORT))

        if BUZZER_UDP_IP != "127.0.0.1":
            try:
                buzzer_sock.sendto(payload, ("127.0.0.1", BUZZER_UDP_PORT))
            except Exception:
                pass

        try:
            buzzer_sock.sendto(payload, ("<broadcast>", BUZZER_UDP_PORT))
        except Exception:
            pass

        buzzer_sock.close()
        print(f"[Buzzer UDP] Transmitted status: {'BREACH' if is_breached else 'SAFE'} -> {BUZZER_UDP_IP}:{BUZZER_UDP_PORT}")
    except Exception as e:
        print(f"[Buzzer UDP Error] {e}")


class DeploymentTelemetryHTTPServer(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve frontend web assets from ./web
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
            # SSE streaming endpoint for map and 3D geofence telemetry
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


def start_udp_listener():
    """
    Listens for incoming UDP telemetry packets from main.py or geofence_engine.py
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
        except Exception:
            pass


class ThreadedHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    daemon_threads = True
    allow_reuse_address = os.name != 'nt'

    def handle_error(self, request, client_address):
        exc_type, exc_val, _ = sys.exc_info()
        if exc_type in (ConnectionAbortedError, ConnectionResetError, BrokenPipeError) or (exc_type and issubclass(exc_type, OSError)):
            return
        super().handle_error(request, client_address)


def main():
    print("=" * 60)
    print("      Production Deployment Web Server (Geofence & Map)")
    print("=" * 60)

    # Start background UDP listener
    udp_thread = threading.Thread(target=start_udp_listener, daemon=True)
    udp_thread.start()

    # Start HTTP + SSE server
    try:
        handler = DeploymentTelemetryHTTPServer
        httpd = ThreadedHTTPServer(("", HTTP_PORT), handler)
        print(f"[HTTP Server] Running on http://localhost:{HTTP_PORT}")
        print(f"[Features] 2D Map & 3D Geofence Live Telemetry Active")
        print(f"[Clean Deployment Mode] Session logging & debug tool endpoints stripped.\n")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Deployment Web Server.")
    except Exception as e:
        print(f"[ERROR] Server error: {e}")

if __name__ == "__main__":
    main()
