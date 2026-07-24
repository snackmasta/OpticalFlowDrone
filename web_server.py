import http.server
import socketserver
import socket
import json
import threading
import time
import os
import sys
import urllib.parse

# Server configuration
HTTP_PORT = 8000
UDP_PORT = 5005
UDP_IP = "0.0.0.0"

# Global state to store latest telemetry packet and connected client queues
latest_telemetry = {
    "timestamp": time.time(),
    "rotation": {"quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}, "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}},
    "translation": {"position": {"x": 0, "y": 0, "z": 0}, "velocity": {"x": 0, "y": 0, "z": 0}, "linear_accel": {"x": 0, "y": 0, "z": 0}},
    "heading": 0.0,
    "status": "waiting"
}

connected_sse_clients = []
clients_lock = threading.Lock()

# Global UDP socket for ESP8266 4-DOF Servo Arm (Port 8888)
udp_arm_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_arm_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

class TelemetryHTTPServer(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        # Serve files from the ./web directory
        web_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
        super().__init__(*args, directory=web_dir, **kwargs)

    def do_GET(self):
        if self.path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return

        if self.path.startswith('/send'):
            # REST API Gateway Endpoint to send UDP packets to ESP8266 Robot Arm (Port 8888)
            parsed_url = urllib.parse.urlparse(self.path)
            query_params = urllib.parse.parse_qs(parsed_url.query)

            ip = query_params.get('ip', ['192.168.137.94'])[0].strip()
            if not ip:
                ip = '192.168.137.94'

            try:
                a1 = int(query_params.get('a1', [90])[0])
                a2 = int(query_params.get('a2', [90])[0])
                a3 = int(query_params.get('a3', [90])[0])
                a4 = int(query_params.get('a4', [90])[0])
            except (ValueError, TypeError, IndexError):
                a1, a2, a3, a4 = 90, 90, 90, 90

            # Constrain angles to valid Servo range [0, 180]
            a1 = max(0, min(180, a1))
            a2 = max(0, min(180, a2))
            a3 = max(0, min(180, a3))
            a4 = max(0, min(180, a4))

            # Send ASCII UDP packet: "a1,a2,a3,a4" to target IP on port 8888 using persistent socket
            payload_str = f"{a1},{a2},{a3},{a4}"
            try:
                udp_arm_socket.sendto(payload_str.encode('ascii'), (ip, 8888))
                response_data = {
                    "status": "ok",
                    "ip": ip,
                    "port": 8888,
                    "a1": a1,
                    "a2": a2,
                    "a3": a3,
                    "a4": a4
                }
            except Exception as e:
                response_data = {
                    "status": "error",
                    "message": str(e),
                    "ip": ip,
                    "port": 8888,
                    "a1": a1,
                    "a2": a2,
                    "a3": a3,
                    "a4": a4
                }

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(response_data).encode('utf-8'))
            return

        if self.path == '/stream':
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
                    time.sleep(0.02) # 50Hz streaming
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
        else:
            try:
                super().do_GET()
            except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                pass

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def log_message(self, format, *args):
        # Suppress routine GET logging for clean console output
        try:
            if len(args) > 0 and isinstance(args[0], str) and "GET /stream" in args[0]:
                return
        except Exception:
            pass
        super().log_message(format, *args)



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
        except Exception:
            pass


class QuietTCPServer(socketserver.TCPServer):
    def handle_error(self, request, client_address):
        _, exc, _ = sys.exc_info()
        if exc and isinstance(exc, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)):
            return
        super().handle_error(request, client_address)

def main():
    print("=" * 60)
    print("      3D VR Controller Trajectory Web Server")
    print("=" * 60)

    # Start UDP listener in background thread
    udp_thread = threading.Thread(target=start_udp_listener, daemon=True)
    udp_thread.start()

    # Start HTTP + SSE Server
    try:
        handler = TelemetryHTTPServer
        httpd = QuietTCPServer(("", HTTP_PORT), handler)
        print(f"[HTTP + Stream Server] Running on http://localhost:{HTTP_PORT}")
        print(f"[Dashboard] Open http://localhost:{HTTP_PORT} in your web browser.\n")
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping Web Server.")
    except Exception as e:
        print(f"[ERROR] Server error: {e}")

if __name__ == "__main__":
    main()
