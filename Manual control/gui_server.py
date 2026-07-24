#!/usr/bin/env python3
"""
Web GUI Server & UDP Forwarder for ESP8266 4-DOF Servo Arm
Serves a responsive Web UI at http://localhost:8000 for controlling 4 servos in real-time.
"""

import http.server
import socket
import socketserver
import urllib.parse
import json

UDP_IP = "192.168.137.78"  # Default Target ESP8266 IP
UDP_PORT = 8888

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP8266 4-DOF Servo Arm Controller</title>
    <style>
        :root {
            --bg-color: #181825;
            --card-bg: #1e1e2e;
            --accent: #89b4fa;
            --accent-hover: #b4befe;
            --text-color: #cdd6f4;
            --subtext: #a6adc8;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            margin: 0;
            padding: 20px;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }

        .container {
            background-color: var(--card-bg);
            border-radius: 16px;
            padding: 30px;
            width: 100%;
            max-width: 480px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
        }

        h1 {
            color: var(--accent);
            margin-top: 0;
            font-size: 1.6rem;
            text-align: center;
        }

        .ip-group {
            display: flex;
            gap: 10px;
            margin-bottom: 25px;
            align-items: center;
        }

        .ip-group label {
            font-weight: 600;
            color: var(--subtext);
        }

        .ip-group input {
            flex: 1;
            background: #313244;
            border: 1px solid #45475a;
            color: #fff;
            padding: 8px 12px;
            border-radius: 8px;
            font-size: 0.95rem;
        }

        .slider-card {
            background: #313244;
            padding: 14px 16px;
            border-radius: 12px;
            margin-bottom: 14px;
        }

        .slider-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
            font-weight: 600;
        }

        .angle-val {
            color: var(--accent);
            font-size: 1.1rem;
            font-family: monospace;
        }

        input[type=range] {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #45475a;
            outline: none;
            cursor: pointer;
            accent-color: var(--accent);
        }

        .preset-buttons {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 25px;
        }

        button {
            background: #313244;
            border: 1px solid var(--accent);
            color: var(--text-color);
            padding: 10px;
            border-radius: 8px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s ease;
        }

        button:hover {
            background: var(--accent);
            color: var(--bg-color);
        }

        .status {
            margin-top: 20px;
            font-size: 0.85rem;
            color: var(--subtext);
            text-align: center;
            min-height: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 4-DOF Servo Arm Control</h1>

        <div class="ip-group">
            <label for="ip">Target IP:</label>
            <input type="text" id="ip" value="192.168.137.78" onchange="updateTargetIP()">
        </div>

        <div class="slider-card">
            <div class="slider-header">
                <span>Servo 1 (GPIO 1 / Elbow Pitch - J3)</span>
                <span class="angle-val" id="val1">90°</span>
            </div>
            <input type="range" id="s1" min="0" max="180" value="90" oninput="sendAngles()">
        </div>

        <div class="slider-card">
            <div class="slider-header">
                <span>Servo 2 (GPIO 3 / Shoulder Pitch - J2)</span>
                <span class="angle-val" id="val2">90°</span>
            </div>
            <input type="range" id="s2" min="0" max="180" value="90" oninput="sendAngles()">
        </div>

        <div class="slider-card">
            <div class="slider-header">
                <span>Servo 3 (GPIO 5 / Base Yaw - J1)</span>
                <span class="angle-val" id="val3">90°</span>
            </div>
            <input type="range" id="s3" min="0" max="180" value="90" oninput="sendAngles()">
        </div>

        <div class="slider-card">
            <div class="slider-header">
                <span>Servo 4 (GPIO 4 / Wrist Pitch - J4)</span>
                <span class="angle-val" id="val4">90°</span>
            </div>
            <input type="range" id="s4" min="0" max="180" value="90" oninput="sendAngles()">
        </div>

        <div class="preset-buttons">
            <button onclick="setPresets(0, 0, 0, 0)">0° (Min)</button>
            <button onclick="setPresets(90, 90, 90, 90)">90° (Neutral)</button>
            <button onclick="setPresets(180, 180, 180, 180)">180° (Max)</button>
        </div>

        <div class="status" id="status">Ready</div>
    </div>

    <script>
        let targetIP = document.getElementById('ip').value;
        let lastSendTime = 0;
        let pendingSend = null;

        function updateTargetIP() {
            targetIP = document.getElementById('ip').value;
        }

        function setPresets(a1, a2, a3, a4) {
            document.getElementById('s1').value = a1;
            document.getElementById('s2').value = a2;
            document.getElementById('s3').value = a3;
            document.getElementById('s4').value = a4;
            sendAngles();
        }

        function sendAngles() {
            const a1 = document.getElementById('s1').value;
            const a2 = document.getElementById('s2').value;
            const a3 = document.getElementById('s3').value;
            const a4 = document.getElementById('s4').value;

            document.getElementById('val1').innerText = a1 + '°';
            document.getElementById('val2').innerText = a2 + '°';
            document.getElementById('val3').innerText = a3 + '°';
            document.getElementById('val4').innerText = a4 + '°';

            const now = Date.now();
            if (now - lastSendTime > 25) { // 40 Hz throttle for smooth realtime control
                executeSend(a1, a2, a3, a4);
                lastSendTime = now;
            } else {
                clearTimeout(pendingSend);
                pendingSend = setTimeout(() => {
                    executeSend(a1, a2, a3, a4);
                    lastSendTime = Date.now();
                }, 25);
            }
        }

        function executeSend(a1, a2, a3, a4) {
            fetch(`/send?ip=${targetIP}&a1=${a1}&a2=${a2}&a3=${a3}&a4=${a4}`)
                .then(res => res.json())
                .then(data => {
                    document.getElementById('status').innerText = `Sent -> IP: ${data.ip}:${data.port} | Angles: ${data.a1}°, ${data.a2}°, ${data.a3}°, ${data.a4}°`;
                })
                .catch(err => {
                    document.getElementById('status').innerText = 'Transmission error';
                });
        }
    </script>
</body>
</html>
"""

class RequestHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return  # Suppress default request logging for high-rate sliders

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/" or parsed.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_CONTENT.encode("utf-8"))
        elif parsed.path == "/send":
            params = urllib.parse.parse_qs(parsed.query)
            ip = params.get("ip", [UDP_IP])[0]
            a1 = int(params.get("a1", [90])[0])
            a2 = int(params.get("a2", [90])[0])
            a3 = int(params.get("a3", [90])[0])
            a4 = int(params.get("a4", [90])[0])

            # Send UDP packet to microcontroller
            msg = f"{a1},{a2},{a3},{a4}".encode("ascii")
            try:
                udp_socket.sendto(msg, (ip, UDP_PORT))
            except Exception as e:
                pass

            response = {"status": "ok", "ip": ip, "port": UDP_PORT, "a1": a1, "a2": a2, "a3": a3, "a4": a4}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(response).encode("utf-8"))
        else:
            self.send_error(404)

def run():
    port = 8000
    server_address = ('', port)
    httpd = socketserver.TCPServer(server_address, RequestHandler)
    print(f"Server started at http://localhost:{port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server.")

if __name__ == "__main__":
    run()
