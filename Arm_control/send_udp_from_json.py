import socket
import json
import os

config_path = os.path.join(os.path.dirname(__file__), "slider_config.json")

# 1. Read JSON file
with open(config_path, "r", encoding="utf-8-sig") as f:
    cfg = json.load(f)

ip = cfg.get("ip", "192.168.137.54")
port = cfg.get("port", 8888)
s1, s2, s3, s4 = cfg["servo1"], cfg["servo2"], cfg["servo3"], cfg["servo4"]

# 2. Format UDP string payload: "servo1,servo2,servo3,servo4"
payload = f"{s1},{s2},{s3},{s4}".encode("ascii")

# 3. Send UDP packet
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(payload, (ip, port))

print(f"Sent UDP packet to {ip}:{port} -> Payload: {payload.decode('ascii')}")
