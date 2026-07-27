#!/usr/bin/env python3
"""
Master Drone Service Orchestrator (Raspberry Pi Edition)
---------------------------------------------------------
Launches and manages all drone services locally on the Raspberry Pi:
1. HMC5883L Compass (`hmc5883l.py`) -> Shared memory stream
2. Optical Flow Stream (`optical_flow_stream.py -stream`) -> Shared memory stream
3. Geofence Alarm Buzzer Listener (`geofence_buzzer_listener.py`) -> Port 5006
4. Telemetry UDP Bridge (`send_attitude_udp.py`) -> Port 5005
5. Web Dashboard Server (`web_server.py`) -> Port 8000 (HTTP + SSE)

Usage:
    python3 run_all_pi.py
"""

import os
import sys
import time
import subprocess
import signal
import argparse

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(PROJECT_DIR, "logs")

SERVICES = [
    {
        "name": "HMC5883L Compass",
        "cmd": [sys.executable, os.path.join(PROJECT_DIR, "hmc5883l.py")],
        "log": os.path.join(LOG_DIR, "hmc5883l.log"),
    },
    {
        "name": "Optical Flow Stream",
        "cmd": [sys.executable, os.path.join(PROJECT_DIR, "optical_flow_stream.py"), "-stream"],
        "log": os.path.join(LOG_DIR, "optical_flow_stream.log"),
    },
    {
        "name": "Buzzer Alarm Listener",
        "cmd": [sys.executable, os.path.join(PROJECT_DIR, "geofence_buzzer_listener.py")],
        "log": os.path.join(LOG_DIR, "geofence_buzzer_listener.log"),
    },
    {
        "name": "Telemetry UDP Bridge",
        "cmd": [sys.executable, os.path.join(PROJECT_DIR, "send_attitude_udp.py"), "--ip", "127.0.0.1", "--port", "5005"],
        "log": os.path.join(LOG_DIR, "send_attitude_udp.log"),
    },
    {
        "name": "Web Dashboard Server",
        "cmd": [sys.executable, os.path.join(PROJECT_DIR, "web_server.py")],
        "log": os.path.join(LOG_DIR, "web_server.log"),
    },
]

processes = []

def stop_all(signum=None, frame=None):
    print("\n[Orchestrator] Stopping all Raspberry Pi drone services...")
    for item in processes:
        proc = item["proc"]
        name = item["name"]
        if proc.poll() is None:
            print(f"  -> Terminating {name} (PID: {proc.pid})...")
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
    print("[Orchestrator] All services stopped cleanly.")
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description="Master Raspberry Pi Autonomous Drone Services Orchestrator")
    parser.add_argument("--no-web", action="store_true", help="Do not start web_server.py")
    args = parser.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    print("=" * 65)
    print("       OPTICAL FLOW DRONE - FULL RASPBERRY PI SUITE")
    print("=" * 65)
    print(f"Project Directory: {PROJECT_DIR}")
    print(f"Logs Directory:    {LOG_DIR}\n")

    for svc in SERVICES:
        if args.no_web and "web_server.py" in svc["cmd"][1]:
            continue

        print(f"[*] Starting {svc['name']}...")
        log_file = open(svc["log"], "a", encoding="utf-8")
        proc = subprocess.Popen(
            svc["cmd"],
            cwd=PROJECT_DIR,
            stdout=log_file,
            stderr=subprocess.STDOUT
        )
        processes.append({
            "name": svc["name"],
            "proc": proc,
            "log_file": log_file
        })
        time.sleep(0.5)

    print("\n[+] All services successfully launched!")
    print("    - Web Dashboard:  http://<raspi-ip>:8000")
    print("    - Telemetry UDP:  127.0.0.1:5005")
    print("    - Buzzer Listener: 127.0.0.1:5006")
    print("Press Ctrl+C to stop all services.\n")

    try:
        while True:
            for item in processes:
                proc = item["proc"]
                name = item["name"]
                ret = proc.poll()
                if ret is not None:
                    print(f"[WARNING] Service '{name}' exited unexpectedly with code {ret}. Check log: {item['name']}")
            time.sleep(2.0)
    except KeyboardInterrupt:
        stop_all()

if __name__ == "__main__":
    main()
