#!/usr/bin/env python3
"""
Drone Integrated System Launcher & Process Supervisor
------------------------------------------------------
Manages and orchestrates the five core microservices:
1. optical_flow_stream.py (Camera optical flow tracking & shared memory publisher)
2. send_attitude_udp.py (AHRS, Sensor Fusion & Telemetry UDP broadcaster)
3. geofence_buzzer_listener.py (Hardware GPIO Buzzer listener on UDP port 5006)
4. web_server.py (HTTP / SSE Dashboard & REST APIs on port 8000)
5. sensor_fusion module dependency monitor
"""

import os
import sys
import time
import signal
import subprocess
import threading
import argparse

# Absolute directory paths
SUITE_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(SUITE_DIR)

PROCESS_CONFIGS = [
    {
        "name": "Optical Flow Stream",
        "cmd": [sys.executable, "optical_flow_stream.py", "-stream"],
        "cwd": SUITE_DIR,
        "critical": False,  # Will fallback to synthetic data in telemetry sender if hardware camera not attached
    },
    {
        "name": "Geofence Buzzer Listener",
        "cmd": [sys.executable, "geofence_buzzer_listener.py", "--port", "5006"],
        "cwd": SUITE_DIR,
        "critical": True,
    },
    {
        "name": "Web Server & Telemetry Hub",
        "cmd": [sys.executable, "web_server.py"],
        "cwd": SUITE_DIR,
        "critical": True,
    },
    {
        "name": "Telemetry Broadcaster",
        "cmd": [sys.executable, "send_attitude_udp.py", "--ip", "127.0.0.1", "--port", "5005", "--rate", "50"],
        "cwd": SUITE_DIR,
        "critical": True,
    },
]

running_processes = {}
shutdown_requested = False


def log_stream_reader(name, pipe):
    """Reads stdout/stderr lines from sub-processes and prefixes output with service tag."""
    try:
        with pipe:
            for line in iter(pipe.readline, ''):
                if not line:
                    break
                print(f"[{name}] {line.strip()}", flush=True)
    except Exception:
        pass


def start_process(config):
    """Starts a process and spawns output monitor threads."""
    name = config["name"]
    cmd = config["cmd"]
    cwd = config["cwd"]
    
    # Ensure PYTHONPATH includes the current directory and parent directory if needed
    env = os.environ.copy()
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = os.path.pathsep.join([SUITE_DIR, PARENT_DIR, existing_pythonpath]).strip(os.path.pathsep)

    try:
        proc = subprocess.Popen(
            cmd,
            cwd=cwd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )
        t = threading.Thread(target=log_stream_reader, args=(name, proc.stdout), daemon=True)
        t.start()
        print(f"[LAUNCHER] Started service '{name}' (PID: {proc.pid})")
        return proc
    except Exception as e:
        print(f"[LAUNCHER ERROR] Failed to start '{name}': {e}")
        return None


def terminate_all():
    """Cleanly stops all running processes."""
    global shutdown_requested
    if shutdown_requested:
        return
    shutdown_requested = True

    print("\n[LAUNCHER] Shutting down drone integrated system processes...")
    for name, proc in list(running_processes.items()):
        if proc and proc.poll() is None:
            print(f"[LAUNCHER] Terminating '{name}' (PID: {proc.pid})...")
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                print(f"[LAUNCHER] Force killing '{name}'...")
                proc.kill()
            except Exception as e:
                print(f"[LAUNCHER ERROR] Error terminating '{name}': {e}")

    print("[LAUNCHER] All services stopped cleanly.")


def signal_handler(sig, frame):
    terminate_all()
    sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description="Drone Integrated System Supervisor Launcher")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Target UDP IP for telemetry broadcast (default: 127.0.0.1)")
    parser.add_argument("--no-flow", action="store_true", help="Disable optical flow camera process (useful for hardware-less testing)")
    args = parser.parse_args()

    # Override Telemetry Broadcaster IP if provided
    for cfg in PROCESS_CONFIGS:
        if cfg["name"] == "Telemetry Broadcaster":
            cfg["cmd"] = [sys.executable, "send_attitude_udp.py", "--ip", args.ip, "--port", "5005", "--rate", "50"]

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 70)
    print("        DRONE OPTICAL FLOW & SENSOR FUSION SYSTEM LAUNCHER")
    print("=" * 70)
    print(f"Base Suite Path : {SUITE_DIR}")
    print(f"Target Telemetry: {args.ip}:5005")
    print("Press Ctrl+C to terminate all services.\n")

    for config in PROCESS_CONFIGS:
        if args.no_flow and config["name"] == "Optical Flow Stream":
            print("[LAUNCHER] Skipping Optical Flow Stream (--no-flow specified).")
            continue

        proc = start_process(config)
        if proc:
            running_processes[config["name"]] = proc
        time.sleep(0.5)

    # Monitor loop
    try:
        while not shutdown_requested:
            for config in PROCESS_CONFIGS:
                name = config["name"]
                if name not in running_processes:
                    continue

                proc = running_processes[name]
                retcode = proc.poll()
                if retcode is not None:
                    print(f"[LAUNCHER WARNING] Service '{name}' exited with code {retcode}.")
                    if not shutdown_requested and config.get("critical", False):
                        print(f"[LAUNCHER] Restarting critical service '{name}'...")
                        new_proc = start_process(config)
                        if new_proc:
                            running_processes[name] = new_proc
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        terminate_all()


if __name__ == "__main__":
    main()
