#!/usr/bin/env python3
"""
Standalone Geofence Engine & Buzzer Listener
---------------------------------------------
Reads Optical Flow data directly from Shared Memory ('optical_flow_stream').
Calculates 2D planar position (X, Y in meters) and evaluates breach conditions
against a defined boundary (Box boundary [min_x, max_x, min_y, max_y] or Circular radius).

When a BREACH is detected:
  - Triggers PWM buzzer alert on GPIO 12 (2300 Hz) asynchronously.
  - Can optionally listen on UDP socket for compatibility.

Execution Command Example:
  python3 geofence_buzzer_listener.py --bounds -0.5 0.5 -0.5 0.5 --pin 12 --freq 2300
"""

import argparse
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from multiprocessing import resource_tracker, shared_memory

# Shared Memory configuration matching optical_flow/shm_writer.py
FLOW_SHM_NAME = "optical_flow_stream"
FLOW_SHM_MAGIC = b"FLOW"
FLOW_SHM_HEADER_FORMAT = "<4sII"
FLOW_SHM_RECORD_FORMAT = "<11d"
FLOW_SHM_HEADER_SIZE = struct.calcsize(FLOW_SHM_HEADER_FORMAT)
FLOW_SHM_RECORD_SIZE = struct.calcsize(FLOW_SHM_RECORD_FORMAT)
FLOW_MAX_SAMPLES = 120

DEFAULT_UDP_IP = "0.0.0.0"
DEFAULT_UDP_PORT = 5006
GPIO_PIN = 12
BUZZER_FREQ = 2300

# Check for gpiozero library
try:
    from gpiozero import PWMOutputDevice
    HAS_GPIOZERO = True
except ImportError:
    HAS_GPIOZERO = False


def safe_unregister_shm(shm):
    """Unregisters shared memory block from Python resource tracker."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


def read_latest_flow_shm():
    """
    Reads the latest Optical Flow sample directly from shared memory segment ('optical_flow_stream').
    Returns dict with x_m, y_m, alt, vx, vy or None if unreadable.
    """
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
            "timestamp": ts,
            "x_m": x_cm / 100.0,
            "y_m": y_cm / 100.0,
            "alt_m": alt,
            "vx": vx,
            "vy": vy,
            "heading": heading
        }
    except Exception:
        return None


class BuzzerController:
    def __init__(self, pin=GPIO_PIN, frequency=BUZZER_FREQ, force_subprocess=False):
        self.pin = pin
        self.frequency = frequency
        self.force_subprocess = force_subprocess
        self.native_pwm = None
        self.is_active = False
        self.worker_thread = None

        if HAS_GPIOZERO and not self.force_subprocess:
            try:
                self.native_pwm = PWMOutputDevice(self.pin, frequency=self.frequency)
                print(f"[Buzzer] Native gpiozero PWM initialized on GPIO {self.pin} ({self.frequency}Hz)")
            except Exception as e:
                print(f"[Buzzer] Native gpiozero init failed ({e}), falling back to subprocess execution.")
                self.native_pwm = None

    def start_alarm(self):
        """Starts pulsing alarm sound ('beeep ... beeep') asynchronously in background."""
        if self.is_active:
            return
        self.is_active = True
        if self.native_pwm is not None:
            self.worker_thread = threading.Thread(target=self._native_pulse_loop, daemon=True)
            self.worker_thread.start()
        else:
            self.worker_thread = threading.Thread(target=self._subprocess_alarm_loop, daemon=True)
            self.worker_thread.start()

    def stop_alarm(self):
        """Stops alarm sound immediately."""
        self.is_active = False
        if self.native_pwm is not None:
            try:
                self.native_pwm.off()
            except Exception:
                pass

    def _native_pulse_loop(self):
        """Pulsing alarm pattern: beeep (0.25s ON) ... pause (0.15s OFF)."""
        while self.is_active:
            try:
                if self.native_pwm is not None:
                    self.native_pwm.value = 0.5
                    time.sleep(0.25)
                    self.native_pwm.off()
                    time.sleep(0.15)
            except Exception:
                time.sleep(0.1)
        if self.native_pwm is not None:
            try:
                self.native_pwm.off()
            except Exception:
                pass

    def _subprocess_alarm_loop(self):
        """Subprocess execution matching pulsing pattern."""
        cmd = [
            "python3", "-c",
            f"from gpiozero import PWMOutputDevice; from time import sleep; p = PWMOutputDevice({self.pin}, frequency={self.frequency}); p.value = 0.5; sleep(0.25); p.off()"
        ]
        while self.is_active:
            try:
                subprocess.run(cmd, timeout=0.8, check=False)
                time.sleep(0.15)
            except Exception:
                time.sleep(0.1)

    def stop(self):
        """Clean shutdown for buzzer hardware."""
        self.stop_alarm()
        if self.native_pwm is not None:
            try:
                self.native_pwm.close()
            except Exception:
                pass


class StandaloneGeofenceEngine:
    """
    Evaluates Optical Flow positions against boundary boundaries.
    Default boundary: Box [-0.5m, +0.5m] on X and Y axis (1m x 1m area).
    """
    def __init__(self, min_x=-0.5, max_x=0.5, min_y=-0.5, max_y=0.5, max_radius=None):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.max_radius = max_radius

    def is_breached(self, x_m, y_m):
        if self.max_radius is not None:
            dist = math.hypot(x_m, y_m)
            return dist > self.max_radius
        
        breach_x = x_m < self.min_x or x_m > self.max_x
        breach_y = y_m < self.min_y or y_m > self.max_y
        return breach_x or breach_y


def main():
    parser = argparse.ArgumentParser(description="Standalone Optical Flow Shared Memory Geofence Engine")
    parser.add_argument("--bounds", type=float, nargs=4, default=[-0.5, 0.5, -0.5, 0.5],
                        metavar=('MIN_X', 'MAX_X', 'MIN_Y', 'MAX_Y'),
                        help="Box geofence boundary coordinates in meters (default: -0.5 0.5 -0.5 0.5)")
    parser.add_argument("--radius", type=float, default=None, help="Circular geofence radius limit in meters (overrides bounds)")
    parser.add_argument("--pin", type=int, default=GPIO_PIN, help=f"GPIO pin for buzzer (default {GPIO_PIN})")
    parser.add_argument("--freq", type=int, default=BUZZER_FREQ, help=f"PWM frequency in Hz (default {BUZZER_FREQ})")
    parser.add_argument("--subprocess", action="store_true", help="Force using python3 -c subprocess command execution")
    parser.add_argument("--enable-udp-fallback", action="store_true", help="Also listen on UDP socket for webserver breach notifications")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT, help=f"UDP fallback bind port (default {DEFAULT_UDP_PORT})")
    args = parser.parse_args()

    engine = StandaloneGeofenceEngine(
        min_x=args.bounds[0], max_x=args.bounds[1],
        min_y=args.bounds[2], max_y=args.bounds[3],
        max_radius=args.radius
    )

    buzzer = BuzzerController(pin=args.pin, frequency=args.freq, force_subprocess=args.subprocess)

    print("=" * 65)
    print("      STANDALONE OPTICAL FLOW GEOFENCE ENGINE & BUZZER")
    print("=" * 65)
    print(f"[Input Source] Shared Memory Segment: '{FLOW_SHM_NAME}'")
    if args.radius is not None:
        print(f"[Geofence Shape] Circular Radius: {args.radius} meters")
    else:
        print(f"[Geofence Shape] Box Boundary X: [{args.bounds[0]}m, {args.bounds[1]}m] | Y: [{args.bounds[2]}m, {args.bounds[3]}m]")
    print(f"[Hardware Alert] GPIO Pin: {args.pin} | PWM Frequency: {args.freq} Hz")
    print(f"[Exec Mode] {'Subprocess python3 -c' if args.subprocess or not HAS_GPIOZERO else 'Native gpiozero PWM'}")
    
    udp_sock = None
    if getattr(args, 'enable_udp_fallback', False):
        try:
            udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_sock.bind((DEFAULT_UDP_IP, args.udp_port))
            udp_sock.settimeout(0.01)
            print(f"[UDP Fallback] Bound on port {args.udp_port}")
        except Exception as e:
            print(f"[UDP Fallback Warning] Failed to bind: {e}")

    print("Press Ctrl+C to stop.\n")

    is_breached = False
    last_shm_log_time = 0
    shm_available = False

    try:
        while True:
            loop_start = time.time()
            flow_data = read_latest_flow_shm()

            udp_breach_detected = False
            if udp_sock is not None:
                try:
                    data, addr = udp_sock.recvfrom(1024)
                    msg = data.decode("utf-8", errors="ignore").strip().upper()
                    if "BREACH" in msg or "1" in msg or "TRUE" in msg:
                        udp_breach_detected = True
                except socket.timeout:
                    pass
                except Exception:
                    pass

            if flow_data is not None:
                if not shm_available:
                    print(f"[{time.strftime('%H:%M:%S')}] [SHM] Connected to Optical Flow Shared Memory segment successfully.")
                    shm_available = True

                x_m = flow_data["x_m"]
                y_m = flow_data["y_m"]

                shm_breached = engine.is_breached(x_m, y_m)
                currently_breached = shm_breached or udp_breach_detected

                if currently_breached and not is_breached:
                    print(f"[{time.strftime('%H:%M:%S')}] >>> GEOFENCE BREACH DETECTED! Pos: ({x_m:.2f}m, {y_m:.2f}m) | Sounding Alarm...")
                    is_breached = True
                    buzzer.start_alarm()
                elif not currently_breached and is_breached:
                    print(f"[{time.strftime('%H:%M:%S')}] >>> Geofence SAFE (Breach Cleared). Pos: ({x_m:.2f}m, {y_m:.2f}m) | Silencing Alarm.")
                    is_breached = False
                    buzzer.stop_alarm()

                if time.time() - last_shm_log_time > 2.0:
                    status_str = "BREACH" if is_breached else "SAFE"
                    print(f"[{time.strftime('%H:%M:%S')}] [SHM Engine] Pos: X={x_m:+.2f}m, Y={y_m:+.2f}m | Status: {status_str}")
                    last_shm_log_time = time.time()

            else:
                if shm_available:
                    print(f"[{time.strftime('%H:%M:%S')}] [SHM] Lost connection to shared memory.")
                    shm_available = False

                if udp_breach_detected and not is_breached:
                    print(f"[{time.strftime('%H:%M:%S')}] >>> GEOFENCE BREACH DETECTED (via UDP fallback)! Sounding Alarm...")
                    is_breached = True
                    buzzer.start_alarm()
                elif not udp_breach_detected and is_breached and udp_sock is not None:
                    is_breached = False
                    buzzer.stop_alarm()

                if time.time() - last_shm_log_time > 5.0:
                    print(f"[{time.strftime('%H:%M:%S')}] Waiting for shared memory segment '{FLOW_SHM_NAME}' to be published by optical_flow_stream...")
                    last_shm_log_time = time.time()

            time.sleep(0.05)  # 20 Hz evaluation loop

    except KeyboardInterrupt:
        print("\nStopping Standalone Geofence Engine Service...")
    finally:
        buzzer.stop()
        if udp_sock is not None:
            udp_sock.close()
        print("Geofence Engine stopped cleanly.")


if __name__ == "__main__":
    main()

