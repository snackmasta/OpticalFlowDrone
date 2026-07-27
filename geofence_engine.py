#!/usr/bin/env python3
"""
Geofence Engine Module & Standalone Service
-------------------------------------------
Handles 3D spatial boundary checking, geofence state management (SAFE vs BREACH),
UDP status broadcasting, and GPIO PWM buzzer alarm control.
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import threading
import time

DEFAULT_UDP_IP = "0.0.0.0"
DEFAULT_UDP_PORT = 5006
DEFAULT_BUZZER_IP = os.getenv("BUZZER_UDP_IP", "127.0.0.1")
DEFAULT_BUZZER_PORT = int(os.getenv("BUZZER_UDP_PORT", "5006"))
GPIO_PIN = 12
BUZZER_FREQ = 2300

# Check for gpiozero library
try:
    from gpiozero import PWMOutputDevice
    HAS_GPIOZERO = True
except ImportError:
    HAS_GPIOZERO = False


class BuzzerController:
    """Manages PWM alarm sound on GPIO pin."""

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


class GeofenceEngine:
    """
    Core Geofence Engine for 3D boundary checking, status management,
    UDP packet broadcasting, and alarm actuation.
    """

    def __init__(self, size_x=1.5, size_y=1.5, size_z=1.5, center=(0.0, 1.5, 0.0)):
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.center_x, self.center_y, self.center_z = center
        self.is_breached = False
        self.last_update_ts = time.time()

    def set_center(self, x, y, z):
        """Recenter geofence target coordinates."""
        self.center_x = x
        self.center_y = y
        self.center_z = z

    def check_position(self, x, y, z):
        """
        Evaluates 3D position relative to geofence boundaries.
        Returns True if breached, False if safe.
        """
        dx = abs(x - self.center_x)
        dy = abs(y - self.center_y)
        dz = abs(z - self.center_z)

        half_x = self.size_x / 2.0
        half_y = self.size_y / 2.0
        half_z = self.size_z / 2.0

        self.is_breached = (dx > half_x or dy > half_y or dz > half_z)
        self.last_update_ts = time.time()
        return self.is_breached

    def update_breach_status(self, is_breached):
        """Manually update breach status."""
        self.is_breached = bool(is_breached)
        self.last_update_ts = time.time()
        return self.is_breached


def send_geofence_status_udp(is_breached, target_ip=DEFAULT_BUZZER_IP, target_port=DEFAULT_BUZZER_PORT):
    """
    Sends UDP packet to the Geofence Listener (default 192.168.137.54:5006 & broadcast).
    Payload: "BREACH" when breached, "SAFE" when inside geofence.
    """
    try:
        payload = b"BREACH" if is_breached else b"SAFE"
        buzzer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            buzzer_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass

        buzzer_sock.sendto(payload, (target_ip, target_port))

        if target_ip != "127.0.0.1":
            try:
                buzzer_sock.sendto(payload, ("127.0.0.1", target_port))
            except Exception:
                pass
            try:
                buzzer_sock.sendto(payload, ("<broadcast>", target_port))
            except Exception:
                pass

        buzzer_sock.close()
        print(f"[Geofence Engine] Transmitted status: {'BREACH' if is_breached else 'SAFE'} -> {target_ip}:{target_port}")
    except Exception as e:
        print(f"[Geofence Engine UDP Error] {e}")


def run_listener_service(ip=DEFAULT_UDP_IP, port=DEFAULT_UDP_PORT, pin=GPIO_PIN, freq=BUZZER_FREQ, force_subprocess=False):
    """Runs the UDP listener service to trigger hardware buzzer on breach signals."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass

    try:
        sock.bind((ip, port))
    except Exception as e:
        print(f"[ERROR] Geofence Engine failed to bind socket on {ip}:{port}: {e}")
        sys.exit(1)

    sock.settimeout(0.1)

    print("=" * 65)
    print("      GEOFENCE ENGINE LISTENER SERVICE")
    print("=" * 65)
    print(f"[UDP Listener] Listening on {ip}:{port}")
    print(f"[Hardware] GPIO Pin: {pin} | PWM Frequency: {freq} Hz")
    print(f"[Mode] {'Subprocess python3 -c' if force_subprocess or not HAS_GPIOZERO else 'Native gpiozero PWM'}")
    print("Press Ctrl+C to stop.\n")

    buzzer = BuzzerController(pin=pin, frequency=freq, force_subprocess=force_subprocess)
    engine = GeofenceEngine()
    last_packet_ts = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                last_packet_ts = time.time()
                msg = data.decode("utf-8", errors="ignore").strip().upper()

                if "BREACH" in msg or '"STATUS": "BREACH"' in msg or msg == "1" or "TRUE" in msg:
                    if not engine.is_breached:
                        print(f"[{time.strftime('%H:%M:%S')}] >>> GEOFENCE BREACH DETECTED from {addr[0]}! Sounding alarm...")
                    engine.update_breach_status(True)
                    buzzer.start_alarm()
                elif "SAFE" in msg or "INSIDE" in msg or msg == "0" or "FALSE" in msg:
                    if engine.is_breached:
                        print(f"[{time.strftime('%H:%M:%S')}] >>> Geofence SAFE (Breach Cleared) from {addr[0]}. Silencing alarm.")
                    engine.update_breach_status(False)
                    buzzer.stop_alarm()
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[Socket Error] {e}")

            # Auto-timeout breach alert if no packets received for >3 seconds
            if engine.is_breached and (time.time() - last_packet_ts > 3.0):
                print(f"[{time.strftime('%H:%M:%S')}] Telemetry timeout (>3s). Silencing alarm.")
                engine.update_breach_status(False)
                buzzer.stop_alarm()

    except KeyboardInterrupt:
        print("\nStopping Geofence Engine Service...")
    finally:
        buzzer.stop()
        sock.close()
        print("Geofence Engine stopped cleanly.")


def main():
    parser = argparse.ArgumentParser(description="Standalone Geofence Engine Service")
    parser.add_argument("--ip", type=str, default=DEFAULT_UDP_IP, help="UDP bind IP address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT, help="UDP bind port (default 5006)")
    parser.add_argument("--pin", type=int, default=GPIO_PIN, help="GPIO pin for buzzer (default 12)")
    parser.add_argument("--freq", type=int, default=BUZZER_FREQ, help="PWM frequency in Hz (default 2300)")
    parser.add_argument("--subprocess", action="store_true", help="Force using python3 -c subprocess command execution")
    args = parser.parse_args()

    run_listener_service(ip=args.ip, port=args.port, pin=args.pin, freq=args.freq, force_subprocess=args.subprocess)


if __name__ == "__main__":
    main()
