#!/usr/bin/env python3
"""
Geofence Alarm Buzzer Listener
-----------------------------------
Listens for incoming UDP breach packets on port 5006 (by default).
When a BREACH packet is received, executes the PWM buzzer on GPIO 12 (2300 Hz) in a loop.
When a SAFE packet is received or breach stops, silences the buzzer immediately.

Requested Command Executed on Breach Loop:
python3 -c "from gpiozero import PWMOutputDevice; from time import sleep; p = PWMOutputDevice(12, frequency=2300); p.value = 0.5; sleep(1); p.off()"
"""

import argparse
import json
import os
import socket
import subprocess
import sys
import time

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


class BuzzerController:
    def __init__(self, pin=GPIO_PIN, frequency=BUZZER_FREQ, force_subprocess=False):
        self.pin = pin
        self.frequency = frequency
        self.force_subprocess = force_subprocess
        self.native_pwm = None

        if HAS_GPIOZERO and not self.force_subprocess:
            try:
                self.native_pwm = PWMOutputDevice(self.pin, frequency=self.frequency)
                print(f"[Buzzer] Native gpiozero PWM initialized on GPIO {self.pin} ({self.frequency}Hz)")
            except Exception as e:
                print(f"[Buzzer] Native gpiozero init failed ({e}), falling back to subprocess execution.")
                self.native_pwm = None

    def run_beep_cycle(self, on_duration=0.4, off_duration=0.1):
        """Runs one beep pulse cycle for continuous alarm looping while breach persists."""
        if self.native_pwm is not None:
            try:
                self.native_pwm.value = 0.5
                time.sleep(on_duration)
                self.native_pwm.off()
                time.sleep(off_duration)
            except Exception as e:
                print(f"[Buzzer Error] Native PWM error: {e}")
        else:
            # Command execution matching user requirement
            cmd = [
                "python3", "-c",
                f"from gpiozero import PWMOutputDevice; from time import sleep; p = PWMOutputDevice({self.pin}, frequency={self.frequency}); p.value = 0.5; sleep({on_duration}); p.off()"
            ]
            try:
                subprocess.run(cmd, timeout=on_duration + 0.5, check=False)
                time.sleep(off_duration)
            except Exception as e:
                print(f"[Buzzer Error] Subprocess execution error: {e}")

    def turn_off(self):
        """Silences the buzzer immediately."""
        if self.native_pwm is not None:
            try:
                self.native_pwm.off()
            except Exception:
                pass

    def stop(self):
        """Clean shutdown for buzzer hardware."""
        if self.native_pwm is not None:
            try:
                self.native_pwm.off()
                self.native_pwm.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="UDP Geofence Breach Buzzer Listener")
    parser.add_argument("--ip", type=str, default=DEFAULT_UDP_IP, help="UDP bind IP address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=DEFAULT_UDP_PORT, help="UDP bind port (default 5006)")
    parser.add_argument("--pin", type=int, default=GPIO_PIN, help="GPIO pin for buzzer (default 12)")
    parser.add_argument("--freq", type=int, default=BUZZER_FREQ, help="PWM frequency in Hz (default 2300)")
    parser.add_argument("--subprocess", action="store_true", help="Force using python3 -c subprocess command execution")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass

    try:
        sock.bind((args.ip, args.port))
    except Exception as e:
        print(f"[ERROR] Failed to bind socket on {args.ip}:{args.port}: {e}")
        sys.exit(1)

    sock.settimeout(0.1)

    print("=" * 65)
    print("      GEOFENCE BREACH BUZZER LISTENER SERVICE")
    print("=" * 65)
    print(f"[UDP Listener] Listening on {args.ip}:{args.port}")
    print(f"[Hardware] GPIO Pin: {args.pin} | PWM Frequency: {args.freq} Hz")
    print(f"[Mode] {'Subprocess python3 -c' if args.subprocess or not HAS_GPIOZERO else 'Native gpiozero PWM'}")
    print("Press Ctrl+C to stop.\n")

    buzzer = BuzzerController(pin=args.pin, frequency=args.freq, force_subprocess=args.subprocess)

    is_breached = False
    last_packet_ts = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                last_packet_ts = time.time()
                msg = data.decode("utf-8", errors="ignore").strip().upper()

                if "BREACH" in msg or '"STATUS": "BREACH"' in msg or msg == "1" or "TRUE" in msg:
                    if not is_breached:
                        print(f"[{time.strftime('%H:%M:%S')}] >>> GEOFENCE BREACH DETECTED from {addr[0]}! Sounding buzzer loop...")
                    is_breached = True
                elif "SAFE" in msg or "INSIDE" in msg or msg == "0" or "FALSE" in msg:
                    if is_breached:
                        print(f"[{time.strftime('%H:%M:%S')}] >>> Geofence SAFE (Breach Cleared) from {addr[0]}. Silencing buzzer.")
                    is_breached = False
                    buzzer.turn_off()
            except socket.timeout:
                pass
            except Exception as e:
                print(f"[Socket Error] {e}")

            # Auto-timeout breach alert if no packets received for >3 seconds
            if is_breached and (time.time() - last_packet_ts > 3.0):
                print(f"[{time.strftime('%H:%M:%S')}] Telemetry timeout (>3s). Silencing buzzer.")
                is_breached = False
                buzzer.turn_off()

            # Continuous buzzer loop while breach persists
            if is_breached:
                buzzer.run_beep_cycle(on_duration=0.4, off_duration=0.1)
            else:
                buzzer.turn_off()
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopping Buzzer Listener Service...")
    finally:
        buzzer.stop()
        sock.close()
        print("Buzzer stopped cleanly.")


if __name__ == "__main__":
    main()
