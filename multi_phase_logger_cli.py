#!/usr/bin/env python3
"""
Interactive CLI Runner for Multi-Phase Drone Flight Logging (F0 - F6)
======================================================================
Subscribes directly to UDP telemetry broadcast from send_attitude_udp.py
on port 5005 and records telemetry data to CSV per phase (F0 - F6).
"""

import os
import sys
import time
import subprocess
from optical_flow.phase_logger import PhaseTestManager
from optical_flow.udp_phase_logger import UDPPhaseLogger


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def print_banner():
    print("==========================================================================")
    print("   MULTI-PHASE FLIGHT TEST LOGGER (UDP BROADCAST SOURCE @ PORT 5005)      ")
    print("==========================================================================")


def print_logging_status_box(udp_logger, current_phase="F0"):
    """Prints visual status box showing UDP telemetry reception & CSV logger activity."""
    stats = udp_logger.get_stats()
    print("\n--------------------------------------------------------------------------")
    print(f" [UDP MONITOR] Stream Source      : UDP Telemetry Broadcast ({stats['udp_source']})")
    print(f" [UDP MONITOR] Data Logging Status: {stats['status']}")
    print(f" [UDP MONITOR] Target CSV File   : {udp_logger.csv_filename}")
    print(f" [UDP MONITOR] UDP Packets Rx    : {stats['packet_count']:,} packets ({stats['bytes_received'] / 1024:.1f} KB)")
    print(f" [UDP MONITOR] CSV File Size & Rows: {stats['file_size_str']} | {stats['csv_rows']:,} data rows logged")

    last = stats["last_telemetry"]
    if last:
        ts = last.get("timestamp", 0.0)
        euler = last.get("rotation", {}).get("euler", {})
        roll = euler.get("roll", 0.0)
        pitch = euler.get("pitch", 0.0)
        yaw = euler.get("yaw", 0.0)
        pos = last.get("translation", {}).get("position", {})
        px = pos.get("x", 0.0) * 100.0
        py = pos.get("y", 0.0) * 100.0
        print(f" [UDP MONITOR] Latest UDP Packet : t={ts:.2f}s | Phase={current_phase} | Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}° | Pos=({px:.1f}, {py:.1f})cm")
    print("--------------------------------------------------------------------------")


def is_running(script_name):
    """Helper function to check if a python script is already running."""
    try:
        if os.name == 'nt':
            cmd = f'wmic process where "name=\'python.exe\' and commandline like \'%{script_name}%\'" get processid,commandline'
            res = subprocess.check_output(cmd, shell=True).decode('utf-8', errors='ignore')
            lines = [line.strip() for line in res.splitlines() if line.strip() and script_name in line]
            return len(lines) > 0, res
        else:
            cmd = f'pgrep -fa {script_name}'
            res = subprocess.check_output(cmd, shell=True).decode('utf-8', errors='ignore')
            return len(res.strip()) > 0, res
    except Exception:
        return False, ""


def run_interactive_logger():
    print_banner()

    # Create timestamped recording output filename
    os.makedirs("recordings", exist_ok=True)
    csv_filename = os.path.join("recordings", f"optical_flow_multiphase_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    print(f"\n[INFO] Initializing multi-phase flight telemetry logging session.")
    print(f"[INFO] Source: UDP Telemetry Broadcast from send_attitude_udp.py (Port 5005)")
    print(f"[INFO] Output CSV Destination: {csv_filename}")
    print("\nPhase Summary (F0 -> F6):")
    for code, info in PhaseTestManager.PHASES.items():
        dur_dist = info.get("indicative_duration") or info.get("indicative_distance") or info.get("indicative_turns") or info.get("indicative_events", "")
        print(f"  [{code}] {info['name']} - {info['description']} ({dur_dist})")

    input("\n>>> Press ENTER to START the UDP Multi-Phase Flight Logger session...")

    pm = PhaseTestManager("F0")

    udp_sender_running, _ = is_running("send_attitude_udp.py")
    flow_running, _ = is_running("optical_flow_stream.py")
    web_running, _ = is_running("web_server.py")

    procs = []

    # Ensure background services are active
    if not flow_running:
        print("[LAUNCH] Starting background Optical Flow Streamer engine...")
        try:
            procs.append(subprocess.Popen([sys.executable, "optical_flow_stream.py", "-stream"]))
        except Exception as e:
            print(f"[WARNING] Could not start optical_flow_stream.py: {e}")

    if not udp_sender_running:
        print("[LAUNCH] Starting background send_attitude_udp.py bridge (127.0.0.1:5005)...")
        try:
            procs.append(subprocess.Popen([sys.executable, "send_attitude_udp.py", "--ip", "127.0.0.1", "--port", "5005"]))
        except Exception as e:
            print(f"[WARNING] Could not start send_attitude_udp.py: {e}")
    else:
        print("[INFO] send_attitude_udp.py is ALREADY RUNNING (attaching to UDP broadcast @ port 5005).")

    if not web_running:
        print("[LAUNCH] Starting background web server: http://localhost:8000")
        try:
            procs.append(subprocess.Popen([sys.executable, "web_server.py"]))
        except Exception as e:
            print(f"[WARNING] Could not start web_server.py: {e}")

    # Start UDP subscriber logger
    udp_logger = UDPPhaseLogger(csv_filename=csv_filename, udp_host="127.0.0.1", udp_port=5005)
    udp_logger.start()

    print("\n[VERIFY] Waiting for UDP telemetry stream on port 5005...")
    time.sleep(2.0)

    try:
        for phase_code, phase_info in PhaseTestManager.PHASES.items():
            pm.set_phase(phase_code)
            clear_screen()
            print_banner()
            print(f"\n>>> ACTIVE PHASE: [{phase_code}] {phase_info['name']}")
            print(f"    Description : {phase_info['description']}")
            dur_dist = phase_info.get("indicative_duration") or phase_info.get("indicative_distance") or phase_info.get("indicative_turns") or phase_info.get("indicative_events", "")
            print(f"    Target/Dur  : {dur_dist}")

            # Print Real-Time Verbose UDP Logger Activity Monitor
            print_logging_status_box(udp_logger, current_phase=phase_code)

            if phase_code == "F0":
                print("\n[INSTRUCTION] Keep vehicle stationary & flat. Rotate device manually across 4 orientations to check magnetometer.")
            elif phase_code == "F1":
                print("\n[INSTRUCTION] Keep vehicle stationary to record complementary filter bias & sensor noise.")
            elif phase_code == "F2":
                print("\n[INSTRUCTION] Move vehicle straight along measured distance marks (8-10m).")
            elif phase_code == "F3":
                print("\n[INSTRUCTION] Turn vehicle around reference angle (e.g. 90 deg turn).")
            elif phase_code == "F4":
                print("\n[INSTRUCTION] Move vehicle across geofence boundary (inside -> outside -> inside).")
            elif phase_code == "F5":
                print("\n[INSTRUCTION] Traverse low-texture / bumpy surface section (2-3m).")
            elif phase_code == "F6":
                print("\n[INSTRUCTION] Stop vehicle. Verifying final log session state.")

            print("\nOptions:")
            print("  [ENTER]       : Confirm & Proceed to NEXT Phase")
            print("  [v / s]       : Check Live UDP Telemetry & CSV Logging Status")
            print("  [m + ENTER]   : Mark Physical Distance / Event (e.g., MARKA_2M / BREACH)")
            print("  [q + ENTER]   : Stop Flight Logger Session Early")

            while True:
                choice = input(f"\n[{phase_code}] Action [ENTER=Next Phase, v=Check Status, m=Mark Event, q=Stop]: ").strip().lower()
                if choice == "":
                    stats = udp_logger.get_stats()
                    print(f"[CONFIRM] Phase [{phase_code}] completed. Total CSV rows logged so far: {stats['csv_rows']:,} ({stats['file_size_str']}).")
                    time.sleep(0.5)
                    break
                elif choice in ["v", "s"]:
                    print_logging_status_box(udp_logger, current_phase=phase_code)
                elif choice == "m":
                    event_name = input("  Enter custom Event Marker name (e.g. MARKA_2M / TURN_90DEG / BREACH): ").strip()
                    if event_name:
                        pm.mark_event(event_name)
                        print(f"  -> Event Marker '{event_name}' recorded!")
                elif choice == "q":
                    print("\n[STOP] User requested early session termination.")
                    udp_logger.stop()
                    stats = udp_logger.get_stats()
                    print(f"\n==========================================================================")
                    print(f"Session closed successfully. Final CSV Log: {csv_filename}")
                    print(f"Total UDP Packets Rx: {stats['packet_count']:,} | CSV Rows Written: {stats['csv_rows']:,} | Size: {stats['file_size_str']}")
                    print(f"==========================================================================")
                    for p in procs:
                        if p.poll() is None:
                            p.terminate()
                    return

        clear_screen()
        udp_logger.stop()
        stats = udp_logger.get_stats()
        print("\n==========================================================================")
        print("   ALL PHASES (F0 - F6) COMPLETED SUCCESSFULLY!                           ")
        print("==========================================================================")
        print(f" Log File Saved To : {csv_filename}")
        print(f" UDP Source        : {stats['udp_source']}")
        print(f" UDP Packets Rx    : {stats['packet_count']:,} packets")
        print(f" Total Logged Rows : {stats['csv_rows']:,} data records written cleanly")
        print(f" Final File Size   : {stats['file_size_str']}")
        print("==========================================================================\n")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopping flight logger session...")
    finally:
        udp_logger.stop()
        for p in procs:
            if p.poll() is None:
                p.terminate()
        print("\n[CLEANUP] Multi-phase flight logger CLI exited cleanly.")


if __name__ == "__main__":
    run_interactive_logger()

