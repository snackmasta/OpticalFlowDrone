#!/usr/bin/env python3
"""
Interactive CLI Runner for Multi-Phase Drone Flight Logging (F0 - F6)
======================================================================
Provides step-by-step user confirmations, event marking, and automatic
multi-phase telemetry logging coordination for optical_flow_stream.py.
"""

import os
import sys
import time
import subprocess
from optical_flow.phase_logger import PhaseTestManager


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def print_banner():
    print("==========================================================================")
    print("   MULTI-PHASE FLIGHT TEST LOGGER RUNNER (F0 -> F6) WITH USER CONFIRMATION ")
    print("==========================================================================")


def run_interactive_logger():
    print_banner()

    # Create timestamped recording output filename
    os.makedirs("recordings", exist_ok=True)
    csv_filename = os.path.join("recordings", f"optical_flow_multiphase_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    print(f"\n[INFO] Initializing multi-phase flight logging session.")
    print(f"[INFO] Log file destination: {csv_filename}")
    print("\nPhase Summary:")
    for code, info in PhaseTestManager.PHASES.items():
        dur_dist = info.get("indicative_duration") or info.get("indicative_distance") or info.get("indicative_turns") or info.get("indicative_events", "")
        print(f"  [{code}] {info['name']} - {info['description']} ({dur_dist})")

    input("\n>>> Press ENTER to START the Multi-Phase Flight Logger session...")

    pm = PhaseTestManager("F0")

    # Helper function to check if a python script is already running
    def is_running(script_name):
        try:
            if os.name == 'nt':
                # On Windows, check using wmic or tasklist
                cmd = f'wmic process where "name=\'python.exe\' and commandline like \'%{script_name}%\'" get processid'
                res = subprocess.check_output(cmd, shell=True).decode('utf-8', errors='ignore')
                lines = [line.strip() for line in res.splitlines() if line.strip() and line.strip().isdigit()]
                return len(lines) > 0
            else:
                # On Linux (Raspberry Pi), check using pgrep
                cmd = f'pgrep -f {script_name}'
                res = subprocess.check_output(cmd, shell=True).decode('utf-8', errors='ignore')
                return len(res.strip()) > 0
        except Exception:
            return False

    web_running = is_running("web_server.py")
    flow_running = is_running("optical_flow_stream.py")

    web_proc = None
    proc = None

    # Handle Web Server
    if web_running:
        print("[INFO] web_server.py is already RUNNING (attached to existing web server).")
    else:
        web_cmd = [sys.executable, "web_server.py"]
        print("[LAUNCH] Starting background web dashboard server: http://localhost:8000")
        try:
            web_proc = subprocess.Popen(web_cmd)
        except Exception as e:
            print(f"[WARNING] Could not start web_server: {e}")

    # Handle Optical Flow Streamer
    if flow_running:
        print("[INFO] optical_flow_stream.py is already RUNNING (attached to active services session).")
    else:
        cmd = [sys.executable, "optical_flow_stream.py", "-stream", "-csv", csv_filename]
        print(f"[LAUNCH] Starting background process: {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(cmd)
        except Exception as e:
            print(f"[ERROR] Failed to start optical_flow_stream process: {e}")
            if web_proc:
                web_proc.terminate()
            return

    time.sleep(1.0)

    try:
        for phase_code, phase_info in PhaseTestManager.PHASES.items():
            pm.set_phase(phase_code)
            clear_screen()
            print_banner()
            print(f"\n>>> ACTIVE PHASE: [{phase_code}] {phase_info['name']}")
            print(f"    Description : {phase_info['description']}")
            dur_dist = phase_info.get("indicative_duration") or phase_info.get("indicative_distance") or phase_info.get("indicative_turns") or phase_info.get("indicative_events", "")
            print(f"    Target/Dur  : {dur_dist}")
            print(f"    CSV Log File: {csv_filename}")
            print("--------------------------------------------------------------------------")

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
            print("  [m + ENTER]   : Mark Physical Distance / Event (e.g., Marka 2m / Breach Event)")
            print("  [q + ENTER]   : Stop Flight Logger Session Early")

            while True:
                choice = input(f"\n[{phase_code}] Action [ENTER=Next Phase, m=Mark Event, q=Stop]: ").strip().lower()
                if choice == "":
                    # Proceed to next phase
                    print(f"[CONFIRM] Phase [{phase_code}] completed.")
                    break
                elif choice == "m":
                    event_name = input("  Enter custom Event Marker name (e.g. MARKA_2M / TURN_90DEG / BREACH): ").strip()
                    if event_name:
                        pm.mark_event(event_name)
                        print(f"  -> Event Marker '{event_name}' recorded!")
                elif choice == "q":
                    print("\n[STOP] User requested early session termination.")
                    if proc and proc.poll() is None:
                        proc.terminate()
                        proc.wait()
                    print(f"Session closed successfully. Log saved to {csv_filename}")
                    return

        print("\n==========================================================================")
        print("   ALL PHASES (F0 - F6) COMPLETED SUCCESSFULLY!                           ")
        print("==========================================================================")
        print(f"Log file saved to: {csv_filename}")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Stopping flight logger session...")
    finally:
        if proc and proc.poll() is None:
            proc.terminate()
            proc.wait()
        if web_proc and web_proc.poll() is None:
            web_proc.terminate()
            web_proc.wait()
        print("\n[CLEANUP] Multi-phase flight logger CLI exited cleanly.")


if __name__ == "__main__":
    run_interactive_logger()
