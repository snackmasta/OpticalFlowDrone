#!/usr/bin/env python3
"""
MAVLink Flight Mode Changer Script
Connects to the drone, waits for a heartbeat, and changes the flight mode.
Reference: mavlink_reboot.py
"""

import argparse
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Barebone MAVLink Flight Mode Changer.")
    parser.add_argument(
        "--port",
        type=int,
        default=14550,
        help="UDP port to listen on (default: 14550)"
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="ALT_HOLD",
        help="Flight mode to set (e.g., ALT_HOLD, LOITER, STABILIZE; default: ALT_HOLD)"
    )
    args = parser.parse_args()

    print("MAVLink Flight Mode Changer Starting...")
    print(f"Listening on UDP port: {args.port} (udpin:127.0.0.1:{args.port})")

    try:
        # Connect to the MAVLink source (using udpin to act as server, same as mavlink_reboot.py)
        master = mavutil.mavlink_connection(f"udpin:127.0.0.1:{args.port}")

        # Wait for first heartbeat
        print("Waiting for heartbeat from drone...")
        master.wait_heartbeat(timeout=15)
        print("Heartbeat received! Connected to drone.")
        print(f"Drone System ID: {master.target_system}")
        print(f"Drone Component ID: {master.target_component}")

        # Get the target flight mode
        mode = args.mode.upper()
        mode_map = master.mode_mapping()
        
        if mode_map and mode in mode_map:
            mode_id = mode_map[mode]
        else:
            try:
                mode_id = int(mode)
            except ValueError:
                print(f"Error: Unknown flight mode '{args.mode}'.")
                print(f"Available modes: {list(mode_map.keys()) if mode_map else 'None'}")
                sys.exit(1)

        print(f"Sending command to change flight mode to {mode} (ID: {mode_id})...")
        
        # Send set mode command
        master.set_mode(mode_id)
        
        # Wait a brief moment to let the command transit
        time.sleep(1.0)
        print("Command sent successfully.")

    except Exception as e:
        print(f"Error: {e}")
        print(f"Make sure MAVProxy is running and forwarding to UDP port {args.port}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nScript interrupted by user")
        sys.exit(0)


if __name__ == "__main__":
    main()
