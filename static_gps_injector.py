#!/usr/bin/env python3
"""
Inject static fake GPS telemetry (GPS_INPUT) into ArduPilot for proof-of-concept testing.
Allows verification of GPS1_TYPE and EKF3 configurations without running the optical flow tracking pipeline.
Reference: mavlink_set_mode.py / mavlink_reboot.py
"""

import argparse
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Inject static fake GPS telemetry into ArduPilot.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udpin:127.0.0.1:14550",
        help="MAVLink connection string (default: udpin:127.0.0.1:14550)"
    )
    parser.add_argument(
        "--lat",
        type=float,
        default=47.3769,
        help="Static Latitude to inject (default: 47.3769)"
    )
    parser.add_argument(
        "--lon",
        type=float,
        default=8.5417,
        help="Static Longitude to inject (default: 8.5417)"
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=500.0,
        help="Static Altitude in meters above mean sea level (default: 500.0)"
    )
    parser.add_argument(
        "--vn",
        type=float,
        default=0.0,
        help="Static Velocity North in m/s (default: 0.0)"
    )
    parser.add_argument(
        "--ve",
        type=float,
        default=0.0,
        help="Static Velocity East in m/s (default: 0.0)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publishing frequency in Hz (default: 10.0)"
    )
    parser.add_argument(
        "--gps-id",
        type=int,
        default=0,
        help="GPS ID reported to ArduPilot (default: 0)"
    )
    parser.add_argument(
        "--satellites",
        type=int,
        default=10,
        help="Number of visible satellites to report (default: 10)"
    )
    parser.add_argument(
        "--fix-type",
        type=int,
        default=3,
        help="GPS Fix Type (3 = 3D Fix, default: 3)"
    )
    parser.add_argument(
        "--set-mode",
        type=str,
        default=None,
        help="Attempt to change flight mode to the specified mode (e.g., LOITER, STABILIZE)"
    )
    args = parser.parse_args()

    print(f"Connecting to MAVLink on {args.connection}...")
    try:
        # Establish connection (using udpin/udp matching the working reference scripts)
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to discover autopilot and populate system IDs and mode mappings
        print("Waiting for heartbeat from drone...")
        master.wait_heartbeat(timeout=15)
        print("Heartbeat received! Connected to drone.")
        print(f"Drone System ID: {master.target_system}")
        print(f"Drone Component ID: {master.target_component}")
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
        print("Make sure MAVProxy is running and outputting to the specified port.")
        sys.exit(1)

    # Set flight mode if requested
    if args.set_mode:
        try:
            mode = args.set_mode.upper()
            mode_map = master.mode_mapping()
            
            if mode_map and mode in mode_map:
                mode_id = mode_map[mode]
                print(f"Sending request to change flight mode to {mode} (ID: {mode_id})...")
                master.set_mode(mode_id)
            else:
                try:
                    mode_id = int(mode)
                    print(f"Sending request to change flight mode to ID {mode_id}...")
                    master.set_mode(mode_id)
                except ValueError:
                    print(f"Unknown flight mode: {args.set_mode}. Available modes: {list(mode_map.keys()) if mode_map else 'None'}")
            
            # Wait a brief moment to let the mode change process
            time.sleep(1.0)
        except Exception as e:
            print(f"Failed to send mode change command: {e}")

    sleep_interval = 1.0 / args.rate
    last_heartbeat_time = 0.0
    last_print_time = 0.0
    packets_sent = 0

    print(f"Starting injection loop at {args.rate}Hz...")
    print(f"Injecting: Lat={args.lat:.6f}, Lon={args.lon:.6f}, Alt={args.alt:.1f}m, Vn={args.vn:.2f}m/s, Ve={args.ve:.2f}m/s")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            now = time.time()

            # Send heartbeat every 1 second
            if now - last_heartbeat_time >= 1.0:
                try:
                    master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0
                    )
                except Exception:
                    pass
                last_heartbeat_time = now

            # Send GPS_INPUT message
            try:
                master.mav.gps_input_send(
                    0,                  # Seconds since epoch or boot (0 for system time)
                    args.gps_id,        # GPS ID
                    0,                  # Ignore flags (use all parameters)
                    0,                  # Time since start of GPS week
                    0,                  # GPS week
                    args.fix_type,      # Fix type
                    int(args.lat * 1e7),# Latitude (degrees * 1e7)
                    int(args.lon * 1e7),# Longitude (degrees * 1e7)
                    float(args.alt),    # Altitude
                    1.0,                # HDOP
                    1.0,                # VDOP
                    float(args.vn),     # Velocity North (vn, m/s)
                    float(args.ve),     # Velocity East (ve, m/s)
                    0.0,                # Velocity Down (vd, m/s)
                    0.1,                # Speed accuracy
                    0.1,                # Horizontal accuracy
                    0.1,                # Vertical accuracy
                    args.satellites     # Satellites visible
                )
                packets_sent += 1
            except Exception as e:
                print(f"Failed to send GPS_INPUT packet: {e}")

            # Print status update every 2 seconds
            if now - last_print_time >= 2.0:
                print(f"Status: Sent {packets_sent} packets so far. Active 3D Fix with {args.satellites} satellites.")
                last_print_time = now

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping static MAVLink GPS injector...")
    finally:
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
