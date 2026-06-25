#!/usr/bin/env python3
"""
Inject static fake GPS telemetry (GPS_INPUT) into ArduPilot for proof-of-concept testing.
Follows the user-provided reference script exactly.
"""

import argparse
import math
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Inject static fake GPS telemetry into ArduPilot.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udpout:127.0.0.1:14550",
        help="MAVLink connection string (default: udpout:127.0.0.1:14550)"
    )
    parser.add_argument(
        "--home-lat",
        type=float,
        default=-6.200000,
        help="Home Latitude (default: -6.200000)"
    )
    parser.add_argument(
        "--home-lon",
        type=float,
        default=106.816666,
        help="Home Longitude (default: 106.816666)"
    )
    parser.add_argument(
        "--home-alt",
        type=float,
        default=10.0,
        help="Home Altitude in meters (default: 10.0)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publishing frequency in Hz (default: 10.0)"
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
        # Establish connection matching the reference script
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to establish link
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Connected")
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
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
            time.sleep(1.0)
        except Exception as e:
            print(f"Failed to send mode change command: {e}")

    # Local coordinate projection matching reference exactly
    def xy_to_latlon(x_m, y_m):
        earth_radius = 6378137.0
        dlat = y_m / earth_radius
        dlon = x_m / (earth_radius * math.cos(math.radians(args.home_lat)))
        lat = args.home_lat + math.degrees(dlat)
        lon = args.home_lon + math.degrees(dlon)
        return lat, lon

    # Simulation state matching reference
    x = 0.0
    y = 0.0
    vx = 0.2
    vy = 0.0
    last = time.time()
    sleep_interval = 1.0 / args.rate

    print(f"Starting injection loop at {args.rate}Hz...")
    print(f"Home: Lat={args.home_lat:.6f}, Lon={args.home_lon:.6f}, Alt={args.home_alt:.1f}m")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            now = time.time()
            dt = now - last
            last = now

            # Simulate estimator movement
            x += vx * dt
            y += vy * dt

            # Project coordinates
            lat, lon = xy_to_latlon(x, y)

            # Send GPS_INPUT message matching reference exactly
            try:
                master.mav.gps_input_send(
                    int(time.time() * 1e6),               # time_usec
                    0,                                    # gps_id
                    (
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_HDOP |
                        mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VDOP
                    ),                                    # ignore_flags
                    0,                                    # time_week_ms
                    0,                                    # time_week
                    3,                                    # fix_type (3D Fix)
                    int(lat * 1e7),                       # lat (degrees * 1e7)
                    int(lon * 1e7),                       # lon (degrees * 1e7)
                    args.home_alt,                        # alt
                    1.0,                                  # hdop
                    1.0,                                  # vdop
                    vy,                                   # vn (velocity north = vy)
                    vx,                                   # ve (velocity east = vx)
                    0,                                    # vd
                    0.3,                                  # speed_accuracy
                    0.5,                                  # horiz_accuracy
                    0.5,                                  # vert_accuracy
                    15                                    # satellites_visible
                )
            except Exception as e:
                print(f"Failed to send GPS_INPUT packet: {e}")

            print(f"X={x:.2f} Y={y:.2f} LAT={lat:.7f} LON={lon:.7f}")
            
            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink GPS injector...")
    finally:
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
