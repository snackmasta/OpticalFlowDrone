#!/usr/bin/env python3
"""
Inject static or simulated moving ExternalNav telemetry (VISION_POSITION_ESTIMATE & VISION_SPEED_ESTIMATE)
into ArduPilot for proof-of-concept testing of non-GPS Loiter and position hold.
Reference: mavlink_set_mode.py / mavlink_reboot.py
"""

import argparse
import sys
import time
from pymavlink import mavutil


def main():
    parser = argparse.ArgumentParser(description="Inject static fake ExternalNav telemetry into ArduPilot.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udpin:127.0.0.1:14550",
        help="MAVLink connection string (default: udpin:127.0.0.1:14550)"
    )
    parser.add_argument(
        "--alt",
        type=float,
        default=1.5,
        help="Static Altitude / height above ground in meters (default: 1.5)"
    )
    parser.add_argument(
        "--vx",
        type=float,
        default=0.2,
        help="Simulated Velocity X in m/s (default: 0.2)"
    )
    parser.add_argument(
        "--vy",
        type=float,
        default=0.0,
        help="Simulated Velocity Y in m/s (default: 0.0)"
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
        # Establish connection (using udpin/udp matching the working reference scripts)
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to establish link
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Connected")
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
            time.sleep(1.0)
        except Exception as e:
            print(f"Failed to send mode change command: {e}")

    # Simulation state
    x = 0.0
    y = 0.0
    last = time.time()
    sleep_interval = 1.0 / args.rate
    last_heartbeat_time = 0.0
    last_print_time = 0.0
    packets_sent = 0

    print(f"Starting ExternalNav injection loop at {args.rate}Hz...")
    print(f"Injecting simulated motion: Vx={args.vx:.2f}m/s, Vy={args.vy:.2f}m/s, Alt={args.alt:.2f}m")
    print("Ensure VISO_TYPE=1 and EKF3 sources are configured for ExternalNav.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            now = time.time()
            dt = now - last
            last = now

            # Integrate simulated velocities to update position coordinates
            x += args.vx * dt
            y += args.vy * dt

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

            # Send VISION_POSITION_ESTIMATE message (position)
            # NED Frame: x=North (forward), y=East (right), z=Down (negative altitude)
            try:
                master.mav.vision_position_estimate_send(
                    int(now * 1e6),             # time_usec
                    float(x),                   # x (meters, North)
                    float(y),                   # y (meters, East)
                    float(-args.alt),           # z (meters, Down)
                    0.0,                        # roll (rad)
                    0.0,                        # pitch (rad)
                    0.0,                        # yaw (rad)
                    [0]*21                      # covariance matrix
                )
                
                # Send VISION_SPEED_ESTIMATE message (velocity)
                master.mav.vision_speed_estimate_send(
                    int(now * 1e6),             # time_usec
                    float(args.vx),             # x velocity (m/s)
                    float(args.vy),             # y velocity (m/s)
                    0.0,                        # z velocity (m/s)
                    [0]*9                       # covariance matrix
                )
                
                packets_sent += 1
            except Exception as e:
                print(f"Failed to send ExternalNav packets: {e}")

            # Print status update every 2 seconds
            if now - last_print_time >= 2.0:
                print(f"Status: Sent {packets_sent} ExternalNav packet pairs. X={x:.2f}, Y={y:.2f}.")
                last_print_time = now

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping ExternalNav injector...")
    finally:
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
