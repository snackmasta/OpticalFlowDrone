#!/usr/bin/env python3
"""
Inject static fake GPS telemetry into ArduPilot via MAVProxy's GPSInput module.
Sends JSON-encoded UDP packets to MAVProxy (default port 25100), which automatically
handles formatting and injecting the GPS_INPUT MAVLink messages to the flight controller.
"""

import argparse
import json
import socket
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="Inject static fake GPS telemetry into MAVProxy GPSInput module.")
    parser.add_argument(
        "--ip",
        type=str,
        default="127.0.0.1",
        help="IP address of MAVProxy GPSInput listener (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=25100,
        help="UDP port of MAVProxy GPSInput listener (default: 25100)"
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
    args = parser.parse_args()

    # Create raw UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        dest_addr = (args.ip, args.port)
        print(f"UDP socket initialized. Sending to {args.ip}:{args.port}")
    except Exception as e:
        print(f"Error creating UDP socket: {e}")
        sys.exit(1)

    sleep_interval = 1.0 / args.rate
    last_print_time = 0.0
    packets_sent = 0

    print(f"Starting JSON GPS injection loop at {args.rate}Hz...")
    print(f"Injecting: Lat={args.lat:.6f}, Lon={args.lon:.6f}, Alt={args.alt:.1f}m, Vn={args.vn:.2f}m/s, Ve={args.ve:.2f}m/s")
    print("Ensure MAVProxy is running with 'module load GPSInput' enabled.")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            loop_start = time.perf_counter()
            now = time.time()

            # Construct the JSON payload exactly as expected by MAVProxy's GPSInput module
            data = {
                'time_usec': int(now * 1e6),          # (uint64_t) Timestamp (micros since boot or Unix epoch)
                'gps_id': args.gps_id,                # (uint8_t) ID of the GPS for multiple GPS inputs
                'ignore_flags': 8,                    # (uint16_t) Ignore vertical velocity (vd), use all other fields
                'time_week_ms': 0,                    # (uint32_t) GPS time (milliseconds from start of GPS week)
                'time_week': 0,                       # (uint16_t) GPS week number
                'fix_type': args.fix_type,            # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix
                'lat': int(args.lat * 1e7),           # (int32_t) Latitude (WGS84), in degrees * 1E7
                'lon': int(args.lon * 1e7),           # (int32_t) Longitude (WGS84), in degrees * 1E7
                'alt': float(args.alt),               # (float) Altitude (AMSL, not WGS84) in meters
                'hdop': 1.0,                          # (float) GPS HDOP horizontal dilution
                'vdop': 1.0,                          # (float) GPS VDOP vertical dilution
                'vn': float(args.vn),                 # (float) GPS velocity in m/s in NORTH direction
                've': float(args.ve),                 # (float) GPS velocity in m/s in EAST direction
                'vd': 0.0,                            # (float) GPS velocity in m/s in DOWN direction (ignored by flag 8)
                'speed_accuracy': 0.1,                # (float) GPS speed accuracy in m/s
                'horiz_accuracy': 0.1,                # (float) GPS horizontal accuracy in m
                'vert_accuracy': 0.1,                 # (float) GPS vertical accuracy in m
                'satellites_visible': args.satellites # (uint8_t) Number of satellites visible
            }

            # Encode and send
            try:
                payload = json.dumps(data)
                sock.sendto(payload.encode(), dest_addr)
                packets_sent += 1
            except Exception as e:
                print(f"Failed to send packet: {e}")

            # Print status update every 2 seconds
            if now - last_print_time >= 2.0:
                print(f"Status: Sent {packets_sent} JSON packets so far. Fix={args.fix_type}, Sats={args.satellites}.")
                last_print_time = now

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVProxy JSON GPS injector...")
    finally:
        sock.close()
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
