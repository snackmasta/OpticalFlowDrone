#!/usr/bin/env python3
"""
Read the optical flow stream and altitude from shared memory and inject it into ArduPilot
as fake GPS via MAVProxy's GPSInput module.
Sends JSON-encoded UDP packets to MAVProxy (default port 25100).
"""

import argparse
import json
import math
import socket
import struct
import sys
import time
from multiprocessing import shared_memory
from multiprocessing import resource_tracker

# Shared memory configuration for optical flow stream
SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<6d"  # timestamp, x, y, vx, vy, alt
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
DEFAULT_FRESHNESS_THRESHOLD = 0.75

# Earth radius in meters for flat-earth coordinate projection
EARTH_RADIUS = 6378137.0

stop_event = False


def open_shared_memory(wait_interval=0.5):
    """Tries to open the shared memory segment, retrying until it becomes available."""
    while not stop_event:
        try:
            return shared_memory.SharedMemory(name=SHM_NAME)
        except FileNotFoundError:
            time.sleep(wait_interval)
    return None


def release_shared_memory(shm):
    """Releases the shared memory segment, ensuring it is properly closed."""
    if shm is None:
        return
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass
    try:
        shm.close()
    except Exception:
        pass


def read_latest_sample(shm):
    """Returns the latest sample as a dict, or None if no valid sample is available."""
    try:
        magic, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
        if magic != SHM_MAGIC or sample_count == 0:
            return None

        latest_index = (write_index - 1) % MAX_SAMPLES
        record_offset = SHM_HEADER_SIZE + (latest_index * SHM_RECORD_SIZE)
        timestamp, x, y, vx, vy, alt = struct.unpack_from(SHM_RECORD_FORMAT, shm.buf, record_offset)
        return {
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "alt": alt,
        }
    except Exception:
        return None


def meters_to_lat_lon(x_m, y_m, home_lat, home_lon):
    """Converts local meters (x=North, y=East) to latitude and longitude degrees."""
    lat_rad = math.radians(home_lat)
    lat_offset = (x_m / EARTH_RADIUS) * (180.0 / math.pi)
    lon_offset = (y_m / (EARTH_RADIUS * math.cos(lat_rad))) * (180.0 / math.pi)
    return home_lat + lat_offset, home_lon + lon_offset


def main():
    global stop_event
    parser = argparse.ArgumentParser(description="Inject optical flow telemetry into MAVProxy GPSInput module.")
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
        "--home-lat",
        type=float,
        default=47.3769,
        help="Home Latitude for coordinates projection (default: 47.3769)"
    )
    parser.add_argument(
        "--home-lon",
        type=float,
        default=8.5417,
        help="Home Longitude for coordinates projection (default: 8.5417)"
    )
    parser.add_argument(
        "--home-alt",
        type=float,
        default=500.0,
        help="Home Altitude above mean sea level in meters (default: 500.0)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publishing frequency in Hz (default: 10.0)"
    )
    parser.add_argument(
        "--freshness",
        type=float,
        default=DEFAULT_FRESHNESS_THRESHOLD,
        help="Maximum age in seconds for a sample to be considered fresh (default: 0.75)"
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

    print(f"Waiting for shared memory segment '{SHM_NAME}'...")
    shm = open_shared_memory()
    if shm is None:
        print("Shared memory opening cancelled.")
        sys.exit(0)
    print("Shared memory segment opened successfully.")

    sleep_interval = 1.0 / args.rate
    last_seen_ts = None
    packets_sent = 0
    last_print_time = 0.0

    try:
        while not stop_event:
            loop_start = time.perf_counter()
            now = time.time()

            sample = read_latest_sample(shm)
            if sample is not None:
                sample_ts = sample["timestamp"]
                age = now - sample_ts

                # Only publish if the sample is fresh and we haven't sent this exact sample before
                if age <= args.freshness and sample_ts != last_seen_ts:
                    last_seen_ts = sample_ts

                    # Project 2D relative displacement to Lat/Lon
                    lat, lon = meters_to_lat_lon(sample["x"], sample["y"], args.home_lat, args.home_lon)

                    # Altitude = Home Altitude + Height above ground
                    current_alt = args.home_alt + sample["alt"]

                    # Construct the JSON payload exactly as expected by MAVProxy's GPSInput module
                    data = {
                        'time_usec': int(now * 1e6),          # Timestamp (micros since epoch or boot)
                        'gps_id': args.gps_id,                # ID of the GPS
                        'ignore_flags': 8,                    # Ignore vertical velocity (vd)
                        'time_week_ms': 0,                    # GPS time
                        'time_week': 0,                       # GPS week number
                        'fix_type': args.fix_type,            # GPS Fix Type
                        'lat': int(lat * 1e7),                # Latitude (degrees * 1e7)
                        'lon': int(lon * 1e7),                # Longitude (degrees * 1e7)
                        'alt': float(current_alt),            # Altitude in meters
                        'hdop': 1.0,                          # GPS HDOP
                        'vdop': 1.0,                          # GPS VDOP
                        'vn': float(sample["vx"]),             # Velocity North
                        've': float(sample["vy"]),             # Velocity East
                        'vd': 0.0,                            # Velocity Down (ignored by flag 8)
                        'speed_accuracy': 0.1,                # Speed accuracy
                        'horiz_accuracy': 0.1,                # Horizontal accuracy
                        'vert_accuracy': 0.1,                 # Vertical accuracy
                        'satellites_visible': args.satellites # Number of satellites visible
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
                print(f"Status: Sent {packets_sent} JSON packets so far. Sats={args.satellites}.")
                last_print_time = now

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink GPS injector...")
    finally:
        stop_event = True
        release_shared_memory(shm)
        sock.close()
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
