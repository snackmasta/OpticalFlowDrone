#!/usr/bin/env python3
"""
Read the optical flow stream from shared memory and inject it into ArduPilot as fake GPS.
Converts local metric coordinates (X/Y) to GPS coordinates relative to a home position.
"""

import argparse
import math
import struct
import sys
import time
import threading
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
from pymavlink import mavutil

# Shared memory configuration for optical flow stream
SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<5d"  # timestamp, x, y, vx, vy
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
DEFAULT_FRESHNESS_THRESHOLD = 0.75

# Earth radius in meters for flat-earth coordinate projection
EARTH_RADIUS = 6378137.0

# Global state to track latest altitude from the flight controller
latest_ground_distance = 1.5
ground_distance_lock = threading.Lock()
stop_event = threading.Event()


def open_shared_memory(wait_interval=0.5):
    """Tries to open the shared memory segment, retrying until it becomes available."""
    while not stop_event.is_set():
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
        timestamp, x, y, vx, vy = struct.unpack_from(SHM_RECORD_FORMAT, shm.buf, record_offset)
        return {
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
        }
    except Exception:
        return None


def mavlink_listener(master):
    """Listens to incoming MAVLink messages to update ground distance (altitude)."""
    global latest_ground_distance
    print("Started MAVLink listener thread for distance sensor telemetry...")
    while not stop_event.is_set():
        try:
            # Non-blocking receive with a timeout
            msg = master.recv_match(type=['DISTANCE_SENSOR', 'RANGEFINDER'], blocking=True, timeout=0.1)
            if msg is not None:
                if msg.get_type() == 'DISTANCE_SENSOR':
                    # current_distance is in cm in MAVLink, convert to meters
                    dist_m = msg.current_distance / 100.0
                    with ground_distance_lock:
                        latest_ground_distance = dist_m
                elif msg.get_type() == 'RANGEFINDER':
                    dist_m = msg.distance
                    with ground_distance_lock:
                        latest_ground_distance = dist_m
        except Exception:
            time.sleep(0.1)


def meters_to_lat_lon(x_m, y_m, home_lat, home_lon):
    """Converts local meters (x=North, y=East) to latitude and longitude degrees."""
    lat_rad = math.radians(home_lat)
    lat_offset = (x_m / EARTH_RADIUS) * (180.0 / math.pi)
    lon_offset = (y_m / (EARTH_RADIUS * math.cos(lat_rad))) * (180.0 / math.pi)
    return home_lat + lat_offset, home_lon + lon_offset


def main():
    parser = argparse.ArgumentParser(description="Inject optical flow telemetry into ArduPilot as fake GPS.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udp:127.0.0.1:14551",
        help="MAVLink connection string (default: udp:127.0.0.1:14551)"
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

    print(f"Connecting to MAVLink on {args.connection}...")
    try:
        master = mavutil.mavlink_connection(args.connection)
        master.wait_heartbeat(timeout=10)
        print("MAVLink connection established.")
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
        sys.exit(1)

    # Start the background MAVLink listener thread to track ground distance
    listener_thread = threading.Thread(target=mavlink_listener, args=(master,), daemon=True)
    listener_thread.start()

    print(f"Waiting for shared memory segment '{SHM_NAME}'...")
    shm = open_shared_memory()
    if shm is None:
        print("Shared memory opening cancelled.")
        sys.exit(0)
    print("Shared memory segment opened successfully.")

    sleep_interval = 1.0 / args.rate
    last_seen_ts = None
    last_heartbeat_time = 0.0

    try:
        while not stop_event.is_set():
            loop_start = time.perf_counter()
            now = time.time()

            # Send heartbeat every 1 second to maintain connection state
            if now - last_heartbeat_time >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_heartbeat_time = now

            sample = read_latest_sample(shm)
            if sample is not None:
                sample_ts = sample["timestamp"]
                age = now - sample_ts

                # Only publish if the sample is fresh and we haven't sent this exact sample before
                if age <= args.freshness and sample_ts != last_seen_ts:
                    last_seen_ts = sample_ts

                    # Project 2D relative displacement to Lat/Lon
                    lat, lon = meters_to_lat_lon(sample["x"], sample["y"], args.home_lat, args.home_lon)

                    with ground_distance_lock:
                        g_dist = latest_ground_distance

                    # Altitude = Home Altitude + Height above ground
                    current_alt = args.home_alt + g_dist

                    # Send GPS_INPUT message
                    # Lat and Lon are converted to degrees * 1E7 (integer)
                    master.mav.gps_input_send(
                        0,                  # Timestamp (0 for system time)
                        args.gps_id,        # GPS ID
                        0,                  # Ignore flags (use all parameters)
                        0,                  # Time since start of GPS week
                        0,                  # GPS week
                        args.fix_type,      # Fix type
                        int(lat * 1e7),     # Latitude (degrees * 1e7)
                        int(lon * 1e7),     # Longitude (degrees * 1e7)
                        float(current_alt), # Altitude
                        1.0,                # HDOP
                        1.0,                # VDOP
                        float(sample["vx"]),# Velocity North (vn, m/s)
                        float(sample["vy"]),# Velocity East (ve, m/s)
                        0.0,                # Velocity Down (vd, m/s)
                        0.1,                # Speed accuracy
                        0.1,                # Horizontal accuracy
                        0.1,                # Vertical accuracy
                        args.satellites     # Satellites visible
                    )

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink GPS injector...")
    finally:
        stop_event.set()
        listener_thread.join(timeout=1.0)
        release_shared_memory(shm)
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
