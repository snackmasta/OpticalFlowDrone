#!/usr/bin/env python3
"""
Read the optical flow stream and altitude from shared memory and inject it into ArduPilot
as fake GPS using the exact message structure and mapping from the user's reference script.
"""

import argparse
import math
import struct
import sys
import time
from multiprocessing import shared_memory
from multiprocessing import resource_tracker
from pymavlink import mavutil

# Shared memory configuration for optical flow stream
SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<6d"  # timestamp, x, y, vx, vy, alt
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
DEFAULT_FRESHNESS_THRESHOLD = 0.75

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


def main():
    global stop_event
    parser = argparse.ArgumentParser(description="Inject optical flow telemetry into ArduPilot as fake GPS.")
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
        "--freshness",
        type=float,
        default=DEFAULT_FRESHNESS_THRESHOLD,
        help="Maximum age in seconds for a sample to be considered fresh (default: 0.75)"
    )
    args = parser.parse_args()

    print(f"Connecting to MAVLink on {args.connection}...")
    try:
        # Establish connection matching reference
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to establish link
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Connected")
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
        sys.exit(1)

    print(f"Waiting for shared memory segment '{SHM_NAME}'...")
    shm = open_shared_memory()
    if shm is None:
        print("Shared memory opening cancelled.")
        sys.exit(0)
    print("Shared memory segment opened successfully.")

    # Local coordinate projection matching reference exactly
    def xy_to_latlon(x_m, y_m):
        earth_radius = 6378137.0
        dlat = y_m / earth_radius
        dlon = x_m / (earth_radius * math.cos(math.radians(args.home_lat)))
        lat = args.home_lat + math.degrees(dlat)
        lon = args.home_lon + math.degrees(dlon)
        return lat, lon

    sleep_interval = 1.0 / args.rate
    last_seen_ts = None

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

                    # Project coordinates matching reference exactly
                    lat, lon = xy_to_latlon(sample["x"], sample["y"])

                    # Altitude = Home Altitude + Height above ground
                    current_alt = args.home_alt + sample["alt"]

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
                            float(current_alt),                   # alt
                            1.0,                                  # hdop
                            1.0,                                  # vdop
                            sample["vy"],                         # vn (velocity north = vy)
                            sample["vx"],                         # ve (velocity east = vx)
                            0,                                    # vd
                            0.3,                                  # speed_accuracy
                            0.5,                                  # horiz_accuracy
                            0.5,                                  # vert_accuracy
                            15                                    # satellites_visible
                        )
                    except Exception as e:
                        print(f"Failed to send GPS_INPUT packet: {e}")

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink GPS injector...")
    finally:
        stop_event = True
        release_shared_memory(shm)
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
