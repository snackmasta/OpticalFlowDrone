#!/usr/bin/env python3
"""
Read the optical flow stream and altitude from shared memory and inject it into ArduPilot
as ExternalNav (VISION_POSITION_ESTIMATE & VISION_SPEED_ESTIMATE) for non-GPS Loiter and position hold.
Reference: mavlink_set_mode.py / mavlink_reboot.py
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
    parser = argparse.ArgumentParser(description="Inject optical flow telemetry into ArduPilot as ExternalNav.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udpin:127.0.0.1:14551",
        help="MAVLink connection string (default: udpin:127.0.0.1:14551)"
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=15.0,
        help="Publishing frequency in Hz (default: 15.0)"
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
        # Connect to the MAVLink source (using udpin/udp matching the working reference scripts)
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to establish link
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Connected")
    except Exception as e:
        print(f"Error establishing MAVLink connection: {e}")
        print("Make sure MAVProxy is running and outputting to the specified port.")
        sys.exit(1)

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
        while not stop_event:
            loop_start = time.perf_counter()
            now = time.time()

            # Send heartbeat every 1 second to maintain connection state
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

            sample = read_latest_sample(shm)
            if sample is not None:
                sample_ts = sample["timestamp"]
                age = now - sample_ts

                # Only publish if the sample is fresh and we haven't sent this exact sample before
                if age <= args.freshness and sample_ts != last_seen_ts:
                    last_seen_ts = sample_ts
                    time_usec = int(now * 1e6)

                    # Send VISION_POSITION_ESTIMATE message (position)
                    # NED Frame: x=North (forward), y=East (right), z=Down (negative altitude)
                    try:
                        master.mav.vision_position_estimate_send(
                            time_usec,                  # time_usec
                            float(sample["x"]),         # x (meters, North)
                            float(sample["y"]),         # y (meters, East)
                            float(-sample["alt"]),      # z (meters, Down)
                            0.0,                        # roll (rad)
                            0.0,                        # pitch (rad)
                            0.0,                        # yaw (rad)
                            [0]*21                      # covariance matrix
                        )
                        
                        # Send VISION_SPEED_ESTIMATE message (velocity)
                        master.mav.vision_speed_estimate_send(
                            time_usec,                  # time_usec
                            float(sample["vx"]),        # x velocity (m/s)
                            float(sample["vy"]),        # y velocity (m/s)
                            0.0,                        # z velocity (m/s)
                            [0]*9                       # covariance matrix
                        )
                    except Exception as e:
                        print(f"Failed to send ExternalNav packets: {e}")

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink ExternalNav injector...")
    finally:
        stop_event = True
        release_shared_memory(shm)
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
