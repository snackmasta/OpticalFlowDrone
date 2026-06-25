#!/usr/bin/env python3
"""
Read the optical flow stream from shared memory and publish to ArduPilot via MAVLink.
Supports both standard OPTICAL_FLOW and VISION_POSITION_ESTIMATE messages.
"""

import argparse
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
                now = time.time()
                if msg.get_type() == 'DISTANCE_SENSOR':
                    # current_distance is in cm in MAVLink, convert to meters
                    dist_m = msg.current_distance / 100.0
                    with ground_distance_lock:
                        latest_ground_distance = dist_m
                elif msg.get_type() == 'RANGEFINDER':
                    dist_m = msg.distance
                    with ground_distance_lock:
                        latest_ground_distance = dist_m
        except Exception as e:
            # Handle potential MAVLink disconnection or read errors silently
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description="Publish optical flow shared memory telemetry to MAVLink.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udp:127.0.0.1:14551",
        help="MAVLink connection string (default: udp:127.0.0.1:14551)"
    )
    parser.add_argument(
        "--msg-type",
        type=str,
        choices=["optical_flow", "vision_position", "both"],
        default="both",
        help="Type of MAVLink message to send (default: both)"
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
    parser.add_argument(
        "--sensor-id",
        type=int,
        default=0,
        help="Sensor ID to report in MAVLink messages (default: 0)"
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

    try:
        while not stop_event.is_set():
            loop_start = time.perf_counter()

            sample = read_latest_sample(shm)
            if sample is not None:
                sample_ts = sample["timestamp"]
                age = time.time() - sample_ts

                # Only publish if the sample is fresh and we haven't sent this exact sample before
                if age <= args.freshness and sample_ts != last_seen_ts:
                    last_seen_ts = sample_ts
                    time_usec = int(sample_ts * 1e6)

                    with ground_distance_lock:
                        g_dist = latest_ground_distance

                    # Send OPTICAL_FLOW message
                    if args.msg_type in ("optical_flow", "both"):
                        # flow_comp_m_sec_x and flow_comp_m_sec_y are in m/s (compensated)
                        # flow_x and flow_y are in pixels (set to 0 since we send physical velocity)
                        master.mav.optical_flow_send(
                            time_usec=time_usec,
                            sensor_id=args.sensor_id,
                            flow_x=0,
                            flow_y=0,
                            flow_comp_m_x=sample["vx"],
                            flow_comp_m_y=sample["vy"],
                            quality=255,  # Max quality/confidence
                            ground_distance=g_dist
                        )

                    # Send VISION_POSITION_ESTIMATE message
                    if args.msg_type in ("vision_position", "both"):
                        # x, y are in meters (NED frame: x=North/forward, y=East/right)
                        # z is set to negative ground distance (altitude) to fit NED frame
                        master.mav.vision_position_estimate_send(
                            usec=time_usec,
                            x=sample["x"],
                            y=sample["y"],
                            z=-g_dist,
                            roll=0.0,
                            pitch=0.0,
                            yaw=0.0,
                            covariance=[0]*21
                        )

            # Control loop rate precisely
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.001, sleep_interval - elapsed))

    except KeyboardInterrupt:
        print("\nStopping MAVLink publisher...")
    finally:
        stop_event.set()
        listener_thread.join(timeout=1.0)
        release_shared_memory(shm)
        print("Cleanup completed.")


if __name__ == "__main__":
    main()
