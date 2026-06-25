#!/usr/bin/env python3
"""
Read the optical flow stream and altitude from shared memory and inject it into ArduPilot
as fake GPS using the highly-supported GPS_RAW_INT MAVLink message.
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
    parser = argparse.ArgumentParser(description="Inject optical flow telemetry into ArduPilot as fake GPS_RAW_INT.")
    parser.add_argument(
        "--connection",
        type=str,
        default="udpin:127.0.0.1:14551",
        help="MAVLink connection string (default: udpin:127.0.0.1:14551)"
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
        # Establish connection (using udpin/udp matching the working reference scripts)
        master = mavutil.mavlink_connection(args.connection)
        
        # Wait for heartbeat to discover autopilot and populate system IDs and mappings
        print("Waiting for heartbeat from drone...")
        master.wait_heartbeat(timeout=15)
        print("Heartbeat received! Connected to drone.")
        print(f"Drone System ID: {master.target_system}")
        print(f"Drone Component ID: {master.target_component}")
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

                    # Project 2D relative displacement to Lat/Lon
                    lat, lon = meters_to_lat_lon(sample["x"], sample["y"], args.home_lat, args.home_lon)

                    # Altitude = Home Altitude + Height above ground
                    current_alt = args.home_alt + sample["alt"]

                    # Calculate Ground Speed (vel in cm/s) and Course Over Ground (cog in deg * 100)
                    speed_mps = math.hypot(sample["vx"], sample["vy"])
                    vel_cm_s = int(speed_mps * 100)
                    
                    if speed_mps > 0.05:
                        cog_deg = math.degrees(math.atan2(sample["vy"], sample["vx"]))
                        cog_cd = int((cog_deg % 360.0) * 100)
                    else:
                        cog_cd = 0

                    # Send GPS_RAW_INT message
                    try:
                        master.mav.gps_raw_int_send(
                            int(now * 1e6),             # Timestamp (micros since epoch or boot)
                            args.fix_type,              # Fix type
                            int(lat * 1e7),             # Latitude (degrees * 1e7)
                            int(lon * 1e7),             # Longitude (degrees * 1e7)
                            int(current_alt * 1000),    # Altitude in mm (AMSL)
                            100,                        # HDOP * 100
                            100,                        # VDOP * 100
                            vel_cm_s,                   # GPS ground speed in cm/s
                            cog_cd,                     # Course over ground in degrees * 100
                            args.satellites,            # Satellites visible
                            int(current_alt * 1000),    # Altitude (ellipsoid) in mm
                            100,                        # Position accuracy in mm
                            100,                        # Altitude accuracy in mm
                            100,                        # Speed accuracy in mm
                            100,                        # Heading accuracy in mm
                            0                           # Yaw in degrees * 100
                        )
                    except Exception as e:
                        print(f"Failed to send GPS_RAW_INT packet: {e}")

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
