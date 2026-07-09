#!/usr/bin/env python3
"""
Battery Monitor and Telemetry Streamer
Reads battery voltage, current, capacity, and mAh consumed from MAVLink
and writes the data to a shared memory segment for system-wide dashboard access.
Supports fallback simulation if MAVLink is not active.
"""

import argparse
import math
import struct
import sys
import time
import threading
from multiprocessing import shared_memory

try:
    from pymavlink import mavutil
except ImportError:
    print("Warning: pymavlink is not installed. Will default to simulation mode.")
    mavutil = None

# Shared memory configuration
SHM_NAME = "battery_status_stream"
SHM_MAGIC = b"BATT"
SHM_HEADER_FORMAT = "<4sII"  # magic, write_index, sample_count
SHM_RECORD_FORMAT = "<5d"    # timestamp, voltage, current, capacity, consumed_mah
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
SHM_SIZE = SHM_HEADER_SIZE + (MAX_SAMPLES * SHM_RECORD_SIZE)

shared_memory_lock = threading.Lock()
battery_shm = None

def safe_unregister_shm(shm):
    """Prevents Python's resource_tracker from unlinking shared memory when process exits."""
    try:
        from multiprocessing import resource_tracker
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass

def attach_battery_shm():
    """Attaches to or creates the shared memory segment."""
    global battery_shm
    if battery_shm is not None:
        return battery_shm

    try:
        battery_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    except FileExistsError:
        battery_shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
        if battery_shm.size < SHM_SIZE:
            battery_shm.close()
            try:
                battery_shm.unlink()
            except FileNotFoundError:
                pass
            battery_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)

    safe_unregister_shm(battery_shm)
    # Initialize header if newly created/clean
    struct.pack_into(SHM_HEADER_FORMAT, battery_shm.buf, 0, SHM_MAGIC, 0, 0)
    return battery_shm

def write_battery_sample(timestamp, voltage, current, capacity, consumed_mah):
    """Writes a sample record into the shared memory segment."""
    try:
        shm = attach_battery_shm()
        with shared_memory_lock:
            _, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
            record_offset = SHM_HEADER_SIZE + (write_index * SHM_RECORD_SIZE)
            
            struct.pack_into(
                SHM_RECORD_FORMAT,
                shm.buf,
                record_offset,
                float(timestamp),
                float(voltage),
                float(current),
                float(capacity),
                float(consumed_mah)
            )
            
            write_index = (write_index + 1) % MAX_SAMPLES
            sample_count = min(sample_count + 1, MAX_SAMPLES)
            struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)
    except Exception as e:
        print(f"Error writing to shared memory: {e}", file=sys.stderr)

def main():
    parser = argparse.ArgumentParser(description="Monitor battery telemetry and stream to shared memory.")
    parser.add_argument("--connection", type=str, default="udp:127.0.0.1:14552", help="MAVLink connection target.")
    parser.add_argument("--simulate", action="store_true", help="Force simulation mode even if MAVLink is available.")
    parser.add_argument("--rate", type=float, default=2.0, help="Telemetry update rate in Hz.")
    args = parser.parse_args()

    print("Starting battery monitor...")
    print(f"Target connection: {args.connection}")

    # State variables for mAh integration
    last_time = time.time()
    consumed_mah = 0.0

    # Simulation state
    sim_voltage = 12.6
    sim_capacity = 100.0

    master = None
    if not args.simulate and mavutil is not None:
        try:
            print("Connecting to MAVLink vehicle...")
            master = mavutil.mavlink_connection(args.connection)
            # Wait for heartbeat with a short timeout to not block boot indefinitely
            master.wait_heartbeat(timeout=5)
            print("Connected to MAVLink vehicle!")
        except Exception as e:
            print(f"Could not connect to MAVLink: {e}. Falling back to simulation mode.")
            master = None

    try:
        while True:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start

            voltage = 0.0
            current = 0.0
            capacity = 0.0

            if master is not None:
                # Read available messages
                msg = master.recv_match(type=['BATTERY_STATUS', 'HEARTBEAT'], blocking=False)
                # Keep draining the buffer to get the latest messages
                latest_batt_msg = None
                while msg is not None:
                    if msg.get_type() == 'BATTERY_STATUS':
                        latest_batt_msg = msg
                    msg = master.recv_match(type=['BATTERY_STATUS', 'HEARTBEAT'], blocking=False)
                
                if latest_batt_msg is not None:
                    # Voltage (in mV) -> V
                    if latest_batt_msg.voltages[0] != 65535:
                        voltage = latest_batt_msg.voltages[0] / 1000.0
                    
                    # Current (in centiamps) -> A
                    if latest_batt_msg.current_battery != -1:
                        current = latest_batt_msg.current_battery / 100.0
                    
                    # Capacity percentage (0-100)
                    if latest_batt_msg.battery_remaining != 255:
                        capacity = float(latest_batt_msg.battery_remaining)

                    # mAh consumed
                    if latest_batt_msg.current_consumed != -1:
                        consumed_mah = float(latest_batt_msg.current_consumed)
                    else:
                        # Integrate current draw over time
                        # mAh = A * 1000 * dt / 3600
                        consumed_mah += current * 1000.0 * (dt / 3600.0)
                else:
                    # If we don't receive battery messages but we are connected, keep the old values
                    # or temporarily skip writing to not populate zero
                    time.sleep(1.0 / args.rate)
                    continue
            else:
                # Simulation Mode
                # Current draws between 2.0A and 15.0A depending on a pseudo-random wave
                current = 3.0 + 5.0 * math.sin(loop_start * 0.05) + (loop_start % 7) * 0.5
                if current < 0:
                    current = 0.5

                # Integrate mAh consumed
                consumed_mah += current * 1000.0 * (dt / 3600.0)

                # Drain capacity percentage based on consumed mAh (assuming a 2200 mAh battery)
                nominal_capacity_mah = 2200.0
                sim_capacity = max(0.0, 100.0 - (consumed_mah / nominal_capacity_mah) * 100.0)
                capacity = sim_capacity

                # Map capacity to voltage (voltage drops from 12.6V down to 10.5V)
                sim_voltage = 10.5 + (sim_capacity / 100.0) * 2.1
                voltage = sim_voltage

                # Auto-reset simulation when empty to keep testing active
                if sim_capacity <= 0.1:
                    print("Simulation reset: Battery recharged.")
                    consumed_mah = 0.0
                    sim_capacity = 100.0

            # Write telemetry to shared memory
            write_battery_sample(loop_start, voltage, current, capacity, consumed_mah)

            # Control frequency
            elapsed = time.time() - loop_start
            sleep_time = max(0.01, (1.0 / args.rate) - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Shutting down battery monitor...")
    finally:
        global battery_shm
        if battery_shm is not None:
            try:
                battery_shm.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()
