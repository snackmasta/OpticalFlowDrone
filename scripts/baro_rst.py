from pymavlink import mavutil
import serial
import time


def get_lidar_distance_meters(master, timeout=1.0):
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True, timeout=timeout)
    if msg is None:
        return None

    return msg.current_distance / 100.0


# Connect to ArduPilot
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

# Example lidar altitude
lidar_alt = 0.0

while True:

    # Replace with actual lidar reading
    lidar_alt = get_lidar_distance_meters(master)
    if lidar_alt is None:
        continue

    master.mav.vision_position_estimate_send(
        int(time.time()*1e6),
        0.0,
        0.0,
        -lidar_alt,
        0.0,
        0.0,
        0.0
    )

    time.sleep(0.02)