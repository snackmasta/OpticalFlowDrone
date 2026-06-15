from pymavlink import mavutil

# Connect to FC
master = mavutil.mavlink_connection('/dev/serial0', baud=921600)

# Wait for heartbeat
master.wait_heartbeat()
print("Connected")

while True:
    msg = master.recv_match(type='DISTANCE_SENSOR', blocking=True)

    if msg:
        print(f"Distance: {msg.current_distance} cm")
        print(f"Min: {msg.min_distance}, Max: {msg.max_distance}")
        print(f"Orientation: {msg.orientation}")
        print("-----")
