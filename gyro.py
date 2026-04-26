from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
master.wait_heartbeat()

print("Connected")

while True:
    msg = master.recv_match(type='RAW_IMU', blocking=True)

    if msg:
        print(f"Gyro X: {msg.xgyro:.4f} rad/s")
        print(f"Gyro Y: {msg.ygyro:.4f} rad/s")
        print(f"Gyro Z: {msg.zgyro:.4f} rad/s")
        print("-----")
