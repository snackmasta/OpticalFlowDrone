#!/usr/bin/env python3
"""
MAVLink Reboot Script
Sends a reboot command to the drone via MAVLink every 10 minutes
"""

import time
import sys
from pymavlink import mavutil

# Connection parameters
# Adjust these based on your mavproxy output configuration
MAVLINK_HOST = "127.0.0.1"
MAVLINK_PORT = 14550
REBOOT_INTERVAL = 480  # 8 minutes in seconds

# MAVLink command IDs
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_COMPONENT_ARM_DISARM = 400

def send_reboot_command(master):
    """Send a reboot command to the drone."""
    try:
        # Create and send COMMAND_LONG message for reboot
        # param1 = 1 means reboot autopilot, param2 = 0 means don't reboot onboard computer
        master.mav.command_long_send(
            target_system=1,           # Target system ID (usually 1)
            target_component=1,        # Target component ID (usually 1)
            command=MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            confirmation=0,
            param1=1,                  # Reboot autopilot
            param2=0,                  # Don't reboot onboard computer
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] Reboot command sent to drone")
        return True
        
    except Exception as e:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] Error sending reboot command: {e}")
        return False

def main():
    """Main loop to send reboot commands every 10 minutes."""
    
    print("MAVLink Reboot Script Starting...")
    print(f"Connecting to {MAVLINK_HOST}:{MAVLINK_PORT}")
    
    try:
        # Connect to the MAVLink source
        master = mavutil.mavlink_connection(f"udpin:{MAVLINK_HOST}:{MAVLINK_PORT}")
        
        # Wait for first heartbeat
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        print("Heartbeat received! Connected to drone.")
        
        # Get drone info
        print(f"Drone system ID: {master.target_system}")
        print(f"Drone component ID: {master.target_component}")
        
        # Main loop - send reboot command every 10 minutes
        iteration = 0
        while True:
            iteration += 1
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"\n[{timestamp}] Iteration {iteration}: Sending reboot command in 10 minutes...")
            
            # Wait for 10 minutes
            time.sleep(REBOOT_INTERVAL)
            
            # Send reboot command
            send_reboot_command(master)
            
    except Exception as e:
        print(f"Connection error: {e}")
        print("Make sure MAVProxy is running and outputting to UDP port 14550")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nScript interrupted by user")
        sys.exit(0)

if __name__ == "__main__":
    main()
