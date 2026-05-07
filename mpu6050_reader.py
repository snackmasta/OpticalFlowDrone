#!/usr/bin/env python3
"""
MPU6050 I2C Sensor Reader
Reads accelerometer and gyroscope data from MPU6050 sensor via I2C
and prints formatted output to the terminal.
"""

import time
from smbus2 import SMBus

# MPU6050 I2C Address
MPU6050_ADDR = 0x68

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Sensitivity scaling factors (for default ranges)
ACCEL_SCALE = 16384.0  # ±2g range: 16384 LSB/g
GYRO_SCALE = 131.0     # ±250°/s range: 131 LSB/(°/s)


def read_word(bus, addr, reg):
    """Read a 16-bit word from I2C bus."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    
    # Convert to signed value
    if value >= 0x8000:
        value -= 65536
    return value


def initialize_mpu6050(bus):
    """Initialize MPU6050 sensor."""
    # Wake up the sensor (clear SLEEP bit)
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    time.sleep(0.2)
    print("✓ MPU6050 initialized successfully\n")


def read_sensor_data(bus):
    """Read accelerometer and gyroscope data from MPU6050."""
    # Read raw accelerometer values
    ax_raw = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H)
    ay_raw = read_word(bus, MPU6050_ADDR, ACCEL_YOUT_H)
    az_raw = read_word(bus, MPU6050_ADDR, ACCEL_ZOUT_H)
    
    # Read raw gyroscope values
    gx_raw = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H)
    gy_raw = read_word(bus, MPU6050_ADDR, GYRO_YOUT_H)
    gz_raw = read_word(bus, MPU6050_ADDR, GYRO_ZOUT_H)
    
    # Convert to physical units
    ax = ax_raw / ACCEL_SCALE
    ay = ay_raw / ACCEL_SCALE
    az = az_raw / ACCEL_SCALE
    
    gx = gx_raw / GYRO_SCALE
    gy = gy_raw / GYRO_SCALE
    gz = gz_raw / GYRO_SCALE
    
    return ax, ay, az, gx, gy, gz


def main():
    """Main function to continuously read and display sensor data."""
    try:
        # Initialize I2C bus
        bus = SMBus(1)  # Bus 1 for Raspberry Pi
        
        # Initialize MPU6050
        initialize_mpu6050(bus)
        print("Starting sensor data stream (Press Ctrl+C to exit)...\n")
        print("-" * 80)
        print(f"{'Time (s)':<10} | {'Accel X (g)':<12} {'Accel Y (g)':<12} {'Accel Z (g)':<12} | "
              f"{'Gyro X (°/s)':<13} {'Gyro Y (°/s)':<13} {'Gyro Z (°/s)':<13}")
        print("-" * 80)
        
        start_time = time.time()
        
        while True:
            elapsed_time = time.time() - start_time
            ax, ay, az, gx, gy, gz = read_sensor_data(bus)
            
            print(f"{elapsed_time:<10.2f} | {ax:<12.4f} {ay:<12.4f} {az:<12.4f} | "
                  f"{gx:<13.2f} {gy:<13.2f} {gz:<13.2f}")
            
            time.sleep(0.1)  # 100ms sampling interval
    
    except KeyboardInterrupt:
        print("\n" + "-" * 80)
        print("Sensor reading stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.close()
        print("I2C bus closed.")


if __name__ == "__main__":
    main()
