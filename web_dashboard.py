#!/usr/bin/env python3
"""
MPU6050 Web Dashboard
Real-time visualization of accelerometer and gyroscope data via web interface
"""

import time
import threading
from collections import deque
from flask import Flask, render_template, jsonify
from smbus2 import SMBus

# MPU6050 I2C Address and Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Sensitivity scaling factors
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

# Data storage (keep last 100 samples)
MAX_SAMPLES = 100
sensor_data = {
    'timestamps': deque(maxlen=MAX_SAMPLES),
    'accel_x': deque(maxlen=MAX_SAMPLES),
    'accel_y': deque(maxlen=MAX_SAMPLES),
    'accel_z': deque(maxlen=MAX_SAMPLES),
    'gyro_x': deque(maxlen=MAX_SAMPLES),
    'gyro_y': deque(maxlen=MAX_SAMPLES),
    'gyro_z': deque(maxlen=MAX_SAMPLES),
}

app = Flask(__name__)


def read_word(bus, addr, reg):
    """Read a 16-bit word from I2C bus."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    
    if value >= 0x8000:
        value -= 65536
    return value


def sensor_thread_worker():
    """Background thread to continuously read sensor data."""
    try:
        bus = SMBus(1)
        bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
        time.sleep(0.2)
        print("MPU6050 sensor initialized")
        
        start_time = time.time()
        
        while True:
            try:
                elapsed = time.time() - start_time
                
                # Read accelerometer
                ax = read_word(bus, MPU6050_ADDR, ACCEL_XOUT_H) / ACCEL_SCALE
                ay = read_word(bus, MPU6050_ADDR, ACCEL_YOUT_H) / ACCEL_SCALE
                az = read_word(bus, MPU6050_ADDR, ACCEL_ZOUT_H) / ACCEL_SCALE
                
                # Read gyroscope
                gx = read_word(bus, MPU6050_ADDR, GYRO_XOUT_H) / GYRO_SCALE
                gy = read_word(bus, MPU6050_ADDR, GYRO_YOUT_H) / GYRO_SCALE
                gz = read_word(bus, MPU6050_ADDR, GYRO_ZOUT_H) / GYRO_SCALE
                
                # Store data
                sensor_data['timestamps'].append(round(elapsed, 2))
                sensor_data['accel_x'].append(round(ax, 4))
                sensor_data['accel_y'].append(round(ay, 4))
                sensor_data['accel_z'].append(round(az, 4))
                sensor_data['gyro_x'].append(round(gx, 2))
                sensor_data['gyro_y'].append(round(gy, 2))
                sensor_data['gyro_z'].append(round(gz, 2))
                
                time.sleep(0.1)
            except Exception as e:
                print(f"Error reading sensor: {e}")
                time.sleep(1)
    
    except Exception as e:
        print(f"Sensor thread error: {e}")
    finally:
        try:
            bus.close()
        except:
            pass


@app.route('/')
def index():
    """Serve the dashboard HTML."""
    return render_template('dashboard.html')


@app.route('/api/sensor-data')
def get_sensor_data():
    """Return latest sensor data as JSON."""
    return jsonify({
        'timestamps': list(sensor_data['timestamps']),
        'accel_x': list(sensor_data['accel_x']),
        'accel_y': list(sensor_data['accel_y']),
        'accel_z': list(sensor_data['accel_z']),
        'gyro_x': list(sensor_data['gyro_x']),
        'gyro_y': list(sensor_data['gyro_y']),
        'gyro_z': list(sensor_data['gyro_z']),
    })


def main():
    """Start the Flask web server."""
    # Start sensor reading thread
    sensor_thread = threading.Thread(target=sensor_thread_worker, daemon=True)
    sensor_thread.start()
    
    print("\n" + "="*60)
    print("MPU6050 Web Dashboard")
    print("="*60)
    print("Starting Flask server...")
    print("Open your browser to: http://localhost:5000")
    print("="*60 + "\n")
    
    # Start Flask app
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()
