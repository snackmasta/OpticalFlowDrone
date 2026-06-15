#!/usr/bin/env python3
"""
Real-time IMU Dashboard
- Reads accelerometer and gyroscope data from MPU6050
- Displays real-time graphs on web interface
- Integrates gyro data for attitude estimation
"""

from flask import Flask, render_template_string, jsonify
from smbus2 import SMBus
import threading
import time
import math
from collections import deque
from datetime import datetime

# Configuration
IMU_I2C_BUS = 1
IMU_I2C_ADDR = 0x68
IMU_PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
ACCEL_LSB_PER_G = 16384.0
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
GYRO_LSB_PER_DPS = 131.0

MAX_SAMPLES = 200  # Keep last 200 samples for graphing
DASHBOARD_PORT = 5002

# Global state
sensor_data = {
    'accel_x': deque(maxlen=MAX_SAMPLES),
    'accel_y': deque(maxlen=MAX_SAMPLES),
    'accel_z': deque(maxlen=MAX_SAMPLES),
    'gyro_x': deque(maxlen=MAX_SAMPLES),
    'gyro_y': deque(maxlen=MAX_SAMPLES),
    'gyro_z': deque(maxlen=MAX_SAMPLES),
    'roll': deque(maxlen=MAX_SAMPLES),
    'pitch': deque(maxlen=MAX_SAMPLES),
    'yaw': deque(maxlen=MAX_SAMPLES),
    'timestamps': deque(maxlen=MAX_SAMPLES),
}

attitude_state = {
    'roll_deg': 0.0,
    'pitch_deg': 0.0,
    'yaw_deg': 0.0,
    'last_ts': None,
}

# Gyro bias calibration (will be set during startup)
gyro_bias = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
}

# Complementary filter coefficient (0-1)
# Higher = more trust in gyro, faster response but more drift
# Lower = more trust in accel, slower response but better stability
COMPLEMENTARY_FILTER_ALPHA = 0.95

data_lock = threading.Lock()
app = Flask(__name__)


def read_i2c_word(bus, addr, reg):
    """Read a 16-bit word from I2C (MSB first, then LSB)."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def normalize_angle(angle):
    """Normalize angle to [-180, 180] range."""
    return ((angle + 180.0) % 360.0) - 180.0


def get_angle_from_accel(ax, ay, az):
    """
    Calculate roll and pitch from accelerometer vector (gravity).
    Yaw cannot be determined from accel alone.
    """
    # Protect against division by zero
    accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
    if accel_mag < 0.1:
        return 0.0, 0.0
    
    # Normalize
    ax /= accel_mag
    ay /= accel_mag
    az /= accel_mag
    
    # Roll: rotation around X axis
    roll = math.atan2(ay, az)
    roll_deg = math.degrees(roll)
    
    # Pitch: rotation around Y axis
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    pitch_deg = math.degrees(pitch)
    
    return normalize_angle(roll_deg), normalize_angle(pitch_deg)


def calibrate_gyro_bias(samples=100):
    """
    Calibrate gyro by averaging readings at rest.
    """
    print("Calibrating gyro bias (keep IMU still)...")
    try:
        bus = SMBus(IMU_I2C_BUS)
        bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
        time.sleep(0.2)
        
        gx_sum = 0.0
        gy_sum = 0.0
        gz_sum = 0.0
        
        for i in range(samples):
            gx_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H)
            gy_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H)
            gz_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H)
            
            gx_sum += gx_raw / GYRO_LSB_PER_DPS
            gy_sum += gy_raw / GYRO_LSB_PER_DPS
            gz_sum += gz_raw / GYRO_LSB_PER_DPS
            
            time.sleep(0.02)
        
        gyro_bias['x'] = gx_sum / samples
        gyro_bias['y'] = gy_sum / samples
        gyro_bias['z'] = gz_sum / samples
        
        print(f"✓ Gyro bias calibrated:")
        print(f"  X: {gyro_bias['x']:.4f} °/s")
        print(f"  Y: {gyro_bias['y']:.4f} °/s")
        print(f"  Z: {gyro_bias['z']:.4f} °/s")
        
        bus.close()
    except Exception as e:
        print(f"✗ Gyro calibration failed: {e}")


def imu_reader_thread():
    """Background thread that reads IMU data continuously."""
    try:
        bus = SMBus(IMU_I2C_BUS)
        bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
        time.sleep(0.2)
        print(f"✓ Connected to IMU on bus {IMU_I2C_BUS} address 0x{IMU_I2C_ADDR:02X}")
        
        last_ts = None
        while True:
            now = time.time()
            
            # Read raw accelerometer values
            ax_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_XOUT_H)
            ay_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_YOUT_H)
            az_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_ZOUT_H)
            
            # Read raw gyroscope values
            gx_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H)
            gy_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H)
            gz_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H)
            
            # Convert to physical units
            ax_g = ax_raw / ACCEL_LSB_PER_G
            ay_g = ay_raw / ACCEL_LSB_PER_G
            az_g = az_raw / ACCEL_LSB_PER_G
            
            # Remove gyro bias (calibrated at startup)
            gx_dps = gx_raw / GYRO_LSB_PER_DPS - gyro_bias['x']
            gy_dps = gy_raw / GYRO_LSB_PER_DPS - gyro_bias['y']
            gz_dps = gz_raw / GYRO_LSB_PER_DPS - gyro_bias['z']
            
            with data_lock:
                if last_ts is not None:
                    dt = now - last_ts
                    if dt > 0 and dt < 0.1:  # Prevent large jumps on pause
                        # Integrate gyro
                        roll_gyro = attitude_state['roll_deg'] + gx_dps * dt
                        pitch_gyro = attitude_state['pitch_deg'] + gy_dps * dt
                        yaw_gyro = attitude_state['yaw_deg'] + gz_dps * dt
                        
                        # Get accel-based angles (only for roll/pitch)
                        roll_accel, pitch_accel = get_angle_from_accel(ax_g, ay_g, az_g)
                        
                        # Complementary filter: blend gyro with accel correction
                        # Gyro: fast, low noise, but drifts (alpha)
                        # Accel: slow, noisy, but no drift (1-alpha)
                        attitude_state['roll_deg'] = normalize_angle(
                            COMPLEMENTARY_FILTER_ALPHA * roll_gyro + (1 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel
                        )
                        attitude_state['pitch_deg'] = normalize_angle(
                            COMPLEMENTARY_FILTER_ALPHA * pitch_gyro + (1 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel
                        )
                        # Yaw: only from gyro (accel can't measure it)
                        attitude_state['yaw_deg'] = normalize_angle(yaw_gyro)
                
                # Store sensor data
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                sensor_data['accel_x'].append(round(ax_g, 3))
                sensor_data['accel_y'].append(round(ay_g, 3))
                sensor_data['accel_z'].append(round(az_g, 3))
                sensor_data['gyro_x'].append(round(gx_dps, 2))
                sensor_data['gyro_y'].append(round(gy_dps, 2))
                sensor_data['gyro_z'].append(round(gz_dps, 2))
                sensor_data['roll'].append(round(attitude_state['roll_deg'], 1))
                sensor_data['pitch'].append(round(attitude_state['pitch_deg'], 1))
                sensor_data['yaw'].append(round(attitude_state['yaw_deg'], 1))
                sensor_data['timestamps'].append(timestamp)
            
            last_ts = now
            time.sleep(0.02)  # ~50 Hz sample rate
            
    except Exception as e:
        print(f"✗ IMU reader error: {e}")
        import traceback
        traceback.print_exc()


@app.route('/')
def index():
    """Serve the main dashboard page."""
    return render_template_string(HTML_TEMPLATE)


@app.route('/api/imu/data')
def get_imu_data():
    """Return current IMU data and historical samples."""
    with data_lock:
        return jsonify({
            'accel_x': list(sensor_data['accel_x']),
            'accel_y': list(sensor_data['accel_y']),
            'accel_z': list(sensor_data['accel_z']),
            'gyro_x': list(sensor_data['gyro_x']),
            'gyro_y': list(sensor_data['gyro_y']),
            'gyro_z': list(sensor_data['gyro_z']),
            'roll': list(sensor_data['roll']),
            'pitch': list(sensor_data['pitch']),
            'yaw': list(sensor_data['yaw']),
            'timestamps': list(sensor_data['timestamps']),
            'current': {
                'accel_x': sensor_data['accel_x'][-1] if sensor_data['accel_x'] else 0,
                'accel_y': sensor_data['accel_y'][-1] if sensor_data['accel_y'] else 0,
                'accel_z': sensor_data['accel_z'][-1] if sensor_data['accel_z'] else 0,
                'gyro_x': sensor_data['gyro_x'][-1] if sensor_data['gyro_x'] else 0,
                'gyro_y': sensor_data['gyro_y'][-1] if sensor_data['gyro_y'] else 0,
                'gyro_z': sensor_data['gyro_z'][-1] if sensor_data['gyro_z'] else 0,
                'roll': sensor_data['roll'][-1] if sensor_data['roll'] else 0,
                'pitch': sensor_data['pitch'][-1] if sensor_data['pitch'] else 0,
                'yaw': sensor_data['yaw'][-1] if sensor_data['yaw'] else 0,
            }
        })


HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Real-Time IMU Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        
        .header {
            background: rgba(255, 255, 255, 0.95);
            padding: 20px 30px;
            border-radius: 10px;
            margin-bottom: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        
        .header h1 {
            color: #333;
            font-size: 28px;
            margin-bottom: 10px;
        }
        
        .status {
            display: flex;
            gap: 20px;
            font-size: 14px;
        }
        
        .status-item {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #4ade80;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        @media (max-width: 1200px) {
            .grid {
                grid-template-columns: 1fr;
            }
        }
        
        .card {
            background: rgba(255, 255, 255, 0.95);
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        
        .card h2 {
            color: #333;
            font-size: 18px;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 2px solid #667eea;
        }
        
        .chart-container {
            position: relative;
            height: 300px;
            margin-bottom: 10px;
        }
        
        .readout {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 15px;
        }
        
        .readout-item {
            background: #f5f5f5;
            padding: 12px;
            border-radius: 6px;
            text-align: center;
        }
        
        .readout-label {
            font-size: 12px;
            color: #666;
            font-weight: 600;
            text-transform: uppercase;
            margin-bottom: 5px;
        }
        
        .readout-value {
            font-size: 20px;
            font-weight: bold;
            color: #333;
            font-family: 'Courier New', monospace;
        }
        
        .attitude-card {
            grid-column: 1 / -1;
        }
        
        .attitude-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 15px;
        }
        
        .attitude-box {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 6px;
            text-align: center;
        }
        
        .attitude-box .label {
            font-size: 12px;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        
        .attitude-box .value {
            font-size: 32px;
            font-weight: bold;
            font-family: 'Courier New', monospace;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>📊 Real-Time IMU Dashboard</h1>
            <div class="status">
                <div class="status-item">
                    <span class="status-indicator"></span>
                    <span>Live Data Stream</span>
                </div>
                <div class="status-item">
                    <span>Sample Rate:</span>
                    <span>50 Hz</span>
                </div>
                <div class="status-item">
                    <span>IMU:</span>
                    <span>MPU6050</span>
                </div>
            </div>
        </div>
        
        <div class="grid">
            <div class="card">
                <h2>📈 Accelerometer (g)</h2>
                <div class="chart-container">
                    <canvas id="accelChart"></canvas>
                </div>
                <div class="readout">
                    <div class="readout-item">
                        <div class="readout-label">X</div>
                        <div class="readout-value" id="accel-x">0.00</div>
                    </div>
                    <div class="readout-item">
                        <div class="readout-label">Y</div>
                        <div class="readout-value" id="accel-y">0.00</div>
                    </div>
                    <div class="readout-item">
                        <div class="readout-label">Z</div>
                        <div class="readout-value" id="accel-z">0.00</div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h2>🔄 Gyroscope (°/s)</h2>
                <div class="chart-container">
                    <canvas id="gyroChart"></canvas>
                </div>
                <div class="readout">
                    <div class="readout-item">
                        <div class="readout-label">X</div>
                        <div class="readout-value" id="gyro-x">0.00</div>
                    </div>
                    <div class="readout-item">
                        <div class="readout-label">Y</div>
                        <div class="readout-value" id="gyro-y">0.00</div>
                    </div>
                    <div class="readout-item">
                        <div class="readout-label">Z</div>
                        <div class="readout-value" id="gyro-z">0.00</div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="card attitude-card">
            <h2>🎯 Attitude Estimation (from Gyro Integration)</h2>
            <div class="chart-container">
                <canvas id="attitudeChart"></canvas>
            </div>
            <div class="attitude-grid">
                <div class="attitude-box">
                    <div class="label">ROLL</div>
                    <div class="value" id="roll-val">0.0°</div>
                </div>
                <div class="attitude-box">
                    <div class="label">PITCH</div>
                    <div class="value" id="pitch-val">0.0°</div>
                </div>
                <div class="attitude-box">
                    <div class="label">YAW</div>
                    <div class="value" id="yaw-val">0.0°</div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // Chart configuration
        const chartOptions = {
            responsive: true,
            maintainAspectRatio: false,
            animation: { duration: 0 },
            plugins: {
                legend: { position: 'top' }
            },
            scales: {
                y: {
                    beginAtZero: true,
                    ticks: { maxTicksLimit: 5 }
                }
            }
        };
        
        // Initialize charts
        const accelCtx = document.getElementById('accelChart').getContext('2d');
        const accelChart = new Chart(accelCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Accel X (g)', borderColor: 'rgb(255, 99, 132)', data: [], tension: 0.1 },
                    { label: 'Accel Y (g)', borderColor: 'rgb(54, 162, 235)', data: [], tension: 0.1 },
                    { label: 'Accel Z (g)', borderColor: 'rgb(75, 192, 75)', data: [], tension: 0.1 }
                ]
            },
            options: chartOptions
        });
        
        const gyroCtx = document.getElementById('gyroChart').getContext('2d');
        const gyroChart = new Chart(gyroCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Gyro X (°/s)', borderColor: 'rgb(255, 99, 132)', data: [], tension: 0.1 },
                    { label: 'Gyro Y (°/s)', borderColor: 'rgb(54, 162, 235)', data: [], tension: 0.1 },
                    { label: 'Gyro Z (°/s)', borderColor: 'rgb(75, 192, 75)', data: [], tension: 0.1 }
                ]
            },
            options: chartOptions
        });
        
        const attitudeCtx = document.getElementById('attitudeChart').getContext('2d');
        const attitudeChart = new Chart(attitudeCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { label: 'Roll (°)', borderColor: 'rgb(255, 99, 132)', data: [], tension: 0.1 },
                    { label: 'Pitch (°)', borderColor: 'rgb(54, 162, 235)', data: [], tension: 0.1 },
                    { label: 'Yaw (°)', borderColor: 'rgb(75, 192, 75)', data: [], tension: 0.1 }
                ]
            },
            options: chartOptions
        });
        
        // Update function
        async function updateDashboard() {
            try {
                const response = await fetch('/api/imu/data');
                const data = await response.json();
                
                // Update readout values
                if (data.current) {
                    document.getElementById('accel-x').textContent = data.current.accel_x.toFixed(2);
                    document.getElementById('accel-y').textContent = data.current.accel_y.toFixed(2);
                    document.getElementById('accel-z').textContent = data.current.accel_z.toFixed(2);
                    document.getElementById('gyro-x').textContent = data.current.gyro_x.toFixed(2);
                    document.getElementById('gyro-y').textContent = data.current.gyro_y.toFixed(2);
                    document.getElementById('gyro-z').textContent = data.current.gyro_z.toFixed(2);
                    document.getElementById('roll-val').textContent = data.current.roll.toFixed(1) + '°';
                    document.getElementById('pitch-val').textContent = data.current.pitch.toFixed(1) + '°';
                    document.getElementById('yaw-val').textContent = data.current.yaw.toFixed(1) + '°';
                }
                
                // Update charts
                accelChart.data.labels = data.timestamps;
                accelChart.data.datasets[0].data = data.accel_x;
                accelChart.data.datasets[1].data = data.accel_y;
                accelChart.data.datasets[2].data = data.accel_z;
                accelChart.update();
                
                gyroChart.data.labels = data.timestamps;
                gyroChart.data.datasets[0].data = data.gyro_x;
                gyroChart.data.datasets[1].data = data.gyro_y;
                gyroChart.data.datasets[2].data = data.gyro_z;
                gyroChart.update();
                
                attitudeChart.data.labels = data.timestamps;
                attitudeChart.data.datasets[0].data = data.roll;
                attitudeChart.data.datasets[1].data = data.pitch;
                attitudeChart.data.datasets[2].data = data.yaw;
                attitudeChart.update();
            } catch (error) {
                console.error('Update error:', error);
            }
        }
        
        // Update every 200ms
        setInterval(updateDashboard, 200);
        updateDashboard();
    </script>
</body>
</html>
"""


if __name__ == '__main__':
    print("=" * 60)
    print("Real-Time IMU Dashboard")
    print("=" * 60)
    print()
    
    # Calibrate gyro first (keep IMU still!)
    calibrate_gyro_bias(samples=200)
    print()
    
    # Start IMU reader thread
    imu_thread = threading.Thread(target=imu_reader_thread, daemon=True)
    imu_thread.start()
    
    # Give IMU thread time to initialize
    time.sleep(1)
    
    # Start Flask server
    print(f"\n✓ Starting Flask dashboard on http://0.0.0.0:{DASHBOARD_PORT}")
    print(f"  Access locally: http://localhost:{DASHBOARD_PORT}")
    print(f"  Access from network: http://<your-ip>:{DASHBOARD_PORT}")
    print("\n" + "=" * 60)
    
    try:
        app.run(host='0.0.0.0', port=DASHBOARD_PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n\nDashboard stopped")
