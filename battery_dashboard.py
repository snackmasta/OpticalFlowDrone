#!/usr/bin/env python3
"""
MAVLink Battery Dashboard with Automatic Logging
- Displays real-time battery status
- Logs battery data when drone is armed
- Stops logging when drone is disarmed
- Web-based dashboard using Flask
"""

from pymavlink import mavutil
from flask import Flask, render_template_string, jsonify
import threading
import csv
import os
import time
from datetime import datetime
import queue

# Configuration
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 921600
LOG_DIR = './battery_logs'

# Create log directory if it doesn't exist
os.makedirs(LOG_DIR, exist_ok=True)

# Global state
vehicle_state = {
    'armed': False,
    'battery_voltage': 0.0,
    'battery_current': 0.0,
    'battery_remaining': 0,
    'is_logging': False,
    'log_filename': None,
    'flight_time': 0,
    'system_status': 'UNKNOWN',
    'connected': False,
    'last_update': 0,
}

data_queue = queue.Queue()
logging_lock = threading.Lock()
log_file_handle = None

app = Flask(__name__)

def connect_to_vehicle():
    """Establish MAVLink connection to the flight controller"""
    try:
        print(f"Connecting to vehicle on {SERIAL_PORT} at {BAUD_RATE} baud...")
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        master.wait_heartbeat()
        print("✓ Connected to vehicle!")
        vehicle_state['connected'] = True
        return master
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        vehicle_state['connected'] = False
        return None

def start_battery_logging():
    """Start a new battery log file"""
    global log_file_handle
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(LOG_DIR, f"battery_log_{timestamp}.csv")
    
    try:
        log_file_handle = open(filename, 'w', newline='')
        writer = csv.writer(log_file_handle)
        writer.writerow(['Timestamp', 'Voltage (V)', 'Current (A)', 'Remaining (%)', 'Flight Time (s)'])
        log_file_handle.flush()
        
        vehicle_state['log_filename'] = filename
        vehicle_state['is_logging'] = True
        print(f"✓ Started logging to {filename}")
        return filename
    except Exception as e:
        print(f"✗ Failed to start logging: {e}")
        return None

def stop_battery_logging():
    """Stop battery logging and close the log file"""
    global log_file_handle
    
    with logging_lock:
        if log_file_handle:
            log_file_handle.close()
            log_file_handle = None
            vehicle_state['is_logging'] = False
            print(f"✓ Stopped logging: {vehicle_state['log_filename']}")

def log_battery_data(voltage, current, remaining, flight_time):
    """Log battery data to CSV file if logging is active"""
    if not vehicle_state['is_logging'] or not log_file_handle:
        return
    
    try:
        with logging_lock:
            if log_file_handle:
                writer = csv.writer(log_file_handle)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                writer.writerow([timestamp, f"{voltage:.2f}", f"{current:.2f}", remaining, flight_time])
                log_file_handle.flush()
    except Exception as e:
        print(f"✗ Error logging battery data: {e}")

def mavlink_receiver(master):
    """MAVLink message receiver thread"""
    while True:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is None:
                continue
            
            msg_type = msg.get_type()
            
            # Update system status (contains armed state)
            if msg_type == 'HEARTBEAT':
                mode = mavutil.mode_string_v10(msg)
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                
                # Detect arm/disarm transitions
                if is_armed and not vehicle_state['armed']:
                    print("✓ Drone ARMED - Starting battery log")
                    vehicle_state['armed'] = True
                    start_battery_logging()
                elif not is_armed and vehicle_state['armed']:
                    print("✓ Drone DISARMED - Stopping battery log")
                    vehicle_state['armed'] = False
                    stop_battery_logging()
                
                vehicle_state['system_status'] = mode
                vehicle_state['last_update'] = time.time()
            
            # Battery information (typically sent at 1Hz)
            elif msg_type == 'BATTERY_STATUS':
                if msg.battery_function == 0:  # Primary battery
                    # Voltage is in mV, convert to V
                    voltage = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0.0
                    # Current is in cA (centamps), convert to A
                    current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0.0
                    # Battery remaining percentage
                    remaining = msg.battery_remaining if msg.battery_remaining != 255 else 0
                    
                    vehicle_state['battery_voltage'] = voltage
                    vehicle_state['battery_current'] = current
                    vehicle_state['battery_remaining'] = remaining
                    
                    # Log if armed
                    if vehicle_state['armed']:
                        log_battery_data(voltage, current, remaining, vehicle_state['flight_time'])
            
            # Get flight time from ATTITUDE message (or other time-based messages)
            elif msg_type == 'SYSTEM_TIME':
                vehicle_state['last_update'] = time.time()
        
        except Exception as e:
            print(f"✗ MAVLink receiver error: {e}")
            time.sleep(0.1)

def read_log_file(filename):
    """Read and parse a log CSV file"""
    filepath = os.path.join(LOG_DIR, filename)
    if not os.path.exists(filepath):
        return None
    
    data = {
        'timestamps': [],
        'voltages': [],
        'currents': [],
        'remaining': [],
        'flight_times': []
    }
    
    try:
        with open(filepath, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    data['timestamps'].append(row['Timestamp'])
                    data['voltages'].append(float(row['Voltage (V)']))
                    data['currents'].append(float(row['Current (A)']))
                    data['remaining'].append(int(row['Remaining (%)']))
                    data['flight_times'].append(int(row['Flight Time (s)']))
                except (ValueError, KeyError):
                    continue
        return data
    except Exception as e:
        print(f"Error reading log file: {e}")
        return None

@app.route('/')
def dashboard():
    """Serve the dashboard HTML"""
    html = '''
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>MAVLink Battery Dashboard</title>
        <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
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
                max-width: 900px;
                margin: 0 auto;
            }
            
            header {
                text-align: center;
                color: white;
                margin-bottom: 30px;
            }
            
            h1 {
                font-size: 2.5em;
                margin-bottom: 10px;
                text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            }
            
            .status-banner {
                background: rgba(255,255,255,0.1);
                padding: 10px 20px;
                border-radius: 10px;
                margin-bottom: 20px;
                font-size: 1.1em;
                color: white;
            }
            
            .grid {
                display: grid;
                grid-template-columns: 1fr 1fr;
                gap: 20px;
                margin-bottom: 20px;
            }
            
            .card {
                background: white;
                border-radius: 15px;
                padding: 25px;
                box-shadow: 0 10px 30px rgba(0,0,0,0.3);
                transition: transform 0.3s, box-shadow 0.3s;
            }
            
            .card:hover {
                transform: translateY(-5px);
                box-shadow: 0 15px 40px rgba(0,0,0,0.4);
            }
            
            .card h2 {
                color: #667eea;
                margin-bottom: 15px;
                font-size: 1.3em;
                border-bottom: 2px solid #667eea;
                padding-bottom: 10px;
            }
            
            .metric {
                margin-bottom: 15px;
            }
            
            .metric-label {
                color: #666;
                font-size: 0.9em;
                margin-bottom: 5px;
            }
            
            .metric-value {
                font-size: 1.8em;
                font-weight: bold;
                color: #333;
            }
            
            .metric-unit {
                font-size: 0.6em;
                color: #999;
                margin-left: 5px;
            }
            
            .status-indicator {
                display: inline-block;
                width: 12px;
                height: 12px;
                border-radius: 50%;
                margin-right: 8px;
                animation: pulse 2s infinite;
            }
            
            .status-connected {
                background: #4CAF50;
            }
            
            .status-disconnected {
                background: #f44336;
                animation: none;
            }
            
            .status-armed {
                background: #ff9800;
                animation: pulse 1s infinite;
            }
            
            .status-disarmed {
                background: #2196F3;
            }
            
            @keyframes pulse {
                0%, 100% { opacity: 1; }
                50% { opacity: 0.5; }
            }
            
            .battery-bar {
                width: 100%;
                height: 25px;
                background: #eee;
                border-radius: 5px;
                overflow: hidden;
                margin: 10px 0;
                border: 1px solid #ddd;
            }
            
            .battery-fill {
                height: 100%;
                background: linear-gradient(90deg, #4CAF50, #8BC34A);
                display: flex;
                align-items: center;
                justify-content: center;
                color: white;
                font-weight: bold;
                font-size: 0.9em;
                transition: width 0.3s ease;
            }
            
            .battery-fill.low {
                background: linear-gradient(90deg, #ff9800, #f44336);
            }
            
            .battery-fill.critical {
                background: #f44336;
                animation: blink 0.5s infinite;
            }
            
            @keyframes blink {
                0%, 50% { opacity: 1; }
                51%, 100% { opacity: 0.7; }
            }
            
            .button-group {
                display: flex;
                gap: 10px;
                margin-top: 20px;
            }
            
            button {
                flex: 1;
                padding: 12px 20px;
                border: none;
                border-radius: 8px;
                font-size: 1em;
                font-weight: bold;
                cursor: pointer;
                transition: all 0.3s;
            }
            
            .btn-primary {
                background: #667eea;
                color: white;
            }
            
            .btn-primary:hover {
                background: #5568d3;
            }
            
            .btn-success {
                background: #4CAF50;
                color: white;
            }
            
            .btn-success:hover {
                background: #45a049;
            }
            
            .btn-danger {
                background: #f44336;
                color: white;
            }
            
            .btn-danger:hover {
                background: #da190b;
            }
            
            .log-info {
                background: #f5f5f5;
                padding: 15px;
                border-radius: 8px;
                margin-top: 15px;
                font-size: 0.9em;
                color: #666;
            }
            
            @media (max-width: 600px) {
                .grid {
                    grid-template-columns: 1fr;
                }
                
                h1 {
                    font-size: 1.8em;
                }
            }
            
            .tabs {
                display: flex;
                gap: 10px;
                margin-bottom: 20px;
                border-bottom: 2px solid #ddd;
            }
            
            .tab-btn {
                padding: 12px 20px;
                border: none;
                background: transparent;
                cursor: pointer;
                font-size: 1em;
                font-weight: 500;
                color: #666;
                border-bottom: 3px solid transparent;
                transition: all 0.3s;
            }
            
            .tab-btn.active {
                color: #667eea;
                border-bottom-color: #667eea;
            }
            
            .tab-btn:hover {
                color: #667eea;
            }
            
            .tab-content {
                display: none;
            }
            
            .tab-content.active {
                display: block;
            }
            
            .log-list {
                max-height: 400px;
                overflow-y: auto;
            }
            
            .log-item {
                background: white;
                padding: 15px;
                margin-bottom: 10px;
                border-radius: 8px;
                cursor: pointer;
                transition: all 0.3s;
                border-left: 4px solid #667eea;
            }
            
            .log-item:hover {
                transform: translateX(5px);
                box-shadow: 0 5px 15px rgba(0,0,0,0.1);
            }
            
            .log-item.selected {
                background: #f0f4ff;
                border-left-color: #4CAF50;
            }
            
            .log-item-name {
                font-weight: bold;
                color: #333;
                margin-bottom: 5px;
            }
            
            .log-item-stats {
                font-size: 0.85em;
                color: #999;
            }
            
            .chart-container {
                position: relative;
                height: 400px;
                margin-bottom: 30px;
                background: white;
                padding: 20px;
                border-radius: 15px;
                box-shadow: 0 10px 30px rgba(0,0,0,0.1);
            }
            
            .chart-title {
                font-size: 1.2em;
                font-weight: bold;
                color: #333;
                margin-bottom: 15px;
            }
            
            .log-stats {
                display: grid;
                grid-template-columns: repeat(2, 1fr);
                gap: 15px;
                margin-bottom: 20px;
            }
            
            .stat-box {
                background: white;
                padding: 15px;
                border-radius: 8px;
                border-left: 4px solid #667eea;
            }
            
            .stat-box.green { border-left-color: #4CAF50; }
            .stat-box.orange { border-left-color: #ff9800; }
            .stat-box.blue { border-left-color: #2196F3; }
            .stat-box.red { border-left-color: #f44336; }
            
            .stat-label {
                font-size: 0.85em;
                color: #666;
                margin-bottom: 5px;
            }
            
            .stat-value {
                font-size: 1.5em;
                font-weight: bold;
                color: #333;
            }
            
            .no-data {
                background: white;
                padding: 40px;
                text-align: center;
                border-radius: 15px;
                color: #999;
            }
            
            .button-group {
                display: flex;
                gap: 10px;
                margin-top: 20px;
                flex-wrap: wrap;
            }
            
            .btn-download {
                flex: 1;
                min-width: 150px;
                padding: 12px 20px;
                background: #4CAF50;
                color: white;
                border: none;
                border-radius: 8px;
                cursor: pointer;
                font-weight: bold;
                transition: all 0.3s;
            }
            
            .btn-download:hover {
                background: #45a049;
            }
            
            .btn-delete {
                flex: 1;
                min-width: 150px;
                padding: 12px 20px;
                background: #f44336;
                color: white;
                border: none;
                border-radius: 8px;
                cursor: pointer;
                font-weight: bold;
                transition: all 0.3s;
            }
            
            .btn-delete:hover {
                background: #da190b;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <header>
                <h1>🚁 MAVLink Battery Dashboard</h1>
                <div class="status-banner">
                    <span class="status-indicator status-disconnected" id="connection-status"></span>
                    Connection: <strong id="connection-text">Disconnected</strong>
                    &nbsp;&nbsp;&nbsp;
                    <span class="status-indicator status-disarmed" id="armed-status"></span>
                    State: <strong id="armed-text">Disarmed</strong>
                </div>
            </header>
            
            <div class="grid">
                <div class="card">
                    <h2>⚡ Battery Status</h2>
                    <div class="metric">
                        <div class="metric-label">Voltage</div>
                        <div class="metric-value"><span id="voltage">0.00</span><span class="metric-unit">V</span></div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Current</div>
                        <div class="metric-value"><span id="current">0.00</span><span class="metric-unit">A</span></div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Remaining Capacity</div>
                        <div class="battery-bar">
                            <div class="battery-fill" id="battery-fill" style="width: 0%">0%</div>
                        </div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Flight Mode</div>
                        <div class="metric-value"><span id="flight-mode">UNKNOWN</span></div>
                    </div>
                </div>
                
                <div class="card">
                    <h2>📊 Logging Status</h2>
                    <div class="metric">
                        <div class="metric-label">Logging Active</div>
                        <div class="metric-value">
                            <span class="status-indicator" id="logging-indicator"></span>
                            <span id="logging-status">Inactive</span>
                        </div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Current Log File</div>
                        <div class="log-info" id="log-file">No active log</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Flight Time</div>
                        <div class="metric-value"><span id="flight-time">00:00:00</span></div>
                    </div>
                </div>
            </div>
            
            <!-- Tabs for Navigation -->
            <div class="tabs">
                <button class="tab-btn active" onclick="switchTab('live')">🔴 Live Data</button>
                <button class="tab-btn" onclick="switchTab('logs')">📁 Log Viewer</button>
            </div>
            
            <!-- Live Data Tab -->
            <div id="live" class="tab-content active">
                <!-- Will be populated by JavaScript -->
            </div>
            
            <!-- Log Viewer Tab -->
            <div id="logs" class="tab-content">
                <div class="grid" style="grid-template-columns: 1fr 2fr;">
                    <div class="card">
                        <h2>📋 Available Logs</h2>
                        <div class="log-list" id="log-list">
                            <div class="no-data">No logs available</div>
                        </div>
                    </div>
                    <div id="log-viewer-container">
                        <div class="card">
                            <div class="no-data">Select a log file to view</div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
        
        <script>
            let currentLogData = null;
            let chartInstances = {};
            
            function switchTab(tabName) {
                // Hide all tabs
                document.querySelectorAll('.tab-content').forEach(tab => {
                    tab.classList.remove('active');
                });
                document.querySelectorAll('.tab-btn').forEach(btn => {
                    btn.classList.remove('active');
                });
                
                // Show selected tab
                document.getElementById(tabName).classList.add('active');
                event.target.classList.add('active');
                
                if (tabName === 'logs') {
                    loadLogsList();
                }
            }
            
            function updateDashboard() {
                fetch('/api/status')
                    .then(response => response.json())
                    .then(data => {
                        // Battery info
                        document.getElementById('voltage').textContent = data.battery_voltage.toFixed(2);
                        document.getElementById('current').textContent = data.battery_current.toFixed(2);
                        document.getElementById('flight-mode').textContent = data.system_status;
                        
                        // Battery bar
                        const remaining = data.battery_remaining;
                        const fill = document.getElementById('battery-fill');
                        fill.style.width = remaining + '%';
                        fill.textContent = remaining + '%';
                        
                        // Change color based on level
                        fill.classList.remove('low', 'critical');
                        if (remaining < 20) {
                            fill.classList.add('critical');
                        } else if (remaining < 40) {
                            fill.classList.add('low');
                        }
                        
                        // Connection status
                        const connStatus = document.getElementById('connection-status');
                        const armedStatus = document.getElementById('armed-status');
                        const loggingIndicator = document.getElementById('logging-indicator');
                        
                        if (data.connected) {
                            connStatus.className = 'status-indicator status-connected';
                            document.getElementById('connection-text').textContent = 'Connected';
                        } else {
                            connStatus.className = 'status-indicator status-disconnected';
                            document.getElementById('connection-text').textContent = 'Disconnected';
                        }
                        
                        if (data.armed) {
                            armedStatus.className = 'status-indicator status-armed';
                            document.getElementById('armed-text').textContent = 'Armed';
                        } else {
                            armedStatus.className = 'status-indicator status-disarmed';
                            document.getElementById('armed-text').textContent = 'Disarmed';
                        }
                        
                        if (data.is_logging) {
                            loggingIndicator.className = 'status-indicator status-armed';
                            document.getElementById('logging-status').textContent = 'Active';
                        } else {
                            loggingIndicator.className = 'status-indicator status-disarmed';
                            document.getElementById('logging-status').textContent = 'Inactive';
                        }
                        
                        // Log file info
                        if (data.log_filename) {
                            document.getElementById('log-file').textContent = 
                                data.log_filename.split('/').pop();
                        }
                    })
                    .catch(error => console.error('Error fetching status:', error));
            }
            
            function loadLogsList() {
                fetch('/api/logs/list')
                    .then(response => response.json())
                    .then(data => {
                        const listContainer = document.getElementById('log-list');
                        
                        if (!data.logs || data.logs.length === 0) {
                            listContainer.innerHTML = '<div class="no-data">No logs available</div>';
                            return;
                        }
                        
                        listContainer.innerHTML = '';
                        data.logs.forEach(logFile => {
                            const item = document.createElement('div');
                            item.className = 'log-item';
                            item.innerHTML = `
                                <div class="log-item-name">${logFile}</div>
                                <div class="log-item-stats">Click to view details</div>
                            `;
                            item.onclick = () => loadLogData(logFile);
                            listContainer.appendChild(item);
                        });
                    })
                    .catch(error => console.error('Error loading logs:', error));
            }
            
            function loadLogData(filename) {
                fetch(`/api/logs/data/${filename}`)
                    .then(response => response.json())
                    .then(data => {
                        if (data.error) {
                            alert('Error loading log: ' + data.error);
                            return;
                        }
                        
                        currentLogData = data;
                        displayLogData(filename, data);
                        
                        // Mark selected
                        document.querySelectorAll('.log-item').forEach(item => {
                            item.classList.remove('selected');
                        });
                        event.target.closest('.log-item').classList.add('selected');
                    })
                    .catch(error => console.error('Error loading log data:', error));
            }
            
            function displayLogData(filename, data) {
                const container = document.getElementById('log-viewer-container');
                
                // Calculate statistics
                const stats = {
                    duration: data.flight_times[data.flight_times.length - 1] || 0,
                    min_voltage: Math.min(...data.voltages),
                    max_voltage: Math.max(...data.voltages),
                    avg_voltage: (data.voltages.reduce((a,b) => a+b, 0) / data.voltages.length).toFixed(2),
                    max_current: Math.max(...data.currents),
                    avg_current: (data.currents.reduce((a,b) => a+b, 0) / data.currents.length).toFixed(2),
                    min_battery: Math.min(...data.remaining),
                    data_points: data.flight_times.length
                };
                
                container.innerHTML = `
                    <div class="card">
                        <h2>📊 Log Analysis: ${filename.replace('battery_log_', '').replace('.csv', '')}</h2>
                        
                        <div class="log-stats">
                            <div class="stat-box green">
                                <div class="stat-label">Duration</div>
                                <div class="stat-value">${Math.floor(stats.duration / 60)}m ${stats.duration % 60}s</div>
                            </div>
                            <div class="stat-box blue">
                                <div class="stat-label">Data Points</div>
                                <div class="stat-value">${stats.data_points}</div>
                            </div>
                            <div class="stat-box orange">
                                <div class="stat-label">Voltage Range</div>
                                <div class="stat-value">${stats.min_voltage.toFixed(2)} - ${stats.max_voltage.toFixed(2)} V</div>
                            </div>
                            <div class="stat-box orange">
                                <div class="stat-label">Max Current</div>
                                <div class="stat-value">${stats.max_current.toFixed(2)} A</div>
                            </div>
                            <div class="stat-box blue">
                                <div class="stat-label">Avg Voltage</div>
                                <div class="stat-value">${stats.avg_voltage} V</div>
                            </div>
                            <div class="stat-box blue">
                                <div class="stat-label">Avg Current</div>
                                <div class="stat-value">${stats.avg_current} A</div>
                            </div>
                            <div class="stat-box red">
                                <div class="stat-label">Min Battery</div>
                                <div class="stat-value">${stats.min_battery}%</div>
                            </div>
                            <div class="stat-box green">
                                <div class="stat-label">Data Quality</div>
                                <div class="stat-value">${((stats.data_points / (stats.duration || 1)) * 100).toFixed(0)}%</div>
                            </div>
                        </div>
                        
                        <div class="chart-container">
                            <div class="chart-title">Battery Voltage Over Time</div>
                            <canvas id="voltageChart"></canvas>
                        </div>
                        
                        <div class="chart-container">
                            <div class="chart-title">Current Draw Over Time</div>
                            <canvas id="currentChart"></canvas>
                        </div>
                        
                        <div class="chart-container">
                            <div class="chart-title">Battery Remaining Percentage</div>
                            <canvas id="remainingChart"></canvas>
                        </div>
                        
                        <div class="chart-container">
                            <div class="chart-title">Voltage vs Current (Power Profile)</div>
                            <canvas id="powerChart"></canvas>
                        </div>
                        
                        <div class="button-group">
                            <button class="btn-download" onclick="downloadLogFile('${filename}')">📥 Download CSV</button>
                        </div>
                    </div>
                `;
                
                // Destroy existing charts
                Object.keys(chartInstances).forEach(key => {
                    if (chartInstances[key]) {
                        chartInstances[key].destroy();
                    }
                });
                
                // Create voltage chart
                setTimeout(() => {
                    const voltageCtx = document.getElementById('voltageChart');
                    if (voltageCtx) {
                        chartInstances.voltage = new Chart(voltageCtx, {
                            type: 'line',
                            data: {
                                labels: data.flight_times,
                                datasets: [{
                                    label: 'Voltage (V)',
                                    data: data.voltages,
                                    borderColor: '#FF6B6B',
                                    backgroundColor: 'rgba(255, 107, 107, 0.1)',
                                    tension: 0.3,
                                    fill: true,
                                    borderWidth: 2
                                }]
                            },
                            options: {
                                responsive: true,
                                maintainAspectRatio: false,
                                plugins: {
                                    legend: { display: true }
                                },
                                scales: {
                                    y: {
                                        beginAtZero: false,
                                        title: { display: true, text: 'Voltage (V)' }
                                    },
                                    x: {
                                        title: { display: true, text: 'Flight Time (s)' }
                                    }
                                }
                            }
                        });
                    }
                    
                    const currentCtx = document.getElementById('currentChart');
                    if (currentCtx) {
                        chartInstances.current = new Chart(currentCtx, {
                            type: 'line',
                            data: {
                                labels: data.flight_times,
                                datasets: [{
                                    label: 'Current (A)',
                                    data: data.currents,
                                    borderColor: '#4ECDC4',
                                    backgroundColor: 'rgba(78, 205, 196, 0.1)',
                                    tension: 0.3,
                                    fill: true,
                                    borderWidth: 2
                                }]
                            },
                            options: {
                                responsive: true,
                                maintainAspectRatio: false,
                                plugins: {
                                    legend: { display: true }
                                },
                                scales: {
                                    y: {
                                        beginAtZero: true,
                                        title: { display: true, text: 'Current (A)' }
                                    },
                                    x: {
                                        title: { display: true, text: 'Flight Time (s)' }
                                    }
                                }
                            }
                        });
                    }
                    
                    const remainingCtx = document.getElementById('remainingChart');
                    if (remainingCtx) {
                        chartInstances.remaining = new Chart(remainingCtx, {
                            type: 'line',
                            data: {
                                labels: data.flight_times,
                                datasets: [{
                                    label: 'Battery Remaining (%)',
                                    data: data.remaining,
                                    borderColor: '#95E1D3',
                                    backgroundColor: 'rgba(149, 225, 211, 0.1)',
                                    tension: 0.3,
                                    fill: true,
                                    borderWidth: 2
                                }]
                            },
                            options: {
                                responsive: true,
                                maintainAspectRatio: false,
                                plugins: {
                                    legend: { display: true }
                                },
                                scales: {
                                    y: {
                                        beginAtZero: true,
                                        max: 100,
                                        title: { display: true, text: 'Remaining (%)' }
                                    },
                                    x: {
                                        title: { display: true, text: 'Flight Time (s)' }
                                    }
                                }
                            }
                        });
                    }
                    
                    const powerCtx = document.getElementById('powerChart');
                    if (powerCtx) {
                        chartInstances.power = new Chart(powerCtx, {
                            type: 'scatter',
                            data: {
                                datasets: [{
                                    label: 'Power Profile',
                                    data: data.voltages.map((v, i) => ({
                                        x: v,
                                        y: data.currents[i]
                                    })),
                                    borderColor: '#667eea',
                                    backgroundColor: 'rgba(102, 126, 234, 0.5)',
                                    borderWidth: 1
                                }]
                            },
                            options: {
                                responsive: true,
                                maintainAspectRatio: false,
                                plugins: {
                                    legend: { display: true }
                                },
                                scales: {
                                    y: {
                                        beginAtZero: true,
                                        title: { display: true, text: 'Current (A)' }
                                    },
                                    x: {
                                        title: { display: true, text: 'Voltage (V)' }
                                    }
                                }
                            }
                        });
                    }
                }, 100);
            }
            
            function downloadLogFile(filename) {
                const link = document.createElement('a');
                link.href = `/api/logs/download/${filename}`;
                link.download = filename;
                link.click();
            }
            
            // Update every 500ms
            setInterval(updateDashboard, 500);
            updateDashboard(); // Initial update
        </script>
    </body>
    </html>
    '''
    return render_template_string(html)

@app.route('/api/status')
def api_status():
    """API endpoint for dashboard status"""
    return jsonify({
        'armed': vehicle_state['armed'],
        'battery_voltage': vehicle_state['battery_voltage'],
        'battery_current': vehicle_state['battery_current'],
        'battery_remaining': vehicle_state['battery_remaining'],
        'is_logging': vehicle_state['is_logging'],
        'log_filename': vehicle_state['log_filename'],
        'system_status': vehicle_state['system_status'],
        'connected': vehicle_state['connected'],
        'flight_time': vehicle_state['flight_time'],
    })

@app.route('/api/logs/list')
def list_logs():
    """List available log files"""
    try:
        log_files = [f for f in os.listdir(LOG_DIR) if f.startswith('battery_log_')]
        log_files.sort(reverse=True)
        return jsonify({'logs': log_files})
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/logs/data/<filename>')
def get_log_data(filename):
    """Get parsed log data for visualization"""
    if '..' in filename or '/' in filename:
        return jsonify({'error': 'Invalid filename'}), 400
    
    data = read_log_file(filename)
    if data is None:
        return jsonify({'error': 'File not found or error reading file'}), 404
    
    return jsonify(data)

@app.route('/api/logs/download/<filename>')
def download_log(filename):
    """Download a specific log file"""
    if '..' in filename or '/' in filename:
        return jsonify({'error': 'Invalid filename'}), 400
    
    filepath = os.path.join(LOG_DIR, filename)
    if not os.path.exists(filepath):
        return jsonify({'error': 'File not found'}), 404
    
    try:
        with open(filepath, 'r') as f:
            content = f.read()
        return content, 200, {'Content-Disposition': f'attachment; filename={filename}'}
    except Exception as e:
        return jsonify({'error': str(e)}), 500

def main():
    """Main entry point"""
    print("=" * 60)
    print("MAVLink Battery Dashboard with Automatic Logging")
    print("=" * 60)
    
    # Connect to vehicle
    master = connect_to_vehicle()
    if not master:
        print("\n✗ Failed to connect to vehicle. Retrying in 5 seconds...")
        time.sleep(5)
        return main()
    
    # Start MAVLink receiver thread
    receiver_thread = threading.Thread(target=mavlink_receiver, args=(master,), daemon=True)
    receiver_thread.start()
    
    print("\n✓ Starting Flask dashboard on http://0.0.0.0:5001")
    print("  Battery logs will be saved to: ./battery_logs/")
    print("  Logging starts automatically when drone is armed")
    print("=" * 60 + "\n")
    
    # Start Flask app
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✓ Dashboard stopped by user")
        stop_battery_logging()
