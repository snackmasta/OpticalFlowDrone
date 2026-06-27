# MAVLink Battery Dashboard with Automatic Logging

A real-time web-based dashboard for monitoring drone battery status with automatic logging when the drone is armed.

## Features

✅ **Real-Time Battery Monitoring**
- Voltage display
- Current draw measurement
- Battery remaining percentage with visual indicator
- Color-coded battery level warnings (green → orange → red)

✅ **Automatic Flight Logging**
- Logging starts automatically when drone is armed
- Stops automatically when drone is disarmed
- CSV format with timestamps
- Columns: Timestamp, Voltage (V), Current (A), Remaining (%), Flight Time (s)

✅ **Web Dashboard**
- Beautiful, responsive UI
- Real-time updates every 500ms
- Connection status indicator
- Flight mode display
- Arm/Disarm status
- Logging status with current log filename

✅ **Log Management**
- Logs saved to `./battery_logs/` directory
- Timestamped filenames for organization
- Download capability for analysis

## Installation

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Verify serial connection:**
   ```bash
   ls -la /dev/serial0
   ```
   (Should show your autopilot connection)

## Usage

### Start the Dashboard

```bash
python3 battery_dashboard.py
```

**Output:**
```
============================================================
MAVLink Battery Dashboard with Automatic Logging
============================================================
Connecting to vehicle on /dev/serial0 at 921600 baud...
✓ Connected to vehicle!

✓ Starting Flask dashboard on http://0.0.0.0:5001
  Battery logs will be saved to: ./battery_logs/
  Logging starts automatically when drone is armed
============================================================
```

### Access the Dashboard

Open your browser and navigate to:
- **Local Network**: `http://<drone-ip>:5001`
- **Local Machine**: `http://localhost:5001`

### Workflow

1. **Dashboard displays real-time battery info:**
   - Voltage and current draw
   - Battery percentage with visual bar
   - Flight mode
   - Connection status

2. **When you arm the drone:**
   - Logging automatically starts
   - You'll see a confirmation in console: `✓ Drone ARMED - Starting battery log`
   - "Logging Active" status shows in dashboard
   - Current filename displayed

3. **During flight:**
   - Battery data logged every time a BATTERY_STATUS message is received (typically 1 Hz)
   - Dashboard updates in real-time

4. **When you disarm the drone:**
   - Logging automatically stops
   - You'll see: `✓ Drone DISARMED - Stopping battery log`
   - Log file is closed and saved
   - Dashboard shows "Logging Inactive"

### Log Files

Logs are saved to `./battery_logs/` with format: `battery_log_YYYYMMDD_HHMMSS.csv`

**Example log content:**
```
Timestamp,Voltage (V),Current (A),Remaining (%),Flight Time (s)
2026-04-26 15:30:45.123,12.45,5.32,95,0
2026-04-26 15:30:46.125,12.44,5.35,95,1
2026-04-26 15:30:47.128,12.43,5.30,94,2
2026-04-26 15:30:48.130,12.42,5.28,94,3
```

### Dashboard Status Indicators

| Indicator | Meaning |
|-----------|---------|
| 🟢 Connected | MAVLink connection active |
| 🔴 Disconnected | No connection to autopilot |
| 🟠 Armed | Drone armed, ready to fly |
| 🔵 Disarmed | Drone disarmed |
| 🟠 Logging Active | Battery data being recorded |
| 🔵 Logging Inactive | No active logging session |

### Battery Warning Levels

| Level | Color | Remaining |
|-------|-------|-----------|
| Good | 🟢 Green | > 40% |
| Warning | 🟠 Orange | 20-40% |
| Critical | 🔴 Red (Blinking) | < 20% |

## Configuration

Edit these variables in `battery_dashboard.py`:

```python
SERIAL_PORT = '/dev/serial0'      # Serial connection to autopilot
BAUD_RATE = 921600                # Communication speed
LOG_DIR = './battery_logs'        # Directory for log files
```

### Change Port or Baud Rate

If using different serial configuration:
- **TCP connection**: `master = mavutil.mavlink_connection('tcp:192.168.1.100:5760')`
- **Different baud**: `BAUD_RATE = 115200`

## Troubleshooting

### Dashboard won't connect

**Error**: `Connection failed: [Errno 13] Permission denied: '/dev/serial0'`

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or run with sudo
sudo python3 battery_dashboard.py
```

### No battery data showing

1. Check that drone's autopilot supports BATTERY_STATUS messages
2. Verify MAVLink connection is working:
   ```bash
   python3 gyro.py  # Should show gyro data
   ```

### Logging not starting on arm

1. Ensure `battery_logs` directory has write permissions:
   ```bash
   chmod 755 battery_logs/
   ```
2. Check console for arm/disarm messages
3. Verify autopilot is sending HEARTBEAT messages with armed state

### Can't access dashboard from network

1. Check firewall settings:
   ```bash
   sudo ufw allow 5001
   ```
2. Use the IP address instead of hostname:
   ```
   http://192.168.1.XX:5001
   ```

## API Endpoints

The dashboard also exposes REST APIs:

### Get Current Status
```bash
curl http://localhost:5001/api/status
```

**Response:**
```json
{
  "armed": false,
  "battery_voltage": 12.45,
  "battery_current": 0.0,
  "battery_remaining": 95,
  "is_logging": false,
  "log_filename": null,
  "system_status": "STABILIZE",
  "connected": true,
  "flight_time": 0
}
```

### List Log Files
```bash
curl http://localhost:5001/api/logs/list
```

### Download a Log
```bash
curl http://localhost:5001/api/logs/download/battery_log_20260426_153045.csv
```

## Data Analysis

You can use the CSV log files with Python, Excel, Matlab, or any data analysis tool:

**Python example:**
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv('battery_logs/battery_log_20260426_153045.csv')

# Plot battery voltage over time
plt.figure(figsize=(12, 5))
plt.subplot(1, 2, 1)
plt.plot(df['Flight Time (s)'], df['Voltage (V)'])
plt.title('Battery Voltage Over Flight Time')
plt.xlabel('Flight Time (s)')
plt.ylabel('Voltage (V)')
plt.grid()

plt.subplot(1, 2, 2)
plt.plot(df['Flight Time (s)'], df['Current (A)'])
plt.title('Current Draw Over Flight Time')
plt.xlabel('Flight Time (s)')
plt.ylabel('Current (A)')
plt.grid()

plt.tight_layout()
plt.show()
```

## Example Flight Session

```
15:30:40.000 - Dashboard starts, connects to drone
15:30:45.123 - Drone armed → Logging starts (battery_log_20260426_153045.csv)
15:30:46.000 - Drone takes off
15:30:50.000 - Flight in progress, data logging
15:31:30.000 - Still flying, battery at 92%
15:32:15.000 - Drone descends and lands
15:32:50.000 - Drone disarmed → Logging stops
15:32:51.000 - Log file saved with flight data
```

## Performance Notes

- **Update Rate**: Dashboard updates every 500ms
- **Log Frequency**: Battery data logged at BATTERY_STATUS message rate (typically 1 Hz)
- **Storage**: ~2-3 KB per minute of logged data
- **CPU Usage**: Minimal (~5% on Raspberry Pi)

## Integration with Other Scripts

You can run the dashboard alongside other monitoring scripts:

```bash
# Terminal 1
python3 battery_dashboard.py

# Terminal 2
python3 optical_flow_stream.py

# Terminal 3
python3 gyro.py
```

They all use independent MAVLink connections and won't interfere.

## Hardware Requirements

- Autopilot with MAVLink support (Pixhawk, APM, etc.)
- Serial connection (USB, UART)
- Python 3.6+
- Flask installed

## License

This script is provided as-is for drone monitoring and data logging purposes.

## Support

For issues or questions:
1. Check the troubleshooting section
2. Verify MAVLink connection with existing scripts (gyro.py, rangefinder.py)
3. Review console output for error messages
