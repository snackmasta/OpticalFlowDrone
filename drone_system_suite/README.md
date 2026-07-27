# Drone Optical Flow & Sensor Fusion Integrated Suite

This self-contained folder provides a process architecture for the 3D Optical Flow Geofence and Drone Telemetry system.

## Folder Structure

```
drone_system_suite/
├── main.py                     # Master launcher and process supervisor
├── web_server.py               # HTTP/SSE Telemetry Dashboard & REST API server (Port 8000)
├── send_attitude_udp.py        # Madgwick AHRS, Sensor Fusion & Telemetry UDP broadcaster (Port 5005)
├── geofence_buzzer_listener.py # Hardware GPIO PWM Alarm listener (Port 5006)
├── optical_flow_stream.py      # Camera optical flow tracker & shared memory publisher
├── sensor_fusion.py            # Multi-Sensor Fusion Engine (IMU, Optical Flow, GPS)
├── madgwick_ahrs.py            # Madgwick orientation & 3D position estimator
├── web/                        # Web dashboard frontend static files
└── optical_flow/               # Submodules for camera & flow computation
```

## Quick Start

To run the entire system with process monitoring and auto-restart:

```bash
python main.py
```

### Options

- **Target Specific Telemetry IP** (default: `127.0.0.1`):
  ```bash
  python main.py --ip 192.168.137.1
  ```

- **Run without Camera / Hardware (Testing mode)**:
  ```bash
  python main.py --no-flow
  ```

## Individual Service Execution

If you prefer to run any of the components independently:

1. **Web Server & Dashboard**:
   ```bash
   python web_server.py
   ```
   Open `http://localhost:8000` in your web browser.

2. **Telemetry Broadcaster**:
   ```bash
   python send_attitude_udp.py --ip 127.0.0.1 --port 5005 --rate 50
   ```

3. **Geofence Alarm Listener**:
   ```bash
   python geofence_buzzer_listener.py --port 5006
   ```

4. **Optical Flow Stream**:
   ```bash
   python optical_flow_stream.py -stream
   ```
