# Network Topology and Port Mapping

This document details the network interfaces, port mappings, protocol structures, and wiring interfaces that facilitate communication between the companion computer, flight controller (autopilot), and ground control station (GCS).

---

## 1. Network Interfaces

The Companion Computer (Raspberry Pi) manages communication across three distinct interface layers:

```
+-------------------------------------------------------------------------------+
|                      Companion Computer (Raspberry Pi)                        |
|                                                                               |
|   +-------------------+     +-------------------+     +-------------------+   |
|   |   Loopback (lo)   |     |   Wireless (wlan0)|     |   Physical Serial |   |
|   |   127.0.0.1       |     |   192.168.X.X     |     |   UART Pins       |   |
|   +---------+---------+     +---------+---------+     +---------+---------+   |
|             |                         |                         |             |
+-------------v-------------------------v-------------------------v-------------+
              |                         |                         |
    [Internal IPC & Sockets]    [GCS / Web Client]       [Flight Controller]
```

1. **Loopback Interface (`lo` / `127.0.0.1`)**: Used for high-frequency internal telemetry routing between local scripts and proxies.
2. **Wireless Interface (`wlan0`)**: Used to connect to the Ground Control Station (GCS) and stream video feeds/dashboards.
3. **Physical Serial Interfaces (UART)**: Hardware level serial connections routed directly to autopilot telemetry and GPS ports.

---

## 2. Port Mappings and Services

Below is the complete port configuration registry utilized in the companion computer network architecture:

| Port | Protocol | Interface | Source / Server | Target / Destination | Purpose |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **5001** | HTTP | `0.0.0.0` (All) | `battery_dashboard.py` | Web Browser / GCS | Flask Battery Web UI |
| **5003** | HTTP | `0.0.0.0` (All) | `read_shared_memory.py` | Web Browser / GCS | Flask Shared Memory Dashboard Web UI |
| **5009** | UDP | `127.0.0.1` | Flask Web Server | `optical_flow_stream.py` | Local API commands (Reset / Offset) |
| **14550**| UDP | `127.0.0.1` | MAVProxy | `battery_dashboard.py` | MAVLink telemetry routing |
| **14551**| UDP | `127.0.0.1` | MAVProxy | `optical_flow_stream.py` | Rangefinder distance data routing |
| **14552**| UDP | `0.0.0.0` (All) | MAVProxy | Ground Control Station | Telemetry link for QGroundControl/Mission Planner |
| **8554** | RTSP | `0.0.0.0` (All) | `mediamtx` | Web Video Client / GCS | Processed camera stream H.264 broadcast |

---

## 3. Physical Serial Configurations

Hardware level links interface directly with the flight controller's telemetry and GPS processor:

```
+-----------------------------------+             +----------------------------------+
|         Raspberry Pi 5            |             |     Pixhawk Flight Controller    |
|                                   |             |                                  |
|  GPIO 14/15 (/dev/serial0)        |------------>| Telemetry 1 Port (921600 baud)   |
|  (MAVLink Ingestion)              |             |                                  |
|                                   |             |                                  |
|  GPIO 8/9 (/dev/ttyAMA2)           |------------>| GPS 1 / GPS 2 Port (38400 baud)  |
|  (NMEA GPS Injection)             |             |                                  |
+-----------------------------------+             +----------------------------------+
```

### A. MAVLink Interface (`/dev/serial0`)
* **Hardware Pins:** GPIO 14 (TXD0) & GPIO 15 (RXD0).
* **Speed:** 921600 Baud.
* **Flow Control:** None (8N1).
* **Protocol:** MAVLink v2.

### B. GPS Injector Interface (`/dev/ttyAMA2`)
* **Hardware Pins:** GPIO 8 (TXD2) & GPIO 9 (RXD2).
* **Speed:** 38400 Baud.
* **Flow Control:** None (8N1).
* **Protocol:** NMEA 0183.

---

## 4. Telemetry Distribution (MAVProxy Orchestration)

To prevent multiple scripts from competing for access to the single physical serial port `/dev/serial0`, the companion computer uses **MAVProxy** as an active multiplexer:

1. **Input Link**: MAVProxy reads the raw serial stream from `/dev/serial0` (921600 baud).
2. **Local Distribution**:
   * Forwards a UDP stream to `127.0.0.1:14550` (consumed by the battery monitoring system).
   * Forwards a UDP stream to `127.0.0.1:14551` (consumed by the optical flow system to capture distance data).
3. **External Distribution**:
   * Broadcasts telemetry out over `wlan0` via UDP to port `14552` to link with wireless Ground Control Stations.
