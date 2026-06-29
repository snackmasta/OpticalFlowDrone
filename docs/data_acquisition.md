# Data Acquisition Architecture

This document describes how the **Optical Flow Drone** project acquires sensor measurements and telemetry from physical hardware interfaces, external processes, and flight controllers.

---

## 1. Camera Frame Capture (Picamera2 API)

* **Implementation:** [optical_flow_stream.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py)
* **Hardware:** Raspberry Pi Camera Module (connected via CSI ribbon cable)
* **Details:**
  * Uses the modern `Picamera2` Python wrapper to capture raw BGR image arrays.
  * Captures at a target resolution of **640x480** at **30 FPS**.
  * Threading is optimized by assigning OpenCV's thread count to match the number of available CPU cores:
    ```python
    cv2.setNumThreads(cv2.getNumberOfCPUs())
    ```
  * Frames are converted from 4-channel BGRA to 3-channel BGR (using `ensure_bgr`) and then downscaled/converted to grayscale (using `to_small_gray`) before being pushed into the optical flow motion estimation pipeline.

---

## 2. IMU Reading (MPU6050 via I2C)

* **Implementation:** [sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py)
* **Hardware:** MPU6050 6-Axis MotionTracking device (connected via Raspberry Pi I2C pins)
* **Details:**
  * Communicates over **I2C Bus 1** at address **0x68**.
  * The driver sets up register `0x6B` (PWR_MGMT_1) to wake up the sensor on startup.
  * Reads the raw 16-bit high/low bytes for:
    * Accelerometers: `ACCEL_XOUT`, `ACCEL_YOUT`, `ACCEL_ZOUT` (scaled by $16384.0 \text{ LSB/g}$)
    * Gyroscopes: `GYRO_XOUT`, `GYRO_YOUT`, `GYRO_ZOUT` (scaled by $131.0 \text{ LSB/dps}$)
  * **Calibration**: During initialization, the system calibrates the gyroscopes by averaging 100 samples while the drone is stationary to compute gyro biases, which are then subtracted from all subsequent readings.

---

## 3. Rangefinder Altitude (MAVLink DISTANCE_SENSOR)

* **Implementation:** [sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py)
* **Connection:** UDP loopback (`udp:127.0.0.1:14551`) from MAVProxy/SITL
* **Details:**
  * Runs in a dedicated background daemon thread (`mavlink_reader`).
  * Listens for incoming MAVLink packets using `pymavlink`.
  * Specifically parses `DISTANCE_SENSOR` packets to extract the `current_distance` attribute (expressed in centimeters). This altitude is updated asynchronously and protected by a thread lock (`distance_lock`) to be consumed by the optical flow velocity scaling process.

---

## 4. Compass Heading (Shared Memory IPC)

* **Implementation:** [sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py)
* **Details:**
  * Runs in a dedicated background thread (`compass_reader_thread`).
  * Instead of communicating directly with a serial compass, the system reads heading data from a global Inter-Process Communication (IPC) shared memory segment named `compass_heading_stream`.
  * Reads are structure-unpacked at **50 Hz** using the format `<6d` (timestamp, raw_heading, heading, x, y, z).
  * Exposes the heading to the complementary filter for orientation tracking and GPS coordinate generation.

---

## 5. Battery and Vehicle State (MAVLink Telemetry)

* **Implementation:** [battery_dashboard.py](file:///e:/OptFlowDrone/OpticalFlowDrone/scripts/battery_dashboard.py)
* **Connection:** UDP port `udp:127.0.0.1:14550` or serial port `/dev/serial0` (at 921600 baud)
* **Details:**
  * Establishes a MAVLink connection to the autopilot.
  * Asynchronously parses:
    * `HEARTBEAT` messages: Extracts the arming status (`base_mode & MAV_MODE_FLAG_SAFETY_ARMED`) and system status.
    * `BATTERY_STATUS` messages: Extracts the battery voltage (converted to Volts), current draw (converted to Amperes), and remaining charge percentage.
  * These values are populated into a globally accessible dictionary (`vehicle_state`) used to drive the Flask monitoring dashboard and trigger flight telemetry CSV log creation.
