# Optical Flow Drone Navigation & Multi-Sensor Attitude Fusion
## Implementation of Real-time Drift-free Odometry & Sensor Fusion for GPS-denied Flight Operations

---

## 1. Title Slide
* **Project Title:** Autonomous Drone Optical Flow Navigation & Multi-Sensor Attitude Fusion
* **Sub-title:** Real-time Drift-free Odometry & Sensor Fusion for GPS-denied Flight Operations
* **Target Hardware:** Raspberry Pi + Picamera2 + GY-86 IMU + ArduPilot/MAVLink
* **Scope:** Graduation Thesis (Skripsi) Research & Implementation

---

## 2. Project Background & Motivation
* **GPS-Denied Environments:** Drone operation in indoor warehouses, tunnels, and deep urban canyons where GPS signals are unavailable.
* **State Estimation Needs:** Real-time velocity and position estimations are required to enable flight control modes like *Loiter* or *Position Hold*.
* **Low-Cost Edge Computing:** Running dense computer vision and sensor fusion on a lightweight companion computer (Raspberry Pi).
* **The Tilt Distortion Challenge:** Drone tilting changes the camera view, generating false optical flow vectors that do not correspond to actual translation.

---

## 3. System Architecture Overview
### Hardware Layer
* **IMU Sensor:** GY-86 (combining MPU6050 Accelerometer/Gyroscope and HMC5883L Magnetometer) on the I2C bus.
* **Camera:** Raspberry Pi Camera Module v3 (Picamera2) capturing 60 FPS video.
* **Flight Controller:** ArduPilot FC connected via telemetry serial link.

### Software Layer
* `sensor_readers.py`: Multithreaded I2C reader and complementary filter processor.
* `optical_flow_stream.py`: Real-time tracking pipeline with camera tilt and offset compensation.
* `shared_memory`: Ultra-low latency IPC for sharing sensor values across scripts.

---

## 4. Hardware Interfaces & Sensor Specs
* **MPU6050 (Gyro & Accel):**
  * Interface: I2C (0x68)
  * Sample Rate: 50 Hz (20ms loop)
  * Purpose: Captures rotational velocities and linear accelerations.
* **HMC5883L (Magnetometer):**
  * Interface: I2C (0x1E)
  * Sample Rate: 10 Hz (100ms loop)
  * Purpose: Provides absolute Yaw heading reference.
* **Altimeter (Distance Sensor):**
  * Interface: MAVLink Telemetry
  * Purpose: Measures height above ground to scale pixels to physical distances.
* **Picamera2:**
  * Interface: CSI-2 port
  * Capture: 60 FPS (16.6ms intervals)
  * Purpose: Motion vectors estimation.

---

## 5. Sensor Reading & Multithreading
* **Decoupled execution loops** avoid blocking camera frame capture:
  * **`imu_reader` Thread (50Hz):** Integrated gyroscope angles, applies the Complementary Filter, and updates attitude states.
  * **`compass_reader` Thread (10Hz):** Periodically polls the HMC5883L magnetometer for heading correction.
  * **`mavlink_reader` Thread:** Monitors incoming MAVLink telemetry for distance sensor inputs.
* **Thread Synchronization:**
  * Safe updates of shared states using `attitude_lock`, `compass_lock`, and `distance_lock`.

---

## 6. Mathematical Model of Optical Flow
* Downward-facing camera displacement translates to horizontal velocity:
  
  $$V_x^{body} = \frac{\Delta x \cdot Z}{f \cdot \Delta t}$$
  
  $$V_y^{body} = -\frac{\Delta y \cdot Z}{f \cdot \Delta t}$$

* **Parameters:**
  * $Z$: Altitude above ground plane (from lidar/altimeter).
  * $f$: Camera focal length in pixels:
    
    $$f = \frac{\text{Width}}{2 \cdot \tan(\text{FOV}/2)}$$
    
  * $\Delta x, \Delta y$: Compensated horizontal pixel shifts.

---

## 7. The Attitude Drift Problem
* **Gyroscope Drift:**
  * Integrating gyroscope rates over time accumulates bias and noise:
    
    $$\theta_t = \theta_{t-1} + \omega \cdot \Delta t$$
    
  * Standard MEMS gyroscopes drift away from the true angle within seconds.
* **Accelerometer Noise:**
  * Direct tilt estimation from accelerometers is highly corrupted by motor vibrations.
* **Solution:** Sensor fusion to combine the fast reaction of the gyroscope with the absolute accuracy of the accelerometer/magnetometer.

---

## 8. Complementary Filter Theory
* Fuses the high-frequency response of the gyroscope with low-frequency absolute sensors:

  $$\text{Angle}_{fused} = \alpha \cdot (\text{Angle}_{prev} + \text{Gyro}_{rate} \cdot dt) + (1 - \alpha) \cdot \text{Angle}_{ref}$$

* **Mixing Factor ($\alpha = 0.96$):**
  * $96\%$ weight on gyroscope integration (smooth, rapid updates, immune to vibration).
  * $4\%$ weight on accelerometer/compass references (forces alignment, eliminating drift).

---

## 9. Roll & Pitch Estimation Implementation
* Code snippet showing integration and blending in `sensor_readers.py`:

```python
# Rotate and integrate gyroscope readings
roll_gyro_deg = attitude_state["roll_deg"] + (xgyro_dps * dt)
pitch_gyro_deg = attitude_state["pitch_deg"] + (ygyro_dps * dt)

# Apply Complementary Filter
attitude_state["roll_deg"] = normalize_angle_deg(
    (COMPLEMENTARY_FILTER_ALPHA * roll_gyro_deg)
    + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel_deg)
)
attitude_state["pitch_deg"] = normalize_angle_deg(
    (COMPLEMENTARY_FILTER_ALPHA * pitch_gyro_deg)
    + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel_deg)
)
```

---

## 10. Yaw Estimation & Magnetometer Fusion
* Fusing Z-axis gyroscope with HMC5883L absolute compass heading:

```python
# Integrate Z-Gyro yaw rate
yaw_gyro_deg = attitude_state["yaw_deg"] + (zgyro_dps * dt)

# Fuse with compass heading (if recent and valid)
if compass_heading_deg is not None and compass_age_s <= COMPASS_FRESHNESS_THRESHOLD_S:
    attitude_state["yaw_deg"] = blend_angle_deg(
        yaw_gyro_deg,
        compass_heading_deg,
        1.0 - COMPLEMENTARY_FILTER_ALPHA, # 0.04 weight to absolute heading
    )
else:
    attitude_state["yaw_deg"] = normalize_angle_deg(yaw_gyro_deg)
```

---

## 11. Standalone Gyroscope Calibration
* **Static Calibration (`calibrate_gyro_standalone.py`):**
  * Collects 300 samples when the drone is stationary.
  * Calculates static average offsets ($X, Y, Z$) to subtract during live telemetry reading.
  * Eliminates constant angular rate biases that cause fast attitude drift.

---

## 12. Tilt Compensation in Optical Flow
* Camera rotations shift the view, introducing false optical flow translation.
* **Correction Formula:**
  
  $$\Delta x_{compensated} = \Delta x_{raw} - (scale\_x \cdot \Delta \text{Roll}_{reticle})$$
  
  $$\Delta y_{compensated} = \Delta y_{raw} - (scale\_y \cdot \Delta \text{Pitch}_{reticle})$$

* Subtraction of rotational flow isolates pure horizontal translation.

---

## 13. Calibration of Tilt Scale Factors
* Dynamically calibrates `scale_x` and `scale_y` (mapping degrees to pixels):
  * **Calibration Mode:** Initiated by sending `c` to the console.
  * **Calibration Movement:** Tilt and roll the drone without moving it horizontally.
  * **Calculations:**
    
    $$scale\_x = \text{median}\left(\frac{tx}{d\_reticle\_x}\right), \quad scale\_y = \text{median}\left(\frac{ty}{d\_reticle\_y}\right)$$
    
  * Saved to `tilt_calibration.json` for persistence.

---

## 14. Camera Offset Compensation
* Compensates for camera placement offset from the drone's center of gravity (CoG):

```python
# Calculate translational velocities induced by yaw spinning
yaw_rate_rad = math.radians(zgyro_dps)
v_offset_x = -yaw_rate_rad * (camera_offset_y / 100.0)
v_offset_y = yaw_rate_rad * (camera_offset_x / 100.0)

# Subtract offsets from body-frame velocities
vx_mps_body_comp = vx_mps_body - v_offset_x
vy_mps_body_comp = vy_mps_body - v_offset_y
```

---

## 15. Flow-to-Velocity Conversion
* Steps to compute dynamic velocity:
  1. Compute raw body velocity based on altitude ($Z$) and camera focal length ($f$).
  2. Subtract center of gravity offsets.
  3. Rotate velocities into absolute coordinates (East/North) using compass Yaw:
     
     $$V_{east} = V_x^{body} \cos(\Psi) + V_y^{body} \sin(\Psi)$$
     
     $$V_{north} = -V_x^{body} \sin(\Psi) + V_y^{body} \cos(\Psi)$$
     
  4. Apply acceleration clamp ($15.0 \text{ m/s}^2$ limit) to filter anomalous spikes.

---

## 16. Position Tracking & Integration
* Horizontal velocities are integrated to update absolute coordinates:

```python
# Accumulate distance in centimeters
with position_lock:
    position_state["x_cm"] += vx_mps * dt_s * 100.0
    position_state["y_cm"] += vy_mps * dt_s * 100.0
    
    # Store path point for trajectory rendering
    position_state["path"].append((position_state["x_cm"], position_state["y_cm"]))
```

---

## 17. MAVLink Integration
* **Autopilot Communication Details:**
  * Uses `pymavlink` to read flight status, distance sensor values, and heartbeats.
  * **`static_gps_injector.py`:** Feeds static GPS messages into the Flight Controller to bypass Arming checks during indoor tests.
  * **Mode Switcher:** Automation scripts to switch the drone state to GUIDED, LOITER, or trigger reboot over Telemetry.

---

## 18. Shared Memory Architecture (SHM)
* **High-Rate Data Pipeline:**
  * Optical flow tracker runs at 60 FPS. Logger and Web frontend read this asynchronously.
  * Uses a Python `SharedMemory` block to prevent socket serialization delays.
  * **Buffer layout:** Magic Header + Write Index + Sample Count + Circular array of 120 slots.
  * High-frequency performance with zero interprocess memory copies.

---

## 19. Web Dashboard & Real-Time Frontend
* **Visual Telemetry Portal:**
  * Streams raw camera feeds alongside overlayed HUD elements (horizon line, reticle, vector flow arrows).
  * Web charts plot live roll, pitch, compass heading, altimeter, and absolute coordinates.
  * Flask background worker continuously reads state variables via Shared Memory.

---

## 20. Standalone Gyro Calibrator Tool
* **Executable Tool (`calibrate_gyro_standalone.py`):**
  * Allows developers to verify and calibrate the MPU6050 separately from the tracker pipeline.
  * Tests sensor noise variance under different conditions.
  * Outputs results to `gyro_calibration.json`.

---

## 21. Experimental Setup & Test Methodology
* **Telemetry Data Log Validation:**
  * Test data collected using `collect_sensor_data.py` during simulated maneuvers.
  * CSV logs record raw gyroscope angles, raw accelerometer/compass data, and complementary filter outputs.
  * Evaluation script `analyze_sensor_data.py` calculates statistical error rates (RMSE) and gyroscopic drift velocity.

---

## 22. Results: Gyroscope Calibration Analysis
* **Biases Eliminated:**
  * Static calibration offsets effectively eliminate linear angle drift.
  * Noise profile fits a tight Gaussian distribution.
  * Verifies sensor behavior meets low-drift specifications before flight testing.

---

## 23. Results: Roll & Pitch Attitude Stability
* **Statistical Performance (20s flight log analysis):**
  * **Roll Gyro Drift Rate:** $0.1270^\circ/\text{s}$ (Accumulated $+2.5386^\circ$ in 20 seconds).
  * **Pitch Gyro Drift Rate:** $0.0218^\circ/\text{s}$ (Accumulated $+0.4369^\circ$ in 20 seconds).
  * **Roll Filter RMSE:** $0.9035^\circ$ (relative to gravity vector).
  * **Pitch Filter RMSE:** $1.1655^\circ$.
* **Impact:** The complementary filter restricts errors to $<1.2^\circ$ with zero cumulative drift.

---

## 24. Results: Yaw Stability & Magnetometer Alignment
* **Compass Fusion Performance:**
  * **Yaw Gyro Drift Rate:** $0.0639^\circ/\text{s}$ (Accumulated $-1.2776^\circ$ in 20 seconds).
  * **Fused Yaw RMSE:** $0.2532^\circ$ (compared to Magnetometer reference).
* **Impact:** Integrates Z-gyro dynamic rates while fixing long-term alignment to the HMC5883L heading, preventing navigation coordinate rotation.

---

## 25. Conclusion & Future Work
* **Key Achievements:**
  * Steady attitude fusion utilizing a low-latency 50Hz Complementary Filter.
  * Reliable tilt and camera offset compensation pipelines, isolating drone translational velocities.
  * Sub-millisecond data sharing via shared memory circular buffers.
* **Future Works:**
  * Upgrade to an EKF (Extended Kalman Filter) for non-linear state estimation.
  * Integrate deep neural networks for feature extraction in low-texture settings.
  * Full autonomous waypoint navigation using MAVLink commands.
