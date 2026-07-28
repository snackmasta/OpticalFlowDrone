# Tilt Compensation in Optical Flow

This document explains the theory, calibration, and software implementation of **Tilt Compensation** within the `OpticalFlowDrone` codebase. 

---

## 1. The Core Challenge: Tilt-Induced Flow Distortion
When a multirotor drone translates (moves horizontally), the camera detects ground texture displacement (optical flow) which is proportional to the translational velocity:

$$\mathbf{v}_{\text{translation}} \propto \frac{\mathbf{d}_{\text{pixels}} \cdot z}{\text{focal\_length}}$$

However, when the drone tilts (rolls or pitches) to change direction or resist wind, the camera rotates. This rotation shifts the entire image frame across the sensor.
* The optical flow algorithm (Farneback or Lucas-Kanade) detects this rotation-induced pixel displacement as translation.
* Without correction, a drone hovering in place that tilts to correct its position will measure a false translation velocity, leading to incorrect velocity estimation, positive feedback loops, and severe navigation drift.

```
       NO COMPENSATION                    WITH TILT COMPENSATION
     
       [ Drone Tilts ]                       [ Drone Tilts ]
             │                                     │
             ▼                                     ▼
     [ Camera Rotates ]                    [ Camera Rotates ]
             │                                     │
             ▼                                     ▼
   [ Image Shifts (tx, ty) ]             [ Image Shifts (tx, ty) ]
             │                                     │
             ▼                                     ├─► [Subtract expected tilt rotation]
    [ Treated as Velocity ]                        │          (scale * d_reticle)
             │                                     ▼
             ▼                            [ True translation isolated ]
    [ DRIFT & INSTABILITY ]                        │
                                                   ▼
                                         [ STABLE HOVER & ODOMETRY ]
```

---

## 2. Attitude Estimation (Sensor Fusion)
To compensate for tilt, the drone must continuously and accurately estimate its pitch and roll. In [`optical_flow/sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py), this is done using a **Complementary Filter** that fuses:
1. **Gyroscopes** (High-rate, low latency, but prone to integration drift).
2. **Accelerometers** (No drift, but highly corrupted by high-frequency motor vibrations).

The complementary fusion formula implemented in the code is:

$$\theta_{t} = \alpha \cdot (\theta_{t-1} + \omega_{\text{gyro}} \cdot dt) + (1 - \alpha) \cdot \theta_{\text{accel}}$$

Where:
* $\theta$ is the roll/pitch angle.
* $\alpha = 0.98$ (defined by `COMPLEMENTARY_FILTER_ALPHA`).
* $\theta_{\text{accel}}$ is derived from static gravity projection:
  $$\text{roll\_accel} = \arctan2(y_g, z_g)$$
  $$\text{pitch\_accel} = \arctan2(-x_g, \sqrt{y_g^2 + z_g^2})$$

---

## 3. Reticle Displacement Mapping
To convert physical angular attitude changes (degrees) into pixel displacement, the system maps the roll and pitch to a simulated HUD ground reticle in [`optical_flow/hud_renderer.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/hud_renderer.py):
* **Roll Scale Factor** (`RETICLE_ROLL_SCALE_PX_PER_DEG`): `4.5` pixels/degree
* **Pitch Scale Factor** (`RETICLE_PITCH_SCALE_PX_PER_DEG`): `6.0` pixels/degree

During each frame in [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py), the change in reticle position since the last frame is computed:

```python
# Convert degrees of attitude to pixel coordinates
roll_px = np.clip(roll_deg * RETICLE_ROLL_SCALE_PX_PER_DEG, -frame_width * 0.35, frame_width * 0.35)
pitch_px = np.clip(-pitch_deg * RETICLE_PITCH_SCALE_PX_PER_DEG, -frame_height * 0.35, frame_height * 0.35)

# Calculate delta displacement of reticle (expected tilt shift in pixels)
d_reticle_x = roll_px - prev_roll_px
d_reticle_y = pitch_px - prev_pitch_px
```

---

## 4. Calibration of Tilt Scale Factors
The mapping between reticle displacement ($d_{\text{reticle}}$) and actual image flow translation ($t$) is governed by scaling factors $scale_x$ and $scale_y$:

$$t_{\text{tilt}} = scale \cdot d_{\text{reticle}}$$

To calibrate these factors:
1. The user starts calibration mode by entering `'c'`.
2. The user tilts and rolls the drone/camera back and forth **without physically translating it**.
3. Since translation is zero, any measured flow translation ($tx, ty$) is purely tilt-induced rotation.
4. The system collects samples for frames with significant movement ($|d_{\text{reticle}}| > 1.5$ pixels):
   $$\text{sample}_x = \frac{tx}{d_{\text{reticle\_x}}}$$
   $$\text{sample}_y = \frac{ty}{d_{\text{reticle\_y}}}$$
5. Upon saving (`'s'`), the **median** of the collected samples is calculated to filter out outliers:
   ```python
   scale_x = float(np.median(calib_samples_x))
   scale_y = float(np.median(calib_samples_y))
   ```
6. The scales are persisted in `tilt_calibration.json` for subsequent flights.

---

## 5. Tilt Compensation & Velocity Processing Pipeline
Once calibrated, the real-time velocity estimation pipeline performs the following steps inside the main loop of [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py):

### Step 5.1: Subtract Tilt-Induced Flow
The expected rotation displacement is subtracted from the raw estimated dense flow translation ($tx, ty$):
```python
tx_comp = tx - (scale_x * d_reticle_x)
ty_comp = ty - (scale_y * d_reticle_y)
```

### Step 5.2: Convert to Physical Body Velocity
Using the current altitude (altitude_m) and camera focal length (pixels), the compensated pixel displacement is converted to physical body-frame velocity (meters per second):
```python
altitude_m = 1.5

vx_mps_body = ((tx_comp * altitude_m) / (focal_length_x_px * dt_s))
vy_mps_body = -((ty_comp * altitude_m) / (focal_length_y_px * dt_s))
```

### Step 5.3: Camera Offset Compensation (Lever-Arm Correction)
If the camera is not mounted exactly at the center of gravity/rotation of the drone, yaw rotations create secondary linear velocity offsets. This is corrected using:
```python
yaw_rate_rad = math.radians(zgyro_dps)
v_offset_x = -yaw_rate_rad * (camera_offset_y / 100.0)
v_offset_y = yaw_rate_rad * (camera_offset_x / 100.0)

vx_mps_body_comp = vx_mps_body - v_offset_x
vy_mps_body_comp = vy_mps_body - v_offset_y
```

### Step 5.4: Rotation to Global Earth Frame
Finally, the body velocities are rotated into the absolute world frame (East/North coordinate system) using the drone's compass heading ($\psi$):
```python
yaw_rad = math.radians(-yaw_deg)
cos_yaw = math.cos(yaw_rad)
sin_yaw = math.sin(yaw_rad)

vx_mps_calc = vx_mps_body_comp * cos_yaw + vy_mps_body_comp * sin_yaw
vy_mps_calc = -vx_mps_body_comp * sin_yaw + vy_mps_body_comp * cos_yaw
```

---

## Summary of Code References
* **Attitude / Sensor Fusion**: [`optical_flow/sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L305-L365)
* **HUD Reticle Scaling**: [`optical_flow/hud_renderer.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/hud_renderer.py#L17-L18)
* **Calibration State & Commands**: [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L317-L345)
* **Real-time Subtraction & Velocity Conversion**: [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L383-L444)
