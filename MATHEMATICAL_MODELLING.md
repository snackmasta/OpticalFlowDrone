# Mathematical Modelling of Optical Flow & Sensor Fusion

This document provides a comprehensive description of the mathematical models, coordinate transformations, and sensor fusion algorithms implemented in this project, alongside their corresponding code implementations.

---

## 1. System Coordinates & Reference Frames
To track the drone's states, we define two primary right-handed coordinate frames:

1. **Body Frame ($X_B, Y_B, Z_B$)**:
   * Attached to the drone's center of gravity (CoG).
   * $X_B$ points forward (Roll axis).
   * $Y_B$ points right (Pitch axis).
   * $Z_B$ points downward (Yaw axis).
2. **Earth/Global Frame ($X_E, Y_E, Z_E$)**:
   * Fixed local tangent plane (East-North-Up / East-North-Down variant).
   * $X_E$ points East.
   * $Y_E$ points North.
   * $Z_E$ points upwards (altitude).

---

## 2. Attitude Estimation & Sensor Fusion Model
Attitude estimation computes Roll ($\phi$), Pitch ($\theta$), and Yaw ($\psi$) to represent the drone's orientation.

### 2.1. Accelerometer Tilt Model
Under static or quasi-static conditions, the accelerometer measures the gravity vector $\mathbf{g} = [0, 0, 1]^T \text{ g}$ projected onto the body axes:

$$a_x = -g \sin\theta$$
$$a_y = g \sin\phi \cos\theta$$
$$a_z = g \cos\phi \cos\theta$$

Inverting these equations yields the accelerometer-derived attitude:

$$\phi_{\text{accel}} = \arctan2(a_y, a_z)$$
$$\theta_{\text{accel}} = \arctan2\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$

#### 💻 Code Implementation
In [`optical_flow/sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L114-L125):
```python
def accel_to_roll_pitch(ax_g, ay_g, az_g):
    magnitude = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
    if magnitude < 0.1:
        return 0.0, 0.0

    ax_g /= magnitude
    ay_g /= magnitude
    az_g /= magnitude

    roll_deg = math.degrees(math.atan2(ay_g, az_g))
    pitch_deg = math.degrees(math.atan2(-ax_g, math.sqrt((ay_g * ay_g) + (az_g * az_g))))
    return normalize_angle_deg(roll_deg), normalize_angle_deg(pitch_deg)
```

---

### 2.2. Gyroscope Angular Rate Integration & Complementary Filter Fusion
Gyroscopes measure body angular rates ($\omega_x, \omega_y, \omega_z$):

$$\phi_{\text{gyro}, t} = \phi_{t-1} + \omega_x \cdot dt$$
$$\theta_{\text{gyro}, t} = \theta_{t-1} + \omega_y \cdot dt$$

To eliminate both high-frequency accelerometer noise (vibrations) and low-frequency gyroscope drift, a complementary filter is applied:

$$\phi_t = \alpha \cdot (\phi_{t-1} + \omega_x \cdot dt) + (1 - \alpha) \cdot \phi_{\text{accel}}$$
$$\theta_t = \alpha \cdot (\theta_{t-1} + \omega_y \cdot dt) + (1 - \alpha) \cdot \theta_{\text{accel}}$$

Where $\alpha \in [0, 1]$ is the mixing factor (defined as `COMPLEMENTARY_FILTER_ALPHA = 0.98`).

#### 💻 Code Implementation
In [`optical_flow/sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L338-L356):
```python
# Integrate angular velocities over time step dt
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

### 2.3. Magnetometer Yaw Fusion
The absolute heading $\psi_{\text{compass}}$ is computed from the magnetometer readings ($m_x, m_y$) and fused with the integrated Z-gyro rate:

$$\psi_t = \alpha \cdot (\psi_{t-1} + \omega_z \cdot dt) + (1 - \alpha) \cdot \psi_{\text{compass}}$$

#### 💻 Code Implementation
In [`optical_flow/sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L334-L364):
```python
yaw_gyro_deg = attitude_state["yaw_deg"] + (zgyro_dps * dt)

if compass_heading_deg is not None and compass_age_s <= COMPASS_FRESHNESS_THRESHOLD_S:
    attitude_state["yaw_deg"] = blend_angle_deg(
        yaw_gyro_deg,
        compass_heading_deg,
        1.0 - COMPLEMENTARY_FILTER_ALPHA,
    )
else:
    attitude_state["yaw_deg"] = normalize_angle_deg(yaw_gyro_deg)
```

---

## 3. Optical Flow Kinematic & Camera Model

### 3.1. Camera Focal Length Calculation
The pixel-equivalent focal length $f_x$ depends on the camera frame width ($W$) and horizontal Field of View ($\text{FOV}$):

$$f_x = \frac{W}{2 \tan\left(\frac{\text{FOV}_h}{2}\right)}$$

#### 💻 Code Implementation
In [`optical_flow/flow_processor.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/flow_processor.py#L112-L113):
```python
def focal_length_px(frame_width):
    return frame_width / (2.0 * math.tan(math.radians(CAMERA_HORIZONTAL_FOV_DEG / 2.0)))
```

---

### 3.2. Flow-to-Velocity Conversion (Translational Flow Only)
Assuming the ground plane is flat and parallel to the camera sensor, a translational displacement of pixels $(tx, ty)$ over time $dt$ at an altitude $Z$ corresponds to physical body velocities:

$$v_{x, \text{body}} = \frac{tx \cdot Z}{f_x \cdot dt}$$

$$v_{y, \text{body}} = -\frac{ty \cdot Z}{f_y \cdot dt}$$

#### 💻 Code Implementation
In [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L423-L428):
```python
with distance_lock:
    altitude_cm = distance_state["current_distance"]

altitude_m = (altitude_cm / 100.0) if altitude_cm is not None else 1.5
vx_mps_body = ((tx_comp * altitude_m) / (focal_length_x_px * dt_s))
vy_mps_body = -((ty_comp * altitude_m) / (focal_length_y_px * dt_s))
```

---

## 4. Tilt Compensation Model
When the camera rotates, the image plane shifts by $(t_{x, \text{rot}}, t_{y, \text{rot}})$ which must be subtracted from the total flow measurements $(tx, ty)$ to isolate translation:

$$tx_{\text{comp}} = tx - t_{x, \text{rot}}$$
$$ty_{\text{comp}} = ty - t_{y, \text{rot}}$$

The rotational component is modeled as:

$$t_{x, \text{rot}} = scale_x \cdot (roll_t - roll_{t-1}) \cdot S_{\phi}$$
$$t_{y, \text{rot}} = scale_y \cdot -(pitch_t - pitch_{t-1}) \cdot S_{\theta}$$

Where $S_{\phi}, S_{\theta}$ are constant pixels-per-degree display reticle scales (`RETICLE_ROLL_SCALE_PX_PER_DEG = 4.5`, `RETICLE_PITCH_SCALE_PX_PER_DEG = 6.0`).

#### 💻 Code Implementation
In [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L389-L420):
```python
# Map roll/pitch angles to reticle displacements
roll_px = np.clip(roll_deg * RETICLE_ROLL_SCALE_PX_PER_DEG, -frame_width * 0.35, frame_width * 0.35)
pitch_px = np.clip(-pitch_deg * RETICLE_PITCH_SCALE_PX_PER_DEG, -frame_height * 0.35, frame_height * 0.35)

d_reticle_x = roll_px - prev_roll_px
d_reticle_y = pitch_px - prev_pitch_px

# Subtract expected rotation from raw optical flow vectors (tx, ty)
tx_comp = tx - (scale_x * d_reticle_x)
ty_comp = ty - (scale_y * d_reticle_y)
```

---

## 5. Camera Offset & Lever-Arm Compensation
If the camera is mounted at an offset vector $\mathbf{r}_{\text{cam}} = [x_{\text{off}}, y_{\text{off}}, z_{\text{off}}]^T$ away from the drone's center of gravity (CoG), yaw rotations ($\omega_z$) generate a linear velocity at the camera sensor:

$$\mathbf{v}_{\text{lever}} = \boldsymbol{\omega} \times \mathbf{r}_{\text{cam}}$$

We subtract this offset to find the velocity of the CoG:

$$v_{offset, x} = -\omega_z \cdot y_{\text{off}}$$
$$v_{offset, y} = \omega_z \cdot x_{\text{off}}$$

#### 💻 Code Implementation
In [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L430-L435):
```python
# Compensate for camera offset from center of rotation
yaw_rate_rad = math.radians(zgyro_dps)
v_offset_x = -yaw_rate_rad * (camera_offset_y / 100.0)
v_offset_y = yaw_rate_rad * (camera_offset_x / 100.0)
vx_mps_body_comp = vx_mps_body - v_offset_x
vy_mps_body_comp = vy_mps_body - v_offset_y
```

---

## 6. Global Frame Coordinate Transformation
To find the velocities in the Earth-fixed frame ($V_{\text{east}}, V_{\text{north}}$), the body velocities are rotated through the yaw angle $\psi$:

$$\begin{bmatrix} V_{\text{east}} \\ V_{\text{north}} \end{bmatrix} = \begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} v_{x, \text{body, comp}} \\ v_{y, \text{body, comp}} \end{bmatrix}$$

Expanding this matrix multiplication yields:

$$V_{\text{east}} = v_{x, \text{body, comp}} \cos\psi + v_{y, \text{body, comp}} \sin\psi$$
$$V_{\text{north}} = -v_{x, \text{body, comp}} \sin\psi + v_{y, \text{body, comp}} \cos\psi$$

#### 💻 Code Implementation
In [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L437-L443):
```python
# Rotate body frame velocities to East/North Earth frame
yaw_actual_deg = -yaw_deg
yaw_rad = math.radians(yaw_actual_deg)
cos_yaw = math.cos(yaw_rad)
sin_yaw = math.sin(yaw_rad)

vx_mps_calc = vx_mps_body_comp * cos_yaw + vy_mps_body_comp * sin_yaw
vy_mps_calc = -vx_mps_body_comp * sin_yaw + vy_mps_body_comp * cos_yaw
```

---

## 7. Position Integration (Dead Reckoning)
Absolute position $(X_E, Y_E)$ is estimated by accumulating the velocity updates over time:

$$X_{E, t} = X_{E, t-1} + V_{\text{east}} \cdot dt$$
$$Y_{E, t} = Y_{E, t-1} + V_{\text{north}} \cdot dt$$

#### 💻 Code Implementation
In [`optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L487-L490):
```python
with position_lock:
    position_state["x_cm"] += vx_mps * dt_s * 100.0
    position_state["y_cm"] += vy_mps * dt_s * 100.0
```
