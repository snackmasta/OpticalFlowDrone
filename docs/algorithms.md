# Core Algorithms Documentation

This document explains the core mathematical models and processing algorithms utilized in the **Optical Flow Drone** project. These algorithms handle the extraction of motion from camera video streams, filter out errors caused by vehicle attitude changes (tilt and rotation), and estimate absolute coordinates.

---

## 1. Visual Motion Estimation (Dense DIS Flow & Affine Fitting)

**File location:** [flow_processor.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/flow_processor.py)

Instead of sparse feature tracking (like Lucas-Kanade corner tracking), which is highly sensitive to lighting changes and textureless surfaces (such as grass or concrete), this system uses a dense flow framework:

### Dense Inverse Search (DIS) Flow
The system initializes the OpenCV Dense Inverse Search algorithm:
```python
dis_flow = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_ULTRAFAST)
```
* **Pre-processing**: Frames are scaled down to 50% (`FLOW_SCALE = 0.5`) and converted to grayscale to minimize CPU overhead.
* **Flow Field Calculation**: DIS computes a dense displacement vector field $\mathbf{f}(x, y) = [u(x, y), v(x, y)]^T$ for every single pixel between the previous frame $I_{t-1}$ and current frame $I_t$.

### Grid Sampling & RANSAC Affine Fitting
Computing motion using all pixels is too slow, so a grid sample approach is combined with RANSAC to calculate camera movement parameters:
1. **Grid Sampling**: The dense flow field is sampled on a uniform grid spacing (default `step = 8` pixels):
   $$P_{\text{old}} = (x_i, y_i) \quad \text{for } i \in \text{sampled grid}$$
   $$P_{\text{new}} = (x_i + u(x_i, y_i), \ y_i + v(x_i, y_i))$$
2. **Magnitude Filtering**: Displacements larger than `MAX_FLOW_STEP_PX = 80.0` are rejected immediately as invalid tracking noise.
3. **Affine Motion Fitting**: OpenCV's `estimateAffinePartial2D` fits a 4-degree-of-freedom partial affine transform (translation, rotation, uniform scale):
   $$\begin{bmatrix} x_{\text{new}} \\ y_{\text{new}} \end{bmatrix} = \begin{bmatrix} s \cos\theta & -s \sin\theta \\ s \sin\theta & s \cos\theta \end{bmatrix} \begin{bmatrix} x_{\text{old}} \\ y_{\text{old}} \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}$$
   * **RANSAC Filtering**: Points that do not agree with the dominant translation/rotation matrix within a reprojection error threshold of 2 pixels are rejected.
   * **Parameter Extraction**: From the calculated matrix $M$, the translation values ($t_x, t_y$), scaling ($s$), and rotation ($\theta$) are extracted:
     $$t_x = M_{0, 2}, \quad t_y = M_{1, 2}$$
     $$s = \sqrt{M_{0, 0}^2 + M_{1, 0}^2}$$
     $$\theta = \operatorname{atan2}(M_{1, 0}, M_{0, 0})$$

---

## 2. Tilt Compensation Algorithm

**File location:** [optical_flow_stream.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py)

Camera rotation (pitch and roll) causes apparent visual flow. When the drone pitches forward, the ground sweeps backward, which looks identical to forward translation. The tilt compensation algorithm separates this rotational flow from physical translation.

1. **Virtual Reticle Projection**: 
   A virtual center projection is calculated using current roll and pitch angles:
   $$\text{roll\_px} = \text{roll}_{\text{deg}} \cdot K_{\text{roll}}$$
   $$\text{pitch\_px} = -\text{pitch}_{\text{deg}} \cdot K_{\text{pitch}}$$
2. **Frame-to-Frame Attitude Delta**:
   Expected pixel displacement is calculated using the change in orientation since the last frame:
   $$d_{\text{reticle}, x} = \text{roll\_px}_t - \text{roll\_px}_{t-1}$$
   $$d_{\text{reticle}, y} = \text{pitch\_px}_t - \text{pitch\_px}_{t-1}$$
3. **Compensation Subtracting**:
   Expected displacement is scaled by the calibrated parameters (`scale_x`, `scale_y`) and subtracted from the affine translation ($t_x, t_y$):
   $$t_{x,\text{compensated}} = t_x - (\text{scale}_x \cdot d_{\text{reticle}, x})$$
   $$t_{y,\text{compensated}} = t_y - (\text{scale}_y \cdot d_{\text{reticle}, y})$$

---

## 3. Camera Lever-Arm Offset Correction

**File location:** [optical_flow_stream.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py)

If the camera is not mounted exactly at the center of gravity (rotation center) of the drone, any yaw rotation of the vehicle will induce linear translation at the camera sensor.

Let $(C_x, C_y)$ be the camera installation offsets (in meters) relative to the center of rotation. Given the vehicle's yaw rate $\omega_z$ (in rad/s):
* **Induced Linear Velocity**:
  $$v_{\text{offset}, x} = -\omega_z \cdot C_y$$
  $$v_{\text{offset}, y} = \omega_z \cdot C_x$$
* **Velocity Correction**:
  $$v_{x,\text{body, comp}} = v_{x,\text{body}} - v_{\text{offset}, x}$$
  $$v_{y,\text{body, comp}} = v_{y,\text{body}} - v_{\text{offset}, y}$$

---

## 4. Dead Reckoning and GPS Coordinates Synthesis

**File location:** [static_gps_injector.py](file:///e:/OptFlowDrone/OpticalFlowDrone/static_gps_injector.py)

The world frame velocity ($V_{x,\text{world}}, V_{y,\text{world}}$) is integrated over time to compute distance from origin and update latitudinal/longitudinal coordinates.

1. **Discrete Position Integration**:
   $$x_{t} = x_{t-1} + V_{x,\text{world}} \cdot dt$$
   $$y_{t} = y_{t-1} + V_{y,\text{world}} \cdot dt$$
2. **Meters-to-GPS Projection**:
   Using the Earth's radius ($R = 6,378,137\text{ m}$), meters displacement $(x, y)$ is converted to latitude and longitude offsets from a configured starting coordinates coordinate ($\text{lat}_0, \text{lon}_0$):
   $$\text{lat}_t = \text{lat}_0 + \left( \frac{y_t}{R} \right) \cdot \left( \frac{180}{\pi} \right)$$
   $$\text{lon}_t = \text{lon}_0 + \left( \frac{x_t}{R \cdot \cos(\text{lat}_t)} \right) \cdot \left( \frac{180}{\pi} \right)$$
3. **NMEA Checksum Algorithm**:
   Every generated NMEA sentence must end with a XOR checksum of all characters between `$` and `*`:
   $$\text{Checksum} = \bigoplus_{i=1}^{N} \operatorname{ord}(\text{sentence}[i])$$
   This calculated hex string is appended to output strings to ensure the autopilot parser does not drop packets.
