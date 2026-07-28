# Mathematical Calculations & Estimations Guide

This document describes the mathematical formulas, coordinate systems, and algorithms used in the drone's optical flow, sensor fusion, and telemetry processing pipeline, with step-by-step numerical examples.

---

## 1. Attitude Estimation (Complementary Filter)
To obtain stable roll ($\phi$) and pitch ($\theta$) angles, the system fuses high-frequency gyroscope angular rates with gravity-aligned accelerometer angles. This filters out high-frequency vibrations from the accelerometer and eliminates long-term drift from the gyroscope.

### 📐 Mathematical Formulation
1. **Accelerometer-derived Roll & Pitch:**
   $$\phi_{\text{accel}} = \arctan2(a_y, a_z)$$
   $$\theta_{\text{accel}} = \arctan2\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$

2. **Complementary Filter Fusion:**
   $$\phi_t = \alpha \cdot (\phi_{t-1} + \omega_x \cdot dt) + (1 - \alpha) \cdot \phi_{\text{accel}}$$
   $$\theta_t = \alpha \cdot (\theta_{t-1} + \omega_y \cdot dt) + (1 - \alpha) \cdot \theta_{\text{accel}}$$
   *(Where $\alpha = 0.98$ is the mixing factor)*

### 💻 Code Reference
* [`accel_to_roll_pitch()` in sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L41-L53)
* [Complementary filter update in `sensor_readers.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py#L338-L356)

### 📝 Step-by-Step Example
Suppose:
* Previous state: Roll $\phi_{t-1} = 2.0^\circ$
* Gyroscope rate: $\omega_x = 10.0^\circ/\text{s}$
* Time step: $dt = 0.05\text{ s}$
* Accelerometer readings: $a_x = 0.0\text{ g}$, $a_y = 0.087\text{ g}$, $a_z = 0.996\text{ g}$
* Filter coefficient: $\alpha = 0.98$

**Step 1: Compute Accelerometer Angle**
$$\phi_{\text{accel}} = \arctan2(0.087, 0.996) \approx 0.0871\text{ rad} \approx 5.0^\circ$$

**Step 2: Integrate Gyroscope Rate**
$$\phi_{\text{gyro}} = \phi_{t-1} + \omega_x \cdot dt = 2.0^\circ + (10.0^\circ/\text{s} \times 0.05\text{ s}) = 2.5^\circ$$

**Step 3: Apply Complementary Filter**
$$\phi_t = 0.98 \cdot (2.5^\circ) + 0.02 \cdot (5.0^\circ) = 2.45^\circ + 0.10^\circ = 2.55^\circ$$

---

## 2. Focal Length & Optical Flow Velocity
To translate raw pixel shifts ($tx, ty$) from the camera into physical body velocities ($v_x, v_y$ in m/s), we need the camera's focal length in pixels ($f$) and the altitude above the ground ($Z$).

### 📐 Mathematical Formulation
1. **Focal Length (pixels):**
   $$f_x = \frac{W}{2 \tan\left(\frac{\text{FOV}_h}{2}\right)}$$

2. **Flow-to-Velocity Conversion (Translational Flow):**
   $$v_{x, \text{body}} = \frac{tx \cdot Z}{f_x \cdot dt}$$
   $$v_{y, \text{body}} = -\frac{ty \cdot Z}{f_y \cdot dt}$$

### 💻 Code Reference
* [`focal_length_px()` in flow_processor.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/flow_processor.py#L122-L123)
* [Velocity computation in `optical_flow_stream.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py#L142-L143)

### 📝 Step-by-Step Example
Suppose:
* Frame Width ($W$) = $640\text{ px}$
* Camera Horizontal FOV = $60^\circ$ (so $\frac{\text{FOV}_h}{2} = 30^\circ$)
* Altitude ($Z$) = $1.5\text{ m}$
* Measured horizontal pixel shift ($tx$) = $4.0\text{ px}$
* Time step ($dt$) = $0.033\text{ s}$ (30 FPS)

**Step 1: Compute Focal Length in Pixels**
$$f_x = \frac{640}{2 \cdot \tan(30^\circ)} = \frac{640}{2 \cdot 0.57735} \approx 554.26\text{ px}$$

**Step 2: Compute Body Velocity**
$$v_{x, \text{body}} = \frac{4.0\text{ px} \times 1.5\text{ m}}{554.26\text{ px} \times 0.033\text{ s}} = \frac{6.0}{18.29} \approx 0.328\text{ m/s}$$

---



## 5. Battery mAh Consumption Integration
During simulation or fallback modes, the system estimates the accumulated battery energy consumption (in mAh) by integrating current draw over time.

### 📐 Mathematical Formulation
$$\Delta\text{Capacity}_{\text{consumed}} = I \cdot 1000 \cdot \frac{dt}{3600}$$
$$\text{Capacity}_{\text{pct}} = 100\% - \left(\frac{\text{Capacity}_{\text{consumed}}}{\text{Capacity}_{\text{nominal}}}\right) \cdot 100\%$$

### 💻 Code Reference
* [Battery capacity math in `battery_monitor.py`](file:///e:/OptFlowDrone/OpticalFlowDrone/battery_monitor.py#L174-L184)

### 📝 Step-by-Step Example
Suppose:
* Current Draw ($I$) = $8.5\text{ A}$
* Time step ($dt$) = $0.5\text{ s}$
* Nominal Battery Capacity = $2200\text{ mAh}$
* Previous Consumed Capacity = $150.0\text{ mAh}$

**Step 1: Calculate mAh Consumed in this Step**
$$\Delta\text{Capacity}_{\text{consumed}} = 8.5\text{ A} \times 1000\text{ mA/A} \times \frac{0.5\text{ s}}{3600\text{ s/h}} \approx 1.18\text{ mAh}$$

**Step 2: Update Total Consumed**
$$\text{Capacity}_{\text{consumed, new}} = 150.0\text{ mAh} + 1.18\text{ mAh} = 151.18\text{ mAh}$$

**Step 3: Compute Remaining Percentage**
$$\text{Capacity}_{\text{pct}} = 100\% - \left(\frac{151.18\text{ mAh}}{2200\text{ mAh}}\right) \times 100\% \approx 100\% - 6.87\% = 93.13\%$$
