# Sensor Calibration Guide

This document details the calibration procedures, math models, trigger mechanisms, and file storage schemas used to align the drone's IMU, optical flow camera, and digital compass.

---

## 1. Static Gyroscope Bias Calibration

**Implementation:** [sensor_readers.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow/sensor_readers.py)  
**Trigger:** Runs automatically during script initialization inside `start_distance_sensor_reader()`.

### Purpose
To calculate the steady-state bias (offset error) of the MPU6050 gyroscope axes. Without calibration, the small offset would accumulate during angular integration, causing the attitude angles to drift over time.

### Procedure
1. Place the drone on a stable, level surface. **The drone must remain completely static.**
2. Run the services stack.
3. The reader thread initiates `calibrate_gyro_bias()`, which executes the following logic:
   * Collects $N = 200$ samples from the gyroscope registers at **100 Hz**.
   * Computes the mean offset error for each axis:
     $$\text{bias}_x = \frac{1}{N} \sum_{i=1}^{N} \omega_{x, i}, \quad \text{bias}_y = \frac{1}{N} \sum_{i=1}^{N} \omega_{y, i}, \quad \text{bias}_z = \frac{1}{N} \sum_{i=1}^{N} \omega_{z, i}$$
   * Sets `gyro_calibrated = True` and writes the biases to a memory configuration dictionary to subtract them from all future raw measurements.

---

## 2. Optical Flow Tilt Scaling Calibration

**Implementation:** [optical_flow_stream.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py)  
**Trigger:** Press key `'c'` inside the terminal, or send the calibration trigger command.

### Purpose
Aligns the virtual reticle scaling ratios (`scale_x`, `scale_y`). It calculates how many pixels of optical flow are generated per degree of physical camera rotation, allowing the filter to subtract rotation-induced flow.

### Procedure
1. Power up the drone and lift it to a height of approximately 1 to 1.5 meters over a high-contrast textured surface.
2. Trigger the calibration mode by pressing `'c'`. The console outputs:
   `>>> TILT CALIBRATION STARTED. Please pitch and roll the camera/drone without translating it.`
3. **Manual Action:** Pitch and roll the drone back and forth (rotate it) while keeping the drone centered in the same space (do not translate it laterally).
4. The system collects sample coordinates where movement exceeds thresholds, calculating ratios:
   $$\text{sample}_x = \frac{t_x}{d_{\text{reticle}, x}}, \quad \text{sample}_y = \frac{t_y}{d_{\text{reticle}, y}}$$
5. Press `'s'` to stop and save the calibration. The system calculates the median of the collected arrays to filter out anomalies:
   $$\text{scale}_x = \operatorname{median}(\text{samples}_x), \quad \text{scale}_y = \operatorname{median}(\text{samples}_y)$$
6. Writes the parameters directly to `tilt_calibration.json`:
   ```json
   {
       "scale_x": 0.0482,
       "scale_y": 0.0491
   }
   ```

---

## 3. Camera Lever-Arm Offsets Configuration

**Implementation:** [optical_flow_stream.py](file:///e:/OptFlowDrone/OpticalFlowDrone/optical_flow_stream.py)  
**Trigger:** Submitted via the Web Dashboard configuration interface, writing directly to `tilt_calibration.json`.

### Purpose
Compensates for visual flow generated when the camera is not mounted at the absolute center of rotation (CoR) of the drone. When the drone rotates, the offset camera moves along an arc, introducing apparent translation.

### Procedure
1. Measure the physical distance from the camera lens to the center of rotation (usually the center of the flight controller) in centimeters:
   * **`camera_offset_x`**: Distance along the forward/backward axis (+ for forward, - for backward).
   * **`camera_offset_y`**: Distance along the left/right axis (+ for right, - for left).
2. Enter these offsets into the Web Dashboard interface or POST to `/api/opticalflow/offset`.
3. The values are saved in `tilt_calibration.json`:
   ```json
   {
       "scale_x": 0.0482,
       "scale_y": 0.0491,
       "camera_offset_x": 5.0,
       "camera_offset_y": -2.0
   }
   ```
4. The optical flow system automatically loads these parameters to correct body velocities:
   $$v_{x,\text{corrected}} = v_{x,\text{body}} - \left( -\omega_z \cdot \frac{C_y}{100.0} \right)$$

---

## 4. Compass Zero Offset Calibration

**Implementation:** [hmc5883l.py](file:///e:/OptFlowDrone/OpticalFlowDrone/hmc5883l.py)  
**Trigger:** Configured on the Compass Web Dashboard GUI (port 5003).

### Purpose
Aligns the digital compass heading reading to True North. This accounts for local magnetic declination and mounting orientation misalignment.

### Procedure
1. Point the drone's nose exactly toward True North.
2. Read the current uncalibrated heading display on the compass dashboard.
3. Click "Zero Offset" on the dashboard to calculate the zero offset:
   $$\text{zero\_offset} = \text{uncalibrated\_heading} \pmod{360}$$
4. The value is saved to `compass_zero_offset.json`:
   ```json
   {
       "zero_offset": 124.5
   }
   ```
5. All future readings are rotated by this offset:
   $$\text{calibrated\_heading} = (\text{raw\_heading} - \text{zero\_offset}) \pmod{360}$$
