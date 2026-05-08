# Complementary Filter Analysis (optical_flow_stream.py)

## Purpose
The complementary filter in this project fuses:
- Gyroscope angular rates (good short-term responsiveness, drifts over time)
- Accelerometer-derived tilt (stable long-term gravity reference, noisy in motion)

This gives a practical estimate of roll and pitch for OSD/reticle stabilization with low compute cost.

## Where it runs in the code
The filter runs inside the I2C IMU loop in optical_flow_stream.py.

Key constants and state:
- COMPLEMENTARY_FILTER_ALPHA = 0.96
- attitude_state["roll_deg"], attitude_state["pitch_deg"], attitude_state["yaw_deg"]
- last_imu_ts for integration time

Main steps happen in this order:
1. Read raw accel and gyro registers from MPU6050.
2. Convert raw counts into physical units (g and deg/s).
3. Subtract calibrated gyro bias from gyro rates.
4. Compute roll/pitch from accelerometer direction.
5. Integrate gyro rates using dt.
6. Blend gyro-integrated tilt with accel tilt using complementary filter.
7. Keep yaw as gyro-only integration.

## Sensor conversion pipeline
Given raw readings:
- ax_raw, ay_raw, az_raw
- gx_raw, gy_raw, gz_raw

The code computes:
- xaccel_g = ax_raw / ACCEL_LSB_PER_G
- yaccel_g = ay_raw / ACCEL_LSB_PER_G
- zaccel_g = az_raw / ACCEL_LSB_PER_G
- xgyro_dps = gx_raw / GYRO_LSB_PER_DPS - gyro_bias["x"]
- ygyro_dps = gy_raw / GYRO_LSB_PER_DPS - gyro_bias["y"]
- zgyro_dps = gz_raw / GYRO_LSB_PER_DPS - gyro_bias["z"]

with:
- ACCEL_LSB_PER_G = 16384.0
- GYRO_LSB_PER_DPS = 131.0

## Accelerometer tilt estimate
Function: accel_to_roll_pitch(ax_g, ay_g, az_g)

Steps:
1. Compute acceleration magnitude.
2. Normalize vector to reduce scale sensitivity.
3. Compute tilt from gravity direction:

$$
\text{roll}_{acc} = \operatorname{atan2}(a_y, a_z)
$$

$$
\text{pitch}_{acc} = \operatorname{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)
$$

Then convert radians to degrees and normalize to [-180, 180).

Note: if acceleration magnitude is too small (< 0.1 g), the function returns (0, 0) as a guard.

## Gyroscope integration
If last_imu_ts is known and dt is valid (0 < dt < 0.1), the code integrates:

$$
\text{roll}_{gyro} = \text{roll}_{prev} + \omega_x \cdot dt
$$

$$
\text{pitch}_{gyro} = \text{pitch}_{prev} + \omega_y \cdot dt
$$

$$
\text{yaw}_{gyro} = \text{yaw}_{prev} + \omega_z \cdot dt
$$

where angular rates are in deg/s, so integrated angles are degrees.

## Complementary blend
For roll and pitch, the code applies:

$$
\text{roll} = \alpha \cdot \text{roll}_{gyro} + (1-\alpha) \cdot \text{roll}_{acc}
$$

$$
\text{pitch} = \alpha \cdot \text{pitch}_{gyro} + (1-\alpha) \cdot \text{pitch}_{acc}
$$

with:
- $\alpha = 0.96$
- $1-\alpha = 0.04$

Interpretation:
- High-pass behavior from gyro path (short-term dynamics)
- Low-pass behavior from accel path (long-term leveling)

Yaw update in current code:

$$
\text{yaw} = \text{yaw}_{gyro}
$$

So yaw has no absolute reference correction and will drift over time.

## Timing behavior in this implementation
The IMU loop includes sleep(0.02), targeting about 50 Hz.

Approximate filter corner intuition for a discrete complementary filter:

$$
\tau \approx -\frac{dt}{\ln(\alpha)}
$$

$$
f_c \approx \frac{1}{2\pi\tau}
$$

At dt ~ 0.02 s and alpha = 0.96:
- $\tau \approx 0.49\,s$
- $f_c \approx 0.32\,Hz$

This means slow accel-based correction (good for smooth leveling, less reactive to transient linear acceleration).

## Why this works well for your stream OSD
- Fast reticle motion comes from gyro integration.
- Long-term horizon alignment is pulled back by gravity estimate.
- Compute cost is tiny compared to full AHRS/EKF.

## Current limitations and edge cases
1. Yaw drift
Yaw is gyro-only; without magnetometer or external heading, long-term yaw error accumulates.

2. Linear acceleration contamination
When the drone accelerates, accelerometer no longer measures pure gravity, so tilt correction can be biased temporarily.

3. Fixed alpha
A single blend factor may be suboptimal across all flight regimes.

4. Axis/sign conventions
If sensor orientation differs from assumed axes, roll/pitch signs can appear inverted or coupled.

5. dt gating
Updates are skipped when dt >= 0.1; this prevents integration spikes, but prolonged timing stalls freeze angle propagation for that period.

## Tuning guidance
1. Alpha tuning
- Increase alpha (for example 0.98): smoother, more gyro-dominant, slower leveling.
- Decrease alpha (for example 0.92): faster leveling, more accel noise influence.

2. Sample rate consistency
Keep IMU loop timing steady; unstable dt increases noise in integrated angles.

3. Bias calibration quality
The startup gyro bias average is critical; calibrate when stationary and vibration-free.

4. Optional improvements
- Dynamic alpha: reduce accel trust during high |a|-1 conditions.
- Add a magnetometer or heading source to correct yaw drift.
- Use quaternion-based Mahony/Madgwick if full attitude robustness is needed.

## Practical validation checklist
- Device stationary on level surface: roll/pitch should settle near 0 deg.
- Hold fixed tilt: estimates should converge and remain stable.
- Quick hand rotations: response should be immediate with minimal overshoot.
- Long stationary run: roll/pitch should not drift significantly; yaw drift is expected.

## Summary
The implemented complementary filter is a classic, efficient tilt estimator:
- Gyro integration provides responsive short-term attitude change.
- Accelerometer tilt provides long-term correction.
- Alpha = 0.96 at ~50 Hz gives smooth behavior with conservative correction.

For this optical flow and OSD use case, it is an appropriate balance of simplicity and performance, with yaw drift as the main expected trade-off.

## Calculation example

This is a simple numerical example showing one IMU update step using the constants from the code.

Assumptions:
- Previous attitude: `roll_prev = 8.0 deg`, `pitch_prev = 1.0 deg`
- Gyro rates: `xgyro = 12.0 deg/s`, `ygyro = -6.0 deg/s` (roll, pitch rates)
- Accelerometer-derived tilt: `roll_acc = 10.0 deg`, `pitch_acc = -2.0 deg`
- Time delta: `dt = 0.02 s` (≈50 Hz IMU loop)
- Alpha: `\(\alpha = 0.96\)`

1) Integrate the gyro rates:

$$
	ext{roll}_{gyro} = 8.0 + 12.0 \times 0.02 = 8.24\ \text{deg}
$$

$$
	ext{pitch}_{gyro} = 1.0 + (-6.0) \times 0.02 = 0.88\ \text{deg}
$$

2) Blend with accelerometer tilt using the complementary filter:

$$
	ext{roll} = 0.96 \times 8.24 + 0.04 \times 10.0 = 7.9104 + 0.4 = 8.3104\ \text{deg}
$$

$$
	ext{pitch} = 0.96 \times 0.88 + 0.04 \times (-2.0) = 0.8448 - 0.08 = 0.7648\ \text{deg}
$$

Result: the fused attitude after this IMU update would be approximately `roll = 8.31 deg`, `pitch = 0.76 deg`.

Interpretation:
- Gyro integration provides the short-term motion (small increment from 8.0 → 8.24 deg).
- The accelerometer pulls the solution slightly toward its 10 deg reading, producing the final 8.31 deg.
- Because alpha is high (0.96), the accel only nudges the gyro result by 4% each update.

## Worked Numerical Example

This small worked example shows how the code's conversions, integration and complementary blend produce the fused roll/pitch values.

Assumptions and constants from the code:
- `ACCEL_LSB_PER_G = 16384.0`
- `GYRO_LSB_PER_DPS = 131.0`
- `COMPLEMENTARY_FILTER_ALPHA = 0.96`
- `dt = 0.02` s (typical IMU loop sleep)

Example raw readings (plausible MPU6050 counts):
- `ax_raw = 2848` (=> `xaccel_g = 2848 / 16384 = 0.17365`)  
- `ay_raw = 0` (=> `yaccel_g = 0`)  
- `az_raw = 16128` (=> `zaccel_g = 16128 / 16384 = 0.98481`)  
- `gx_raw = 262` (=> `xgyro_dps = 262 / 131 = 2.0` dps)  
- `gy_raw = 66`  (=> `ygyro_dps ≈ 66 / 131 ≈ 0.504` dps)  
- gyro biases assumed zero for clarity

1) Accelerometer tilt estimate (using the same math as `accel_to_roll_pitch`):

$$
	ext{roll}_{acc} = \operatorname{atan2}(a_y, a_z) = \operatorname{atan2}(0, 0.98481) = 0^\circ
$$

$$
	ext{pitch}_{acc} = \operatorname{atan2}(-a_x, \sqrt{a_y^2 + a_z^2}) = \operatorname{atan2}(-0.17365, 0.98481) \approx -10^\circ
$$

2) Gyro integration (suppose previous state `roll_prev = -9.5^\circ`, `pitch_prev = -9.5^\circ`):

$$
	ext{roll}_{gyro} = -9.5 + 2.0 \cdot 0.02 = -9.5 + 0.04 = -9.46^\circ
$$

$$
	ext{pitch}_{gyro} = -9.5 + 0.504 \cdot 0.02 \approx -9.5 + 0.0101 = -9.4899^\circ
$$

3) Complementary blend (\(\alpha = 0.96\)):

$$
	ext{roll} = 0.96 \cdot (-9.46) + 0.04 \cdot 0 = -9.0816^\circ
$$

$$
	ext{pitch} = 0.96 \cdot (-9.4899) + 0.04 \cdot (-10) \approx -9.1103 - 0.4 = -9.5103^\circ
$$

Interpretation:
- The fused roll moves toward the accelerometer's 0° reading (reducing the negative error) because `roll_acc` was 0°.
- The fused pitch moves slightly toward the accelerometer's -10° reference while largely preserving the short-term gyro change.

This example mirrors the actual code sequence: convert -> accel tilt -> gyro integrate -> blend. It shows numerically how a high alpha (0.96) keeps the gyroscope dominant while the accelerometer provides a small but steady correction.
