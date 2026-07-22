#!/usr/bin/env python3
"""
Madgwick AHRS (Attitude and Heading Reference System) Algorithm & Position Estimator
-----------------------------------------------------------------------------------
Implements Sebastian Madgwick's 6-DOF IMU gradient descent orientation filter
(Accelerometer + Gyroscope, without Magnetometer), extracts Earth-frame linear
acceleration by removing gravity projection, and estimates 3D velocity and position
via drift-compensated double numerical integration.

References:
  - Sebastian O.H. Madgwick, "An efficient orientation filter for inertial and
    inertial/magnetic sensor arrays", Report x-io and University of Bristol, 2010.
"""

import math
import time


class MadgwickAHRS:
    """
    6-DOF Madgwick AHRS Filter (IMU mode, without magnetometer).
    Maintains orientation quaternion q = [q0, q1, q2, q3] where q0 is scalar (w).
    """

    def __init__(self, beta=0.1, sample_freq=50.0):
        """
        :param beta: Filter gain (gradient descent step size). Default: 0.1
        :param sample_freq: Default sampling frequency in Hz.
        """
        self.beta = beta
        self.sample_freq = sample_freq
        # Quaternion state: [qw, qx, qy, qz]
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.last_update_ts = None

    def reset(self):
        """Resets orientation quaternion to identity [1, 0, 0, 0]."""
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.last_update_ts = None

    def update_imu(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt=None):
        """
        Updates the 6-DOF orientation quaternion using gyro (deg/s) and accel (g).

        :param gx_dps, gy_dps, gz_dps: Gyroscope angular rates in degrees/second.
        :param ax_g, ay_g, az_g: Accelerometer values in g (or m/s^2, normalized internally).
        :param dt: Time step in seconds. If None, computed from time.time().
        :return: (qw, qx, qy, qz)
        """
        now = time.time()
        if dt is None:
            if self.last_update_ts is not None:
                dt = now - self.last_update_ts
            else:
                dt = 1.0 / self.sample_freq
        self.last_update_ts = now

        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.sample_freq

        q0, q1, q2, q3 = self.q

        # Convert gyroscope angular rates from deg/s to rad/s
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        # Compute rate of change of quaternion from gyroscope: 0.5 * q (x) w
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Compute feedback only if accelerometer measurement is valid (non-zero)
        norm_accel = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        if norm_accel > 1e-4:
            ax = ax_g / norm_accel
            ay = ay_g / norm_accel
            az = az_g / norm_accel

            # Auxiliary variables to avoid repeated calculations
            _2q0 = 2.0 * q0
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _4q0 = 4.0 * q0
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _8q1 = 8.0 * q1
            _8q2 = 8.0 * q2
            q0q0 = q0 * q0
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3

            # Objective function f(q, a) = q* x g_ref x q - a
            f0 = _2q1 * q3 - _2q0 * q2 - ax
            f1 = _2q0 * q1 + _2q2 * q3 - ay
            f2 = 1.0 - _2q1 * q1 - _2q2 * q2 - az

            # Jacobian J(q) transposed times objective function f(q, a)
            s0 = -_2q2 * f0 + _2q1 * f1
            s1 = _2q3 * f0 + _2q0 * f1 - _4q1 * f2
            s2 = -_2q0 * f0 + _2q3 * f1 - _4q2 * f2
            s3 = _2q1 * f0 + _2q2 * f1

            # Normalize gradient
            norm_s = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if norm_s > 1e-8:
                s0 /= norm_s
                s1 /= norm_s
                s2 /= norm_s
                s3 /= norm_s

                # Apply feedback step: qDot = qDot_gyro - beta * gradient
                qDot1 -= self.beta * s0
                qDot2 -= self.beta * s1
                qDot3 -= self.beta * s2
                qDot4 -= self.beta * s3

        # Integrate rate of change of quaternion
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt

        # Normalize quaternion
        norm_q = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm_q > 1e-8:
            q0 /= norm_q
            q1 /= norm_q
            q2 /= norm_q
            q3 /= norm_q

        self.q = [q0, q1, q2, q3]
        return self.get_quaternion()

    def get_quaternion(self):
        """Returns dict of quaternion components {'w', 'x', 'y', 'z'}."""
        return {"w": self.q[0], "x": self.q[1], "y": self.q[2], "z": self.q[3]}

    def get_euler_deg(self):
        """
        Converts orientation quaternion to Euler angles in degrees (Roll, Pitch, Yaw).
        Convention: Z-Y-X (Yaw-Pitch-Roll).
        """
        q0, q1, q2, q3 = self.q

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll_rad = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            pitch_rad = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_rad = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        return (
            math.degrees(roll_rad),
            math.degrees(pitch_rad),
            math.degrees(yaw_rad),
        )


class MadgwickPositionEstimator:
    """
    Estimates 3D Position and Velocity using Madgwick AHRS orientation filtering,
    Earth-frame linear acceleration extraction, and leak-decay double integration.
    """

    def __init__(
        self,
        beta=0.1,
        sample_freq=50.0,
        gravity_m_s2=9.80665,
        vel_decay=0.985,
        zupt_threshold=0.12,
    ):
        """
        :param beta: Madgwick filter gain.
        :param sample_freq: Default sampling frequency in Hz.
        :param gravity_m_s2: Gravity acceleration constant in m/s^2 (default: 9.80665).
        :param vel_decay: Velocity decay factor per step to bound integration drift (0.95 - 0.99).
        :param zupt_threshold: Linear acceleration norm threshold (m/s^2) for zero-velocity update.
        """
        self.ahrs = MadgwickAHRS(beta=beta, sample_freq=sample_freq)
        self.gravity = gravity_m_s2
        self.vel_decay = vel_decay
        self.zupt_threshold = zupt_threshold

        # 3D State Vectors
        self.pos = [0.0, 0.0, 0.0]  # [x, y, z] in meters
        self.vel = [0.0, 0.0, 0.0]  # [vx, vy, vz] in m/s
        self.lin_accel_earth = [0.0, 0.0, 0.0]  # [ax, ay, az] in m/s^2 in Earth frame
        self.lin_accel_filtered = [0.0, 0.0, 0.0]
        self.last_ts = None

    def reset_position(self):
        """Resets position and velocity vectors to zero."""
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.lin_accel_earth = [0.0, 0.0, 0.0]
        self.lin_accel_filtered = [0.0, 0.0, 0.0]

    def reset_all(self):
        """Resets both orientation filter and position/velocity states."""
        self.ahrs.reset()
        self.reset_position()

    def rotate_vector_by_quaternion(self, vx, vy, vz, q):
        """
        Rotates 3D vector [vx, vy, vz] from Sensor/Body frame to Earth frame
        using quaternion q = [q0, q1, q2, q3].
        v_earth = q * v_body * q_conj
        """
        q0, q1, q2, q3 = q

        # Quaternion multiplication: q * v
        tw0 = -q1 * vx - q2 * vy - q3 * vz
        tw1 = q0 * vx + q2 * vz - q3 * vy
        tw2 = q0 * vy - q1 * vz + q3 * vx
        tw3 = q0 * vz + q1 * vy - q2 * vx

        # (q * v) * q_conj where q_conj = [q0, -q1, -q2, -q3]
        ex = tw0 * (-q1) + tw1 * q0 + tw2 * (-q3) - tw3 * (-q2)
        ey = tw0 * (-q2) - tw1 * (-q3) + tw2 * q0 + tw3 * (-q1)
        ez = tw0 * (-q3) + tw1 * (-q2) - tw2 * (-q1) + tw3 * q0

        return ex, ey, ez

    def update(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt=None):
        """
        Main update step for Madgwick AHRS and 3D Position / Velocity Integration.

        :param gx_dps, gy_dps, gz_dps: Gyroscope angular rates (deg/s).
        :param ax_g, ay_g, az_g: Accelerometer values in g.
        :param dt: Delta time step in seconds.
        :return: Dict containing position, velocity, linear_accel, quaternion, euler.
        """
        now = time.time()
        if dt is None:
            if self.last_ts is not None:
                dt = now - self.last_ts
            else:
                dt = 1.0 / self.ahrs.sample_freq
        self.last_ts = now

        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.ahrs.sample_freq

        # 1. Update Madgwick AHRS filter orientation
        self.ahrs.update_imu(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt=dt)
        q = self.ahrs.q

        # 2. Convert body accelerometer readings to m/s^2
        ax_ms2 = ax_g * self.gravity
        ay_ms2 = ay_g * self.gravity
        az_ms2 = az_g * self.gravity

        # 3. Rotate body acceleration vector into Earth frame
        ax_earth, ay_earth, az_earth = self.rotate_vector_by_quaternion(ax_ms2, ay_ms2, az_ms2, q)

        # 4. Subtract gravity (gravity acts along Earth +Z)
        lin_ax = ax_earth
        lin_ay = ay_earth
        lin_az = az_earth - self.gravity

        self.lin_accel_earth = [lin_ax, lin_ay, lin_az]

        # Low-pass filter linear acceleration to eliminate high-frequency motor/sensor noise (alpha = 0.2)
        alpha_accel = 0.2
        self.lin_accel_filtered[0] += (lin_ax - self.lin_accel_filtered[0]) * alpha_accel
        self.lin_accel_filtered[1] += (lin_ay - self.lin_accel_filtered[1]) * alpha_accel
        self.lin_accel_filtered[2] += (lin_az - self.lin_accel_filtered[2]) * alpha_accel

        fax, fay, faz = self.lin_accel_filtered

        # Deadband small noise floor below 0.025 m/s^2
        if abs(fax) < 0.025: fax = 0.0
        if abs(fay) < 0.025: fay = 0.0
        if abs(faz) < 0.025: faz = 0.0

        # 5. Check linear acceleration magnitude for Zero Velocity Update (ZUPT)
        accel_norm = math.sqrt(fax * fax + fay * fay + faz * faz)

        if accel_norm < self.zupt_threshold:
            # Smooth exponential velocity dampening when stationary
            self.vel[0] *= 0.90
            self.vel[1] *= 0.90
            self.vel[2] *= 0.90
        else:
            # Integrate acceleration to velocity with exponential decay factor
            self.vel[0] = (self.vel[0] + fax * dt) * self.vel_decay
            self.vel[1] = (self.vel[1] + fay * dt) * self.vel_decay
            self.vel[2] = (self.vel[2] + faz * dt) * self.vel_decay

        # 6. Integrate velocity to position
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt

        return self.get_state()

    def get_state(self):
        """Returns current full telemetry state dictionary."""
        roll, pitch, yaw = self.ahrs.get_euler_deg()
        quat = self.ahrs.get_quaternion()

        return {
            "position": {
                "x": round(self.pos[0], 4),
                "y": round(self.pos[1], 4),
                "z": round(self.pos[2], 4),
            },
            "velocity": {
                "x": round(self.vel[0], 4),
                "y": round(self.vel[1], 4),
                "z": round(self.vel[2], 4),
            },
            "linear_accel": {
                "x": round(self.lin_accel_earth[0], 4),
                "y": round(self.lin_accel_earth[1], 4),
                "z": round(self.lin_accel_earth[2], 4),
            },
            "quaternion": quat,
            "euler": {
                "roll": round(roll, 2),
                "pitch": round(pitch, 2),
                "yaw": round(yaw, 2),
            },
        }


if __name__ == "__main__":
    print("Testing Madgwick AHRS Algorithm & Position Estimator...")
    estimator = MadgwickPositionEstimator(beta=0.1, sample_freq=50.0)

    # Simulated IMU static test (0.0 dps gyro, 1.0g z-axis accel)
    for i in range(100):
        state = estimator.update(
            gx_dps=0.0, gy_dps=0.0, gz_dps=0.0, ax_g=0.0, ay_g=0.0, az_g=1.0, dt=0.02
        )

    print("Static test position:", state["position"])
    print("Static test euler:", state["euler"])
    print("Static test quaternion:", state["quaternion"])
