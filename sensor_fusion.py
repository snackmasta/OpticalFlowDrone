#!/usr/bin/env python3
"""
Multi-Sensor Fusion Engine for UAV Navigation
---------------------------------------------
Fuses 6-DOF IMU (Accelerometer + Gyroscope), HMC5883L Compass (Heading),
Optical Flow (Body Velocities), and GPS (Global Coordinates) into a drift-free,
high-rate (50Hz) smooth 2D/3D state estimator.

Mathematical Formulation:
1. Attitude & Heading (IMU + Compass):
   - Roll (phi), Pitch (theta) from IMU Madgwick AHRS or Accelerometer gravity vector.
   - Yaw (psi) from HMC5883L Compass with tilt compensation.

2. Body-to-Earth Velocity Rotation:
   - v_East  =  v_body_x * cos(psi) + v_body_y * sin(psi)
   - v_North = -v_body_x * sin(psi) + v_body_y * cos(psi)

3. 50Hz Dead-Reckoning Prediction:
   - x_pred = x_fused + v_East  * dt
   - y_pred = y_fused + v_North * dt

4. Low-Frequency GPS Measurement Innovation Update (1 - 5Hz):
   - innovation = p_gps - p_pred
   - p_fused = p_pred + K_gps * innovation
   - Inverse project p_fused back to WGS-84 Geographic (Fused_Lat, Fused_Lon).
"""

import math
import time

EARTH_RADIUS_M = 6378137.0


def geo_to_meters(lat, lon, origin_lat, origin_lon):
    """Converts WGS-84 Geographic (Lat, Lon) to Local Tangent Plane (X East, Y North) in meters."""
    if lat is None or lon is None or origin_lat is None or origin_lon is None:
        return 0.0, 0.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    orig_lat_rad = math.radians(origin_lat)
    orig_lon_rad = math.radians(origin_lon)

    dlat = lat_rad - orig_lat_rad
    dlon = lon_rad - orig_lon_rad
    avg_lat = (lat_rad + orig_lat_rad) / 2.0

    x_m = dlon * math.cos(avg_lat) * EARTH_RADIUS_M
    y_m = dlat * EARTH_RADIUS_M
    return round(x_m, 3), round(y_m, 3)


def meters_to_geo(x_m, y_m, origin_lat, origin_lon):
    """Converts Local Tangent Plane (X East, Y North) in meters to WGS-84 Geographic (Lat, Lon)."""
    if origin_lat is None or origin_lon is None:
        return 0.0, 0.0
    dlat_rad = y_m / EARTH_RADIUS_M
    lat_rad = math.radians(origin_lat) + dlat_rad
    lat = math.degrees(lat_rad)

    dlon_rad = x_m / (EARTH_RADIUS_M * math.cos(lat_rad))
    lon = origin_lon + math.degrees(dlon_rad)
    return round(lat, 6), round(lon, 6)


class SensorFusionEngine:
    def __init__(self, gps_gain=0.15, flow_weight=0.85):
        """
        :param gps_gain: Kalman / Complementary innovation gain for GPS update (0.05 to 0.3).
        :param flow_weight: Weight of optical flow velocity vs IMU acceleration integration.
        """
        self.gps_gain = gps_gain
        self.flow_weight = flow_weight

        # Fused position state (in local tangent plane meters relative to origin)
        self.fused_x_m = 0.0
        self.fused_y_m = 0.0
        self.fused_alt_m = 0.0

        # Fused velocity state (m/s)
        self.vx_east_m_s = 0.0
        self.vy_north_m_s = 0.0

        # Origin reference calibration
        self.origin_lat = None
        self.origin_lon = None
        self.origin_set = False

        # Raw sensor inputs cache
        self.heading_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0

        self.raw_gps_lat = None
        self.raw_gps_lon = None
        self.raw_gps_sats = 0
        self.raw_gps_status = "NO FIX"

        self.last_prediction_ts = None
        self.last_gps_update_ts = None

    def set_origin(self, lat, lon):
        """Sets or recalibrates the reference home origin."""
        if lat is not None and lon is not None:
            self.origin_lat = float(lat)
            self.origin_lon = float(lon)
            self.origin_set = True

    def update_attitude(self, roll_deg, pitch_deg, heading_deg=None):
        """Updates orientation angles from IMU and Compass."""
        if roll_deg is not None: self.roll_deg = roll_deg
        if pitch_deg is not None: self.pitch_deg = pitch_deg
        if heading_deg is not None: self.heading_deg = heading_deg

    def predict_flow_step(self, flow_vx_m_s, flow_vy_m_s, dt=0.02):
        """
        High-rate (50Hz) dead-reckoning prediction step.
        Rotates optical flow body frame velocities (vx, vy) into Earth North-East frame
        using Compass Heading (yaw angle psi) and integrates position.
        """
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        # Convert heading angle to radians
        psi_rad = math.radians(self.heading_deg)
        cos_p = math.cos(psi_rad)
        sin_p = math.sin(psi_rad)

        # Rotate body-frame optical flow velocities to Earth-frame (East, North)
        # Body X (Forward) -> rotated by heading
        # Body Y (Right)   -> rotated by heading
        v_east = flow_vx_m_s * cos_p + flow_vy_m_s * sin_p
        v_north = -flow_vx_m_s * sin_p + flow_vy_m_s * cos_p

        # Complementary velocity update
        self.vx_east_m_s = self.flow_weight * v_east + (1.0 - self.flow_weight) * self.vx_east_m_s
        self.vy_north_m_s = self.flow_weight * v_north + (1.0 - self.flow_weight) * self.vy_north_m_s

        # Integrate position
        self.fused_x_m += self.vx_east_m_s * dt
        self.fused_y_m += self.vy_north_m_s * dt
        self.last_prediction_ts = time.time()

    def update_gps(self, raw_lat, raw_lon, alt_m=0.0, speed_kmh=0.0, sats=0, fix_status=""):
        """
        Low-frequency (1 - 5Hz) GPS measurement update.
        Corrects dead-reckoning drift using GPS position innovation.
        """
        if raw_lat is None or raw_lon is None:
            return

        self.raw_gps_lat = float(raw_lat)
        self.raw_gps_lon = float(raw_lon)
        self.raw_gps_sats = int(sats)
        self.raw_gps_status = str(fix_status)
        self.fused_alt_m = float(alt_m)

        # Auto-initialize origin on first valid GPS fix
        if not self.origin_set:
            self.set_origin(self.raw_gps_lat, self.raw_gps_lon)
            self.fused_x_m = 0.0
            self.fused_y_m = 0.0
            self.last_gps_update_ts = time.time()
            return

        # Convert raw GPS position to local meters relative to origin
        gps_x_m, gps_y_m = geo_to_meters(self.raw_gps_lat, self.raw_gps_lon, self.origin_lat, self.origin_lon)

        # Calculate Innovation Residuals
        innov_x = gps_x_m - self.fused_x_m
        innov_y = gps_y_m - self.fused_y_m

        # Adaptive gain based on satellite count & fix quality
        gain = self.gps_gain
        if sats >= 6 and ("3D" in fix_status or "FIX" in fix_status):
            gain = min(0.3, self.gps_gain * 1.2)
        elif sats < 4:
            gain = 0.05  # Trust optical flow dead-reckoning more when GPS signal is weak

        # Apply Innovation Correction
        self.fused_x_m += gain * innov_x
        self.fused_y_m += gain * innov_y
        self.last_gps_update_ts = time.time()

    def get_fused_state(self):
        """
        Returns full state dictionary containing raw GPS, fused GPS coordinates,
        2D plane meters, heading, velocity, and accuracy metrics.
        """
        if self.origin_set and self.origin_lat is not None and self.origin_lon is not None:
            fused_lat, fused_lon = meters_to_geo(self.fused_x_m, self.fused_y_m, self.origin_lat, self.origin_lon)
        else:
            fused_lat = self.raw_gps_lat
            fused_lon = self.raw_gps_lon

        # Estimate position accuracy radius (meters) based on sensor agreement
        if self.raw_gps_lat is not None and self.origin_set:
            raw_x, raw_y = geo_to_meters(self.raw_gps_lat, self.raw_gps_lon, self.origin_lat, self.origin_lon)
            diff_m = math.sqrt((raw_x - self.fused_x_m) ** 2 + (raw_y - self.fused_y_m) ** 2)
            accuracy_radius_m = round(max(0.5, min(5.0, diff_m)), 2)
        else:
            accuracy_radius_m = 2.5

        return {
            "fused_lat": fused_lat,
            "fused_lon": fused_lon,
            "fused_x_m": round(self.fused_x_m, 3),
            "fused_y_m": round(self.fused_y_m, 3),
            "fused_alt_m": round(self.fused_alt_m, 2),
            "vx_east_m_s": round(self.vx_east_m_s, 2),
            "vy_north_m_s": round(self.vy_north_m_s, 2),
            "speed_kmh": round(math.sqrt(self.vx_east_m_s ** 2 + self.vy_north_m_s ** 2) * 3.6, 1),
            "heading_deg": round(self.heading_deg, 1),
            "accuracy_radius_m": accuracy_radius_m,
            "origin": {
                "lat": self.origin_lat,
                "lon": self.origin_lon
            } if self.origin_set else None,
            "raw_gps": {
                "lat": self.raw_gps_lat,
                "lon": self.raw_gps_lon,
                "satellites": self.raw_gps_sats,
                "fix_status": self.raw_gps_status
            }
        }
