"""
Modul Visualisasi OSD & Render HUD (On-Screen Display Renderer)
================================================================
Deskripsi:
    Modul ini bertanggung jawab untuk merender elemen grafis Head-Up Display (HUD) 
    dan On-Screen Display (OSD) secara real-time pada frame video. 
    Menampilkan data telemetri penerbangan, attitude (roll, pitch, heading), 
    kecepatan spasial, serta pergerakan Optical Flow.

Fitur Utama:
    1. Reticle Horizon Buatan (Ground Reticle):
       - Visualisasi proyeksi horizon tanah berbasis sudut roll dan pitch IMU.
    2. Widget Kompas (Compass Rose):
       - Visualisasi kompas analog/digital dengan penunjuk arah mata angin (N, NW, W, dsb.) dan indikator kebaruan data.
    3. Render OSD & Overlays Telemetri (`draw_osd`):
       - Indikator pitch ladder & roll arc.
       - Tampilan numerik kecepatan linier (vx, vy, vz), altitude/jarak, dan koordinat pergeseran posisi.
       - Visualisasi vektor pergerakan Optical Flow dan jejak lintasan (path tracking).
       - Status kalibrasi sensor, jumlah inliers, dan peringatan batas geofence.
"""

import cv2
import numpy as np
import math
import time

from .sensor_readers import (
    attitude_lock, attitude_state,
    accel_lock, accel_state,
    compass_lock, compass_state,
    gyro_integrated_lock, gyro_integrated_state,
    normalize_angle_deg
)
from .flow_processor import velocity_state, position_state, position_lock

RETICLE_COLOR = (0, 220, 220)
RETICLE_ROLL_SCALE_PX_PER_DEG = 4.5
RETICLE_PITCH_SCALE_PX_PER_DEG = 6.0


def compass_cardinal_from_heading_deg(heading_deg):
    """
    Converts a numerical heading in degrees into a cardinal/ordinal string representation.
    """
    directions = ["N", "NW", "W", "SW", "S", "SE", "E", "NE"]   
    normalized_heading_deg = heading_deg % 360.0
    index = int((normalized_heading_deg + 22.5) // 45.0) % 8
    return directions[index]


def draw_compass_widget(frame, heading_deg, age_s):
    """
    Draws a visual compass rose widget on the frame showing current heading.
    """
    h, w = frame.shape[:2]
    radius_outer = max(22, min(w, h) // 12)
    radius_inner = max(8, radius_outer // 2)
    center = (w - radius_outer - 22, radius_outer + 22)

    compass_overlay = frame.copy()
    cv2.circle(compass_overlay, center, radius_outer, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.circle(compass_overlay, center, radius_inner, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] - radius_outer, center[1]), (center[0] + radius_outer, center[1]), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0], center[1] - radius_outer), (center[0], center[1] + radius_outer), (255, 255, 255), 1, cv2.LINE_AA)

    tick = max(4, radius_outer // 5)
    cv2.line(compass_overlay, (center[0], center[1] - radius_outer), (center[0], center[1] - radius_outer - tick), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] + radius_outer, center[1]), (center[0] + radius_outer + tick, center[1]), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0], center[1] + radius_outer), (center[0], center[1] + radius_outer + tick), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(compass_overlay, (center[0] - radius_outer, center[1]), (center[0] - radius_outer - tick, center[1]), (255, 255, 255), 1, cv2.LINE_AA)

    if heading_deg is not None:
        heading_rad = math.radians(-heading_deg)
        needle_length = radius_outer - 4
        needle_end = (
            int(center[0] + math.sin(heading_rad) * needle_length),
            int(center[1] - math.cos(heading_rad) * needle_length),
        )
        cv2.line(compass_overlay, center, needle_end, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(compass_overlay, center, 3, (0, 0, 255), -1, cv2.LINE_AA)

    font_scale = 0.45
    cv2.putText(compass_overlay, "N", (center[0] - 6, center[1] - radius_outer - 6), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "E", (center[0] + radius_outer + 4, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "S", (center[0] - 6, center[1] + radius_outer + 14), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(compass_overlay, "W", (center[0] - radius_outer - 14, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 1, cv2.LINE_AA)

    if heading_deg is not None:
        heading_360 = heading_deg % 360.0
        cardinal = compass_cardinal_from_heading_deg(heading_360)
        label = f"{cardinal} {heading_360:05.1f}"
    else:
        label = "COMPASS"
    if age_s is not None:
        label = f"{label} {age_s:.1f}s"
    cv2.putText(compass_overlay, label, (center[0] - radius_outer, center[1] + radius_outer + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)

    return cv2.addWeighted(frame, 1.0, compass_overlay, 0.85, 0)


def draw_imu_analysis_widget(frame, imu_roll_deg, imu_pitch_deg, imu_yaw_deg, accel_roll_deg, accel_pitch_deg, accel_tilt_deg):
    """
    Draws a sidebar widget showing detailed comparison bars between Gyro IMU and Accel angles.
    """
    h, w = frame.shape[:2]
    box_w = min(300, max(220, int(w * 0.34)))
    box_h = 166
    x0 = w - box_w - 14
    y0 = min(max(130, h // 5), h - box_h - 14)

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    cv2.putText(frame, "IMU / ACCEL ANALYSIS", (x0 + 10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

    def draw_signed_bar(label, value_deg, row, color):
        """
        Inner helper to draw a horizontal progress bar centered at zero for signed values.
        """
        y = y0 + 38 + (row * 22)
        bar_x0 = x0 + 104
        bar_x1 = x0 + box_w - 12
        center_x = (bar_x0 + bar_x1) // 2
        span = (bar_x1 - bar_x0) // 2
        v = float(np.clip(value_deg, -180.0, 180.0))
        end_x = int(center_x + (v / 180.0) * span)

        cv2.putText(frame, f"{label} {value_deg:+6.1f}", (x0 + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, y), (bar_x1, y), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y - 4), (center_x, y + 4), (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y), (end_x, y), color, 2, cv2.LINE_AA)

    draw_signed_bar("IMU R", imu_roll_deg, 0, (0, 180, 255))
    draw_signed_bar("IMU P", imu_pitch_deg, 1, (0, 180, 255))
    draw_signed_bar("IMU Y", imu_yaw_deg, 2, (0, 180, 255))
    draw_signed_bar("ACC R", accel_roll_deg, 3, (0, 255, 255))
    draw_signed_bar("ACC P", accel_pitch_deg, 4, (0, 255, 255))
    draw_signed_bar("A TLT", accel_tilt_deg, 5, (0, 255, 140))

    return frame


def draw_osd(frame, total_tracked=0):
    """
    Overlays a HUD dashboard OSD (On-Screen Display) with telemetry widgets, 
    minimap, compass, and analysis readouts on top of the video frame.
    """
    with attitude_lock:
        roll_deg = attitude_state["roll_deg"]
        pitch_deg = attitude_state["pitch_deg"]
        yaw_deg = attitude_state["yaw_deg"]
        xgyro_dps = attitude_state["xgyro_dps"]
        ygyro_dps = attitude_state["ygyro_dps"]
        zgyro_dps = attitude_state["zgyro_dps"]
    with gyro_integrated_lock:
        int_roll = gyro_integrated_state["roll_deg"]
        int_pitch = gyro_integrated_state["pitch_deg"]
        int_yaw = gyro_integrated_state["yaw_deg"]
    with accel_lock:
        xaccel_g = accel_state["x_g"]
        yaccel_g = accel_state["y_g"]
        zaccel_g = accel_state["z_g"]
        accel_roll_deg = accel_state["roll_deg"]
        accel_pitch_deg = accel_state["pitch_deg"]
        accel_tilt_deg = accel_state["tilt_deg"]
    with compass_lock:
        compass_heading_deg = compass_state["heading_deg"]
        compass_timestamp = compass_state["timestamp"]

    vx_mps = velocity_state["vx_mps"]
    vy_mps = velocity_state["vy_mps"]
    speed_mps = velocity_state["speed_mps"]
    inliers = velocity_state["inliers"]

    lines = [
        # f"VX: {vx_mps:+.3f} m/s",
        # f"VY: {vy_mps:+.3f} m/s",
        # f"SPD: {speed_mps:.3f} m/s ({inliers} inliers)",
        # f"GYRO: X:{xgyro_dps:+.1f} Y:{ygyro_dps:+.1f} Z:{zgyro_dps:+.1f} dps",
        # f"ACCEL: X:{xaccel_g:+.2f}g Y:{yaccel_g:+.2f}g Z:{zaccel_g:+.2f}g",
        # f"ROLL: {roll_deg:+.1f} deg",
        # f"PITCH: {pitch_deg:+.1f} deg",
        # f"YAW: {yaw_deg:+.1f} deg",
    ]

    overlay = frame.copy()
    x, y = 12, 28
    box_w = 0
    for line in lines:
        (text_w, text_h), baseline = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        box_w = max(box_w, text_w)

    box_h = 18 * len(lines) + 14
    cv2.rectangle(overlay, (8, 8), (20 + box_w, 12 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    for line in lines:
        # highlight attitude values with brighter color for quick visual cue
        color = (0, 255, 0)
        if line.startswith("ROLL") or line.startswith("PITCH") or line.startswith("YAW"):
            color = (0, 200, 255)
        elif line.startswith("ACCEL"):
            color = (255, 200, 0)
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        y += 18

    compass_age_s = None
    if compass_timestamp:
        compass_age_s = max(0.0, time.time() - compass_timestamp)
    frame = draw_compass_widget(frame, compass_heading_deg, compass_age_s)
    frame = draw_raw_sensor_widget(
        frame,
        xgyro_dps,
        ygyro_dps,
        zgyro_dps,
        xaccel_g,
        yaccel_g,
        zaccel_g,
    )
    frame = draw_gyro_drift_widget(
        frame,
        int_roll,
        int_pitch,
        int_yaw,
        roll_deg,
        pitch_deg,
        yaw_deg,
    )
    frame = draw_feature_track_widget(
        frame,
        total_tracked,
    )
    frame = draw_imu_analysis_widget(
        frame,
        roll_deg,
        pitch_deg,
        yaw_deg,
        accel_roll_deg,
        accel_pitch_deg,
        accel_tilt_deg,
    )
    frame = draw_minimap_widget(frame)

    return frame


def draw_raw_sensor_widget(frame, gx, gy, gz, ax, ay, az):
    """
    Draws a sidebar widget showing raw values for Gyroscope (dps) and Accelerometer (g) axes.
    """
    h, w = frame.shape[:2]
    box_w = min(300, max(220, int(w * 0.34)))
    box_h = 166
    x0 = 14
    y0 = 144

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    cv2.putText(frame, "RAW IMU & ACCEL", (x0 + 10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

    def draw_signed_bar(label, value, max_val, row, color):
        """
        Inner helper to draw a signed raw sensor value bar.
        """
        y = y0 + 38 + (row * 22)
        bar_x0 = x0 + 104
        bar_x1 = x0 + box_w - 12
        center_x = (bar_x0 + bar_x1) // 2
        span = (bar_x1 - bar_x0) // 2
        v = float(np.clip(value, -max_val, max_val))
        end_x = int(center_x + (v / max_val) * span)

        cv2.putText(frame, f"{label} {value:+6.2f}", (x0 + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, y), (bar_x1, y), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y - 4), (center_x, y + 4), (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y), (end_x, y), color, 2, cv2.LINE_AA)

    draw_signed_bar("GYR X", gx, 250.0, 0, (0, 255, 0))
    draw_signed_bar("GYR Y", gy, 250.0, 1, (0, 255, 0))
    draw_signed_bar("GYR Z", gz, 250.0, 2, (0, 255, 0))
    draw_signed_bar("ACC X", ax, 2.0, 3, (255, 100, 100))
    draw_signed_bar("ACC Y", ay, 2.0, 4, (255, 100, 100))
    draw_signed_bar("ACC Z", az, 2.0, 5, (255, 100, 100))

    return frame


def draw_gyro_drift_widget(frame, int_roll, int_pitch, int_yaw, flt_roll, flt_pitch, flt_yaw):
    """
    Draws a widget analyzing drift of integrated raw gyro angles compared to complementary-filtered angles.
    """
    h, w = frame.shape[:2]
    box_w = min(300, max(220, int(w * 0.34)))
    box_h = 166
    x0 = 14
    y0 = 310

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    cv2.putText(frame, "GYRO INTEGRATION DRIFT", (x0 + 10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

    drift_roll = normalize_angle_deg(int_roll - flt_roll)
    drift_pitch = normalize_angle_deg(int_pitch - flt_pitch)
    drift_yaw = normalize_angle_deg(int_yaw - flt_yaw)

    def draw_signed_bar(label, value, max_val, row, color):
        """
        Inner helper to draw a signed drift comparison bar.
        """
        y = y0 + 38 + (row * 22)
        bar_x0 = x0 + 104
        bar_x1 = x0 + box_w - 12
        center_x = (bar_x0 + bar_x1) // 2
        span = (bar_x1 - bar_x0) // 2
        v = float(np.clip(value, -max_val, max_val))
        end_x = int(center_x + (v / max_val) * span)

        cv2.putText(frame, f"{label} {value:+6.1f}", (x0 + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, y), (bar_x1, y), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y - 4), (center_x, y + 4), (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y), (end_x, y), color, 2, cv2.LINE_AA)

    draw_signed_bar("INT R", int_roll, 180.0, 0, (255, 0, 255))
    draw_signed_bar("DFT R", drift_roll, 45.0, 1, (0, 180, 255))
    draw_signed_bar("INT P", int_pitch, 180.0, 2, (255, 0, 255))
    draw_signed_bar("DFT P", drift_pitch, 45.0, 3, (0, 180, 255))
    draw_signed_bar("INT Y", int_yaw, 180.0, 4, (255, 0, 255))
    draw_signed_bar("DFT Y", drift_yaw, 45.0, 5, (0, 180, 255))

    return frame


def draw_feature_track_widget(frame, total_tracked):
    """
    Draws a widget showing track counts, inliers, outliers, inlier percentage, speed, and axis velocities.
    """
    h, w = frame.shape[:2]
    box_w = min(300, max(220, int(w * 0.34)))
    box_h = 166
    x0 = w - box_w - 14
    y0 = 310

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    cv2.putText(frame, "FEATURE TRACK ANALYSIS", (x0 + 10, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 220), 1, cv2.LINE_AA)

    vx_mps = velocity_state["vx_mps"]
    vy_mps = velocity_state["vy_mps"]
    speed_mps = velocity_state["speed_mps"]
    inliers = velocity_state["inliers"]
    outliers = max(0, total_tracked - inliers)
    inlier_ratio = (inliers / max(1, total_tracked)) * 100.0

    def draw_label_value_bar(label, value, value_str, max_val, row, color):
        """
        Inner helper to draw labeled feature tracking progress bars.
        """
        y = y0 + 38 + (row * 22)
        bar_x0 = x0 + 104
        bar_x1 = x0 + box_w - 12
        center_x = (bar_x0 + bar_x1) // 2
        span = (bar_x1 - bar_x0) // 2
        v = float(np.clip(value, -max_val, max_val))
        end_x = int(center_x + (v / max_val) * span)

        cv2.putText(frame, f"{label} {value_str}", (x0 + 10, y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1, cv2.LINE_AA)
        cv2.line(frame, (bar_x0, y), (bar_x1, y), (110, 110, 110), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y - 4), (center_x, y + 4), (150, 150, 150), 1, cv2.LINE_AA)
        cv2.line(frame, (center_x, y), (end_x, y), color, 2, cv2.LINE_AA)

    draw_label_value_bar("TRACK", total_tracked, f"{total_tracked:>6d}", 20.0, 0, (0, 255, 255))
    draw_label_value_bar("INLRS", inliers, f"{inliers:>6d}", 20.0, 1, (0, 255, 0))
    draw_label_value_bar("OUTLR", outliers, f"{outliers:>6d}", 20.0, 2, (100, 100, 255))
    draw_label_value_bar("INL %", inlier_ratio, f"{inlier_ratio:5.1f}%", 100.0, 3, (0, 255, 0))
    draw_label_value_bar("SPEED", speed_mps, f"{speed_mps:6.2f}", 5.0, 4, (255, 255, 0))
    draw_label_value_bar("VEL X", vx_mps, f"{vx_mps:+6.2f}", 5.0, 5, (255, 150, 0))

    return frame


def draw_minimap_widget(frame):
    """
    Draws a 2D positioning minimap at the bottom center of the frame showing
    historical path tracking of the drone from optical flow.
    """
    h, w = frame.shape[:2]
    box_w = 160
    box_h = 160
    x0 = w // 2 - box_w // 2
    y0 = h - box_h - 14
    center_x = x0 + box_w // 2
    center_y = y0 + box_h // 2

    # Draw background box
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.48, frame, 0.52, 0)

    # Draw border
    cv2.rectangle(frame, (x0, y0), (x0 + box_w, y0 + box_h), (110, 110, 110), 1, cv2.LINE_AA)

    # Draw grid/crosshairs
    cv2.line(frame, (center_x, y0), (center_x, y0 + box_h), (60, 60, 60), 1, cv2.LINE_AA)
    cv2.line(frame, (x0, center_y), (x0 + box_w, center_y), (60, 60, 60), 1, cv2.LINE_AA)

    # Scale: 250 pixels per meter (box covers +/- 32cm)
    pixels_per_meter = 250.0

    # Draw range rings at 10cm, 20cm, 30cm (0.1m, 0.2m, 0.3m)
    for r_m in [0.1, 0.2, 0.3]:
        r_px = int(r_m * pixels_per_meter)
        cv2.circle(frame, (center_x, center_y), r_px, (70, 70, 70), 1, cv2.LINE_AA)

    # Retrieve position trail
    with position_lock:
        x_curr = position_state["x_cm"] / 100.0
        y_curr = position_state["y_cm"] / 100.0
        path = [(xp / 100.0, yp / 100.0) for xp, yp in position_state["path"]]

    # Draw trail
    points = []
    for xp, yp in path:
        dx = xp - x_curr
        dy = yp - y_curr
        px = int(center_x + dx * pixels_per_meter)
        py = int(center_y - dy * pixels_per_meter)
        # Clip points to stay inside the box
        px = int(np.clip(px, x0 + 1, x0 + box_w - 1))
        py = int(np.clip(py, y0 + 1, y0 + box_h - 1))
        points.append((px, py))

    # Connect trail with lines
    for i in range(len(points) - 1):
        # Fade color based on age
        alpha = int(255 * (i / len(points)))
        color = (0, alpha, 0)
        cv2.line(frame, points[i], points[i+1], color, 1, cv2.LINE_AA)

    # Current position cursor
    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1, cv2.LINE_AA)

    # Text annotations
    cv2.putText(frame, "MINIMAP", (x0 + 6, y0 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(frame, f"X:{x_curr:+.2f}", (x0 + 6, y0 + box_h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, f"Y:{y_curr:+.2f}", (x0 + 6, y0 + box_h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(frame, "GRID: 10cm", (x0 + box_w - 75, y0 + box_h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1, cv2.LINE_AA)

    return frame


def draw_ground_reticle(frame):
    """
    Draws a dynamic central ground HUD reticle warped by current roll, pitch, and yaw.
    Also overlays an accelerometer-driven crosshair vector showing force direction.
    """
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2
    radius_outer = max(24, min(w, h) // 7)
    radius_inner = max(12, radius_outer // 2)
    arm = max(16, radius_outer // 2)

    with attitude_lock:
        roll_deg = attitude_state["roll_deg"]
        pitch_deg = attitude_state["pitch_deg"]
        yaw_deg = attitude_state["yaw_deg"]
    with accel_lock:
        xaccel_g = accel_state["x_g"]
        yaccel_g = accel_state["y_g"]

    # plane-style HUD motion: stronger pitch and roll offsets than the previous tuning
    pitch_px = int(np.clip(-pitch_deg * RETICLE_PITCH_SCALE_PX_PER_DEG, -h * 0.35, h * 0.35))
    roll_px = int(np.clip(roll_deg * RETICLE_ROLL_SCALE_PX_PER_DEG, -w * 0.35, w * 0.35))
    center = (cx + roll_px, cy + pitch_px)

    overlay = frame.copy()
    rotation_matrix = cv2.getRotationMatrix2D(center, -yaw_deg, 1.0)

    reticle = np.zeros_like(frame)
    cv2.circle(reticle, center, radius_outer, RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.circle(reticle, center, radius_inner, RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] - radius_outer - arm, center[1]), (center[0] + radius_outer + arm, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0], center[1] - radius_outer - arm), (center[0], center[1] + radius_outer + arm), RETICLE_COLOR, 1, cv2.LINE_AA)

    tick = max(6, radius_outer // 5)
    cv2.line(reticle, (center[0], center[1] - radius_outer), (center[0], center[1] - radius_outer - tick), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] + radius_outer, center[1]), (center[0] + radius_outer + tick, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0], center[1] + radius_outer), (center[0], center[1] + radius_outer + tick), RETICLE_COLOR, 1, cv2.LINE_AA)
    cv2.line(reticle, (center[0] - radius_outer, center[1]), (center[0] - radius_outer - tick, center[1]), RETICLE_COLOR, 1, cv2.LINE_AA)

    reticle = cv2.warpAffine(reticle, rotation_matrix, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
    frame = cv2.addWeighted(frame, 1.0, reticle, 0.9, 0)

    arrow_scale = 45.0
    arrow_dx = int(np.clip(xaccel_g * arrow_scale, -w * 0.25, w * 0.25))
    arrow_dy = int(np.clip(-yaccel_g * arrow_scale, -h * 0.25, h * 0.25))
    arrow_end = (cx + arrow_dx, cy + arrow_dy)
    cv2.arrowedLine(frame, (cx, cy), arrow_end, (255, 200, 0), 3, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(frame, (cx, cy), 4, (255, 200, 0), -1, cv2.LINE_AA)

    return frame
