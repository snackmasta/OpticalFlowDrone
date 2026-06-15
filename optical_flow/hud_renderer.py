import cv2
import numpy as np
import math
import time

from .sensor_readers import (
    distance_lock, distance_state,
    attitude_lock, attitude_state,
    accel_lock, accel_state,
    compass_lock, compass_state
)
from .flow_processor import velocity_state

RETICLE_COLOR = (0, 220, 220)
RETICLE_ROLL_SCALE_PX_PER_DEG = 4.5
RETICLE_PITCH_SCALE_PX_PER_DEG = 6.0


def compass_cardinal_from_heading_deg(heading_deg):
    directions = ["N", "NW", "W", "SW", "S", "SE", "E", "NE"]   
    normalized_heading_deg = heading_deg % 360.0
    index = int((normalized_heading_deg + 22.5) // 45.0) % 8
    return directions[index]


def draw_compass_widget(frame, heading_deg, age_s):
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


def draw_osd(frame):
    with distance_lock:
        current_distance = distance_state["current_distance"]
    with attitude_lock:
        roll_deg = attitude_state["roll_deg"]
        pitch_deg = attitude_state["pitch_deg"]
        yaw_deg = attitude_state["yaw_deg"]
        xgyro_dps = attitude_state["xgyro_dps"]
        ygyro_dps = attitude_state["ygyro_dps"]
        zgyro_dps = attitude_state["zgyro_dps"]
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
        f"DIST: {current_distance if current_distance is not None else 'N/A'} cm",
        # f"VX: {vx_mps:+.3f} m/s",
        # f"VY: {vy_mps:+.3f} m/s",
        # f"SPD: {speed_mps:.3f} m/s ({inliers} inliers)",
        f"GYRO: X:{xgyro_dps:+.1f} Y:{ygyro_dps:+.1f} Z:{zgyro_dps:+.1f} dps",
        f"ACCEL: X:{xaccel_g:+.2f}g Y:{yaccel_g:+.2f}g Z:{zaccel_g:+.2f}g",
        f"ROLL: {roll_deg:+.1f} deg",
        f"PITCH: {pitch_deg:+.1f} deg",
        f"YAW: {yaw_deg:+.1f} deg",
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
    frame = draw_imu_analysis_widget(
        frame,
        roll_deg,
        pitch_deg,
        yaw_deg,
        accel_roll_deg,
        accel_pitch_deg,
        accel_tilt_deg,
    )

    return frame


def draw_ground_reticle(frame):
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
