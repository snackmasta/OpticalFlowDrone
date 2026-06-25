# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink, smbus2

import cv2
import numpy as np
import time
import math
from picamera2 import Picamera2

from optical_flow.sensor_readers import (
    start_distance_sensor_reader,
    distance_lock,
    distance_state
)
from optical_flow.flow_processor import (
    to_small_gray,
    ensure_bgr,
    focal_length_px,
    estimate_dense_flow_and_motion,
    velocity_state,
    FLOW_SCALE,
    position_state,
    position_lock
)
from optical_flow.hud_renderer import (
    draw_ground_reticle,
    draw_osd
)
from optical_flow.video_sinks import (
    FileSink,
    create_output_sink
)

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS

# Initialize camera
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
try:
    frame_duration_us = int(1_000_000 / TARGET_FPS)
    camera_config['controls']['FrameDurationLimits'] = (frame_duration_us, frame_duration_us)
except Exception:
    pass
picam2.configure(camera_config)
picam2.start()
time.sleep(2)

# Warm up camera and capture initial frame
old_frame = ensure_bgr(picam2.capture_array())
old_gray = to_small_gray(old_frame)

# Start sensor reader threads
start_distance_sensor_reader()

frame_height, frame_width = old_frame.shape[:2]
focal_length_x_px = focal_length_px(frame_width)
focal_length_y_px = focal_length_x_px

output_sink, rtsp_selected, rtsp_server = create_output_sink((frame_width, frame_height), TARGET_FPS)
fallback_sink = None
print("Press Ctrl+C to stop")


def switch_to_file_sink(frame_size, fps, reason):
    global fallback_sink, output_sink, rtsp_selected, rtsp_server
    if fallback_sink is None:
        print(reason)
        fallback_sink = FileSink(frame_size, fps)
    output_sink = fallback_sink
    rtsp_selected = False


def record_optical_flow():
    global old_gray, output_sink
    next_frame_time = time.perf_counter()
    previous_frame_ts = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now < next_frame_time:
            time.sleep(next_frame_time - now)
        frame_ts = time.perf_counter()
        dt_s = frame_ts - previous_frame_ts
        previous_frame_ts = frame_ts
        next_frame_time = frame_ts + FRAME_INTERVAL_S

        frame = ensure_bgr(picam2.capture_array())
        frame_gray = to_small_gray(frame)
        img = frame.copy()
        draw_scale = 1.0 / FLOW_SCALE

        dense_motion = estimate_dense_flow_and_motion(old_gray, frame_gray, step=8)

        tracked_count = 0
        if dense_motion is not None:
            tx, ty, scale, theta, inlier_old, inlier_new = dense_motion
            tracked_count = len(inlier_new)

            with distance_lock:
                altitude_cm = distance_state["current_distance"]

            # Calculate physical velocity using RANSAC-fitted translations tx, ty
            altitude_m = altitude_cm / 100.0
            vx_mps = (tx * altitude_m) / (focal_length_x_px * dt_s)
            vy_mps = (ty * altitude_m) / (focal_length_y_px * dt_s)

            velocity_state["vx_mps"] = vx_mps
            velocity_state["vy_mps"] = vy_mps
            velocity_state["speed_mps"] = float(math.hypot(vx_mps, vy_mps))
            velocity_state["inliers"] = tracked_count
            velocity_state["last_update"] = frame_ts

            with position_lock:
                position_state["x_m"] += vx_mps * dt_s
                position_state["y_m"] += vy_mps * dt_s
                position_state["path"].append((position_state["x_m"], position_state["y_m"]))
                if len(position_state["path"]) > 200:
                    position_state["path"].pop(0)

            # Draw inlier vectors
            for new, old in zip(inlier_new, inlier_old):
                a, b = new.ravel()
                c, d = old.ravel()
                ax, by = int(a * draw_scale), int(b * draw_scale)
                cx, dy = int(c * draw_scale), int(d * draw_scale)
                img = cv2.line(img, (cx, dy), (ax, by), (0, 255, 0), 1)
                img = cv2.circle(img, (ax, by), 3, (0, 0, 255), -1)
        else:
            velocity_state["vx_mps"] = 0.0
            velocity_state["vy_mps"] = 0.0
            velocity_state["speed_mps"] = 0.0
            velocity_state["inliers"] = 0
            velocity_state["last_update"] = frame_ts

            with position_lock:
                position_state["path"].append((position_state["x_m"], position_state["y_m"]))
                if len(position_state["path"]) > 200:
                    position_state["path"].pop(0)

        old_gray = frame_gray.copy()
        img = draw_ground_reticle(img)
        img = draw_osd(img, tracked_count)
        try:
            output_sink.write(img)
        except RuntimeError as exc:
            if rtsp_selected:
                switch_to_file_sink((frame_width, frame_height), TARGET_FPS, f"RTSP failed, falling back to local recording: {exc}")
                output_sink.write(img)
            else:
                raise

if __name__ == '__main__':
    try:
        record_optical_flow()
    except KeyboardInterrupt:
        print("\nStopping output...")
    finally:
        output_sink.release()
        if rtsp_server is not None:
            rtsp_server.release()
        picam2.stop()
