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
    reject_outlier_tracks,
    estimate_body_velocity_mps,
    velocity_state,
    feature_params,
    lk_params,
    TRACK_FEATURE_COUNT,
    FLOW_SCALE
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

# Warm up camera and detect initial features
old_frame = ensure_bgr(picam2.capture_array())
old_gray = to_small_gray(old_frame)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

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
    global old_gray, p0, output_sink
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

        tracked_count = 0
        if p0 is not None and len(p0) > 0:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            if p1 is not None and st is not None:
                good_new = p1[st.flatten() == 1]
                good_old = p0[st.flatten() == 1]
                tracked_count = len(good_new)
                inlier_old, inlier_new = reject_outlier_tracks(good_old, good_new)

                with distance_lock:
                    altitude_cm = distance_state["current_distance"]

                velocity_estimate = estimate_body_velocity_mps(
                    inlier_old,
                    inlier_new,
                    altitude_cm,
                    dt_s,
                    focal_length_x_px,
                    focal_length_y_px,
                )

                if velocity_estimate is not None:
                    vx_mps, vy_mps = velocity_estimate
                    velocity_state["vx_mps"] = vx_mps
                    velocity_state["vy_mps"] = vy_mps
                    velocity_state["speed_mps"] = float(math.hypot(vx_mps, vy_mps))
                    velocity_state["inliers"] = len(inlier_new)
                    velocity_state["last_update"] = frame_ts
                else:
                    velocity_state["vx_mps"] = 0.0
                    velocity_state["vy_mps"] = 0.0
                    velocity_state["speed_mps"] = 0.0
                    velocity_state["inliers"] = 0
                    velocity_state["last_update"] = frame_ts

                for new, old in zip(inlier_new, inlier_old):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    ax, by = int(a * draw_scale), int(b * draw_scale)
                    cx, dy = int(c * draw_scale), int(d * draw_scale)
                    img = cv2.line(img, (cx, dy), (ax, by), (0, 255, 0), 1)
                    img = cv2.circle(img, (ax, by), 3, (0, 0, 255), -1)

                if len(good_new) < TRACK_FEATURE_COUNT:
                    new_features = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                    if new_features is not None:
                        good_new = np.concatenate((good_new.reshape(-1, 2), new_features.reshape(-1, 2)), axis=0)

                if len(good_new) > TRACK_FEATURE_COUNT:
                    good_new = good_new[:TRACK_FEATURE_COUNT]

                if len(good_new) > 0:
                    p0 = good_new.reshape(-1, 1, 2)
                else:
                    p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
            else:
                p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        else:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)

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
