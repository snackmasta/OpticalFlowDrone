# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink, smbus2

import cv2
import numpy as np
import time
import math
import struct
import threading
from multiprocessing import shared_memory
from picamera2 import Picamera2

from optical_flow.sensor_readers import (
    start_distance_sensor_reader,
    distance_lock,
    distance_state,
    attitude_lock,
    attitude_state
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
    draw_osd,
    RETICLE_ROLL_SCALE_PX_PER_DEG,
    RETICLE_PITCH_SCALE_PX_PER_DEG
)
from optical_flow.video_sinks import (
    FileSink,
    create_output_sink
)

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS

# Shared memory configuration for optical flow stream
SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<10d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
SHM_SIZE = SHM_HEADER_SIZE + (MAX_SAMPLES * SHM_RECORD_SIZE)

flow_shm = None
flow_shm_lock = threading.Lock()


def attach_flow_stream_shm():
    global flow_shm
    with flow_shm_lock:
        if flow_shm is not None:
            return flow_shm
        try:
            flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        except FileExistsError:
            flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
            if flow_shm.size < SHM_SIZE:
                flow_shm.close()
                try:
                    flow_shm.unlink()
                except FileNotFoundError:
                    pass
                flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)

        struct.pack_into(SHM_HEADER_FORMAT, flow_shm.buf, 0, SHM_MAGIC, 0, 0)
        return flow_shm


def write_flow_stream_sample(timestamp, x, y, x_raw, y_raw, vx, vy, vx_raw, vy_raw, alt):
    try:
        shm = attach_flow_stream_shm()
        with flow_shm_lock:
            _, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
            record_offset = SHM_HEADER_SIZE + (write_index * SHM_RECORD_SIZE)
            struct.pack_into(
                SHM_RECORD_FORMAT,
                shm.buf,
                record_offset,
                float(timestamp),
                float(x),
                float(y),
                float(x_raw),
                float(y_raw),
                float(vx),
                float(vy),
                float(vx_raw),
                float(vy_raw),
                float(alt),
            )
            write_index = (write_index + 1) % MAX_SAMPLES
            sample_count = min(sample_count + 1, MAX_SAMPLES)
            struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)
    except Exception as exc:
        print(f"Failed to write to shared memory: {exc}")


def close_flow_stream_shm():
    global flow_shm
    with flow_shm_lock:
        if flow_shm is None:
            return
        try:
            flow_shm.close()
        finally:
            try:
                flow_shm.unlink()
            except FileNotFoundError:
                pass
            flow_shm = None


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
    prev_roll_px = None
    prev_pitch_px = None
    x_raw_m = 0.0
    y_raw_m = 0.0
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

        # Calculate reticle displacement for tilt compensation
        with attitude_lock:
            roll_deg = attitude_state["roll_deg"]
            pitch_deg = attitude_state["pitch_deg"]

        roll_px = np.clip(roll_deg * RETICLE_ROLL_SCALE_PX_PER_DEG, -frame_width * 0.35, frame_width * 0.35)
        pitch_px = np.clip(-pitch_deg * RETICLE_PITCH_SCALE_PX_PER_DEG, -frame_height * 0.35, frame_height * 0.35)

        if prev_roll_px is None:
            prev_roll_px = roll_px
            prev_pitch_px = pitch_px

        d_reticle_x = roll_px - prev_roll_px
        d_reticle_y = pitch_px - prev_pitch_px

        prev_roll_px = roll_px
        prev_pitch_px = pitch_px

        tracked_count = 0
        vx_raw_mps = 0.0
        vy_raw_mps = 0.0
        if dense_motion is not None:
            tx, ty, scale, theta, inlier_old, inlier_new = dense_motion
            tracked_count = len(inlier_new)

            # Apply reticle-based tilt compensation (opposite to apparent motion of features)
            tx_comp = tx + (d_reticle_x * FLOW_SCALE)
            ty_comp = ty + (d_reticle_y * FLOW_SCALE)

            with distance_lock:
                altitude_cm = distance_state["current_distance"]

            # Calculate physical velocity using compensated translations
            altitude_m = (altitude_cm / 100.0) if altitude_cm is not None else 1.5
            vx_mps = (tx_comp * altitude_m) / (focal_length_x_px * dt_s)
            vy_mps = (ty_comp * altitude_m) / (focal_length_y_px * dt_s)

            # Calculate raw physical velocity (uncompensated)
            vx_raw_mps = (tx * altitude_m) / (focal_length_x_px * dt_s)
            vy_raw_mps = (ty * altitude_m) / (focal_length_y_px * dt_s)

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

            x_raw_m += vx_raw_mps * dt_s
            y_raw_m += vy_raw_mps * dt_s

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

        # Stream current X, Y position, velocity, and altitude to shared memory
        with position_lock:
            current_x = position_state["x_m"]
            current_y = position_state["y_m"]
        current_vx = velocity_state["vx_mps"]
        current_vy = velocity_state["vy_mps"]
        with distance_lock:
            altitude_cm = distance_state["current_distance"]
        current_alt = (altitude_cm / 100.0) if altitude_cm is not None else 1.5
        write_flow_stream_sample(
            frame_ts,
            current_x,
            current_y,
            x_raw_m,
            y_raw_m,
            current_vx,
            current_vy,
            vx_raw_mps,
            vy_raw_mps,
            current_alt
        )

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
        close_flow_stream_shm()

