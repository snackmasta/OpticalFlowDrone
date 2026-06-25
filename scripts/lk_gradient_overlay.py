#!/usr/bin/env python3
"""
Lucas-Kanade Optical Flow with Spatial/Motion Gradients and Flight Instrument HUD.
Headless / Streaming Edition (No Qt/GUI dependencies).

Features:
- Dual-source input pipeline: Picamera2 (for Raspberry Pi drone) or standard cv2.VideoCapture (webcam).
- Full integration with optical_flow HUD renderer (draw_ground_reticle, draw_osd) for standard drone telemetry UI.
- Direct output pipeline integration with optical_flow.video_sinks (Local MP4 recording or RTSP streaming).
- Auto-cycling visual modes or command line selection to showcase all overlays without needing a GUI window.
- Background sensor reader threads (IMU, Compass, MAVLink distance sensor) with simulation fallback for desk-testing.
"""

import cv2
import numpy as np
import time
import math
import sys
import os
import argparse

# Add parent directory of the script to sys.path to allow importing optical_flow package
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Import core drone optical flow modules
from optical_flow.sensor_readers import (
    start_distance_sensor_reader,
    distance_lock,
    distance_state,
    attitude_lock,
    attitude_state,
    accel_lock,
    accel_state,
    compass_lock,
    compass_state
)
from optical_flow.flow_processor import (
    to_small_gray,
    ensure_bgr,
    focal_length_px,
    reject_outlier_tracks,
    estimate_body_velocity_mps,
    velocity_state,
    position_state,
    position_lock,
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

class GradientFlowVisualizer:
    def __init__(self, initial_mode=0):
        self.mode = initial_mode  # 0: Camera + Flow, 1: Sobel Gradient + Flow, 2: Blended Glow + Flow, 3: Motion Vector Field
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Color palette for custom flow vectors (BGR)
        self.color_cyan = (220, 220, 0)
        self.color_hud_bg = (15, 10, 10)

    def get_flow_color(self, angle_rad, magnitude, max_mag=15.0):
        """Map flow angle and magnitude to a beautiful BGR color using HSV."""
        angle_deg = math.degrees(angle_rad) % 360
        hue = int(angle_deg / 2)  # OpenCV Hue is 0-179
        val = int(min(255, (magnitude / max_mag) * 200 + 55))
        hsv = np.uint8([[[hue, 255, val]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
        return tuple(int(c) for c in bgr)

    def compute_sobel_gradient(self, gray):
        """Compute visual spatial image gradient magnitude."""
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        magnitude = np.uint8(np.clip(magnitude, 0, 255))
        return magnitude

    def draw_interactive_help(self, frame):
        """Draw an elegant mode toggle notification at the top-center of the screen."""
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Current gradient mode notification
        mode_names = [
            "GRADIENT MODE: CAMERA + SPARSE FLOW",
            "GRADIENT MODE: SOBEL EDGE INTENSITY MAP",
            "GRADIENT MODE: HYBRID SPATIAL GLOW",
            "GRADIENT MODE: DENSE MOTION FIELD MAP"
        ]
        
        # Render clean notification box
        help_text = f"ACTIVE: {mode_names[self.mode]} (AUTO-CYCLING ACTIVE)"
        (tw, th), _ = cv2.getTextSize(help_text, font, 0.38, 1)
        bx, by = (w - tw) // 2, 85
        
        cv2.rectangle(frame, (bx - 10, by - 14), (bx + tw + 10, by + 6), (15, 10, 10), -1)
        cv2.rectangle(frame, (bx - 10, by - 14), (bx + tw + 10, by + 6), (100, 100, 120), 1, cv2.LINE_AA)
        cv2.putText(frame, help_text, (bx, by - 2), font, 0.38, self.color_cyan, 1, cv2.LINE_AA)
        return frame


def run_pipeline(initial_mode, cycle_interval, target_fps):
    print("[INFO] Initializing Lucas-Kanade Headless Gradient Engine...")
    
    # Start sensor reader threads
    start_distance_sensor_reader()
    
    # Try to initialize camera
    cap = None
    is_picamera = False
    
    # Try Picamera2 first
    try:
        from picamera2 import Picamera2
        print("[INFO] Attempting to connect to Picamera2...")
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration()
        try:
            frame_duration_us = int(1_000_000 / target_fps)
            camera_config['controls']['FrameDurationLimits'] = (frame_duration_us, frame_duration_us)
        except Exception:
            pass
        picam2.configure(camera_config)
        picam2.start()
        is_picamera = True
        print("[SUCCESS] Connected to Picamera2.")
    except Exception as e:
        print(f"[INFO] Picamera2 not available: {e}. Falling back to standard webcam.")
        
    if not is_picamera:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[WARNING] Primary camera (index 0) failed to open. Trying index 1...")
            cap = cv2.VideoCapture(1)
            
        if not cap.isOpened():
            print("[ERROR] No hardware camera detected. Generating synthetic drone target stream.")
            cap = None

    # Wait for sensor/camera warmup
    time.sleep(2.0)
    
    # Capture first frame
    if is_picamera:
        old_frame = ensure_bgr(picam2.capture_array())
    elif cap is not None:
        ret, raw_frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read from webcam. Falling back to synthetic stream.")
            cap = None
            old_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            old_frame = ensure_bgr(raw_frame)
    else:
        old_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Synthetic frame fallback generator
    if cap is None and not is_picamera:
        h, w = 480, 640
        old_frame = np.zeros((h, w, 3), dtype=np.uint8)
        for x in range(0, w, 40):
            cv2.line(old_frame, (x, 0), (x, h), (120, 120, 120), 2)
        for y in range(0, h, 40):
            cv2.line(old_frame, (0, y), (w, y), (120, 120, 120), 2)
        cv2.circle(old_frame, (w//2, h//2), 80, (200, 200, 200), -1)
    
    frame_height, frame_width = old_frame.shape[:2]
    focal_length_x_px = focal_length_px(frame_width)
    focal_length_y_px = focal_length_x_px
    
    old_gray = to_small_gray(old_frame)
    
    # Initialize output sink (MP4 File or RTSP Server Stream)
    output_sink, rtsp_selected, rtsp_server = create_output_sink((frame_width, frame_height), target_fps)
    fallback_sink = None
    
    def switch_to_file_sink(frame_size, fps, reason):
        nonlocal fallback_sink, output_sink, rtsp_selected, rtsp_server
        if fallback_sink is None:
            print(f"[WARNING] {reason}")
            fallback_sink = FileSink(frame_size, fps)
        output_sink = fallback_sink
        rtsp_selected = False

    # Detect initial Shi-Tomasi features
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    
    visualizer = GradientFlowVisualizer(initial_mode)
    synthetic_angle = 0.0
    previous_frame_ts = time.perf_counter()
    last_mode_switch_time = time.time()
    frame_interval_s = 1.0 / target_fps
    
    print("[INFO] Press Ctrl+C to stop.")
    
    try:
        while True:
            now = time.perf_counter()
            # Frame rate pacing
            elapsed = now - previous_frame_ts
            if elapsed < frame_interval_s:
                time.sleep(frame_interval_s - elapsed)
            
            frame_ts = time.perf_counter()
            dt_s = frame_ts - previous_frame_ts
            previous_frame_ts = frame_ts
            
            # Handle Auto-cycling of visualization modes
            if cycle_interval > 0 and (time.time() - last_mode_switch_time) >= cycle_interval:
                visualizer.mode = (visualizer.mode + 1) % 4
                last_mode_switch_time = time.time()
                print(f"[INFO] Auto-cycling gradient display mode to: {visualizer.mode}")
            
            # 1. Grab new frame
            if is_picamera:
                frame = ensure_bgr(picam2.capture_array())
            elif cap is not None:
                ret, raw_frame = cap.read()
                if not ret:
                    continue
                frame = ensure_bgr(raw_frame)
            else:
                # Generate moving synthetic frame and simulate drone attitude changes
                synthetic_angle += 0.035
                dx = int(math.cos(synthetic_angle) * 10)
                dy = int(math.sin(synthetic_angle) * 8)
                
                with attitude_lock:
                    attitude_state["roll_deg"] = math.sin(synthetic_angle) * 12.0
                    attitude_state["pitch_deg"] = math.cos(synthetic_angle) * 10.0
                    attitude_state["yaw_deg"] = (synthetic_angle * 10) % 360.0
                    attitude_state["xgyro_dps"] = math.cos(synthetic_angle) * 35.0
                    attitude_state["ygyro_dps"] = -math.sin(synthetic_angle) * 35.0
                    attitude_state["zgyro_dps"] = 5.0
                with accel_lock:
                    accel_state["x_g"] = math.sin(synthetic_angle) * 0.25
                    accel_state["y_g"] = -math.cos(synthetic_angle) * 0.25
                    accel_state["roll_deg"] = math.sin(synthetic_angle) * 12.0
                    accel_state["pitch_deg"] = math.cos(synthetic_angle) * 10.0
                    accel_state["tilt_deg"] = 5.0
                with compass_lock:
                    compass_state["heading_deg"] = (synthetic_angle * 10) % 360.0
                    compass_state["timestamp"] = time.time()
                    
                frame = np.zeros((frame_height, frame_width, 3), dtype=np.uint8)
                for x in range(-80, frame_width + 80, 40):
                    cv2.line(frame, (x + dx, 0), (x + dx + dy, frame_height), (90, 90, 90), 1)
                for y in range(-80, frame_height + 80, 40):
                    cv2.line(frame, (0, y + dy), (frame_width, y + dy + dx), (90, 90, 90), 1)
                
                cx1 = int(frame_width//2 + math.cos(synthetic_angle)*60)
                cy1 = int(frame_height//2 + math.sin(synthetic_angle)*60)
                cv2.circle(frame, (cx1, cy1), 60, (200, 200, 200), -1)
                cv2.circle(frame, (cx1, cy1), 30, (140, 140, 140), -1)

            frame_gray = to_small_gray(frame)
            display_frame = frame.copy()
            draw_scale = 1.0 / FLOW_SCALE
            
            # 2. Compute spatial image gradients for overlays
            spatial_grad = visualizer.compute_sobel_gradient(frame_gray)
            spatial_grad_full = cv2.resize(spatial_grad, (frame_width, frame_height), interpolation=cv2.INTER_LINEAR)
            
            # 3. Calculate Lucas-Kanade Optical Flow
            tracked_count = 0
            if p0 is not None and len(p0) > 0:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
                if p1 is not None and st is not None:
                    good_new = p1[st.flatten() == 1]
                    good_old = p0[st.flatten() == 1]
                    tracked_count = len(good_new)
                    
                    # Filter out outlier motion vectors via RANSAC
                    inlier_old, inlier_new = reject_outlier_tracks(good_old, good_new)
                    
                    # Fetch drone altitude
                    with distance_lock:
                        altitude_cm = distance_state["current_distance"]
                    
                    if altitude_cm is None or altitude_cm <= 0:
                        altitude_cm = 150.0  # 1.5 meters simulated height
                    
                    # Calculate physical drone body velocities
                    velocity_estimate = estimate_body_velocity_mps(
                        inlier_old,
                        inlier_new,
                        altitude_cm,
                        dt_s,
                        focal_length_x_px,
                        focal_length_y_px,
                    )
                    
                    # Populate shared velocity state for HUD consumption
                    if velocity_estimate is not None:
                        vx_mps, vy_mps = velocity_estimate
                        velocity_state["vx_mps"] = vx_mps
                        velocity_state["vy_mps"] = vy_mps
                        velocity_state["speed_mps"] = float(math.hypot(vx_mps, vy_mps))
                        velocity_state["inliers"] = len(inlier_new)
                        velocity_state["last_update"] = frame_ts
                        
                        with position_lock:
                            position_state["x_m"] += vx_mps * dt_s
                            position_state["y_m"] += vy_mps * dt_s
                            position_state["path"].append((position_state["x_m"], position_state["y_m"]))
                            if len(position_state["path"]) > 200:
                                position_state["path"].pop(0)
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

                    # 4. Prepare Background Visualization based on current Mode
                    if visualizer.mode == 1:
                        # Mode 1: Sobel Gradient Magnitude (viridis)
                        grad_color = cv2.applyColorMap(spatial_grad_full, cv2.COLORMAP_VIRIDIS)
                        display_frame = grad_color
                    elif visualizer.mode == 2:
                        # Mode 2: Hybrid Spatial Glow (edges highlighted in magenta)
                        edges_bgr = cv2.merge([spatial_grad_full, np.zeros_like(spatial_grad_full), spatial_grad_full])
                        display_frame = cv2.addWeighted(frame, 0.70, edges_bgr, 0.65, 0)
                    elif visualizer.mode == 3:
                        # Mode 3: Dense Motion Gradient Map
                        dense_overlay = np.zeros_like(frame)
                        if len(inlier_new) > 2:
                            grid_sz = 16
                            grid_h, grid_w = frame_height // grid_sz, frame_width // grid_sz
                            grid_flow_x = np.zeros((grid_h, grid_w), dtype=np.float32)
                            grid_flow_y = np.zeros((grid_h, grid_w), dtype=np.float32)
                            
                            for new, old in zip(inlier_new, inlier_old):
                                nx, ny = new.ravel()
                                ox, oy = old.ravel()
                                gx = int(nx / (grid_sz * FLOW_SCALE))
                                gy = int(ny / (grid_sz * FLOW_SCALE))
                                if 0 <= gx < grid_w and 0 <= gy < grid_h:
                                    grid_flow_x[gy, gx] = nx - ox
                                    grid_flow_y[gy, gx] = ny - oy
                                    
                            grid_flow_x = cv2.GaussianBlur(grid_flow_x, (5, 5), 0)
                            grid_flow_y = cv2.GaussianBlur(grid_flow_y, (5, 5), 0)
                            
                            mag, ang = cv2.cartToPolar(grid_flow_x, grid_flow_y)
                            hsv = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)
                            hsv[..., 0] = ang * 90 / np.pi
                            hsv[..., 1] = 255
                            hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
                            
                            dense_overlay_small = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                            dense_overlay = cv2.resize(dense_overlay_small, (frame_width, frame_height), interpolation=cv2.INTER_CUBIC)
                            
                        display_frame = cv2.addWeighted(frame, 0.35, dense_overlay, 0.65, 0)

                    # 5. Draw inlier flow vectors onto the frame
                    for new, old in zip(inlier_new, inlier_old):
                        a, b = new.ravel()
                        c, d = old.ravel()
                        ax, by = int(a * draw_scale), int(b * draw_scale)
                        cx, dy = int(c * draw_scale), int(d * draw_scale)
                        
                        dx_px, dy_px = ax - cx, by - dy
                        magnitude = math.hypot(dx_px, dy_px)
                        angle = math.atan2(dy_px, dx_px)
                        
                        vector_color = visualizer.get_flow_color(angle, magnitude)
                        display_frame = cv2.arrowedLine(display_frame, (cx, dy), (ax, by), vector_color, 2, cv2.LINE_AA, tipLength=0.25)
                        display_frame = cv2.circle(display_frame, (ax, by), 4, (0, 0, 255), -1, cv2.LINE_AA)

                    if len(good_new) < TRACK_FEATURE_COUNT:
                        new_features = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                        if new_features is not None:
                            good_new = np.concatenate((good_new.reshape(-1, 2), new_features.reshape(-1, 2)), axis=0)
                    
                    if len(good_new) > TRACK_FEATURE_COUNT:
                        good_new = good_new[:TRACK_FEATURE_COUNT]
                    
                    p0 = good_new.reshape(-1, 1, 2)
                else:
                    p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
            else:
                p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)

            old_gray = frame_gray.copy()
            
            # 6. Apply Drone OSD and Ground Reticle overlay on top of selected visual mode
            display_frame = draw_ground_reticle(display_frame)
            display_frame = draw_osd(display_frame, tracked_count)
            
            # 7. Add Mode Toggle Notification Overlay
            display_frame = visualizer.draw_interactive_help(display_frame)
            
            # 8. Write processed frame to output sink (Stream or File)
            try:
                output_sink.write(display_frame)
            except RuntimeError as exc:
                if rtsp_selected:
                    switch_to_file_sink((frame_width, frame_height), target_fps, f"RTSP failed, falling back to local recording: {exc}")
                    output_sink.write(display_frame)
                else:
                    raise

            # FPS calculation
            visualizer.frame_count += 1
            curr_time = time.time()
            if curr_time - visualizer.last_fps_time >= 1.0:
                visualizer.fps = visualizer.frame_count / (curr_time - visualizer.last_fps_time)
                visualizer.frame_count = 0
                visualizer.last_fps_time = curr_time
                
    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt received. Stopping output...")
    finally:
        # Cleanup
        print("[INFO] Releasing resources and stopping subprocesses...")
        output_sink.release()
        if rtsp_server is not None:
            rtsp_server.release()
        if is_picamera:
            picam2.stop()
        if cap is not None:
            cap.release()
        print("[INFO] Engine stopped cleanly.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Lucas-Kanade Headless Gradient Engine")
    parser.add_argument("--mode", type=int, default=0, choices=[0, 1, 2, 3], help="Initial gradient visualization mode")
    parser.add_argument("--cycle", type=float, default=8.0, help="Interval in seconds to auto-cycle visualization modes (set to 0 to disable)")
    parser.add_argument("--fps", type=int, default=60, help="Target frame rate")
    args = parser.parse_args()
    
    run_pipeline(args.mode, args.cycle, args.fps)
