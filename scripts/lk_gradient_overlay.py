#!/usr/bin/env python3
"""
Lucas-Kanade Optical Flow with Spatial and Motion Gradient Overlays.
Features:
- Dual-source input pipeline: Picamera2 (for Raspberry Pi drone) or standard cv2.VideoCapture (webcam).
- Image Gradient Overlay: Visualizes spatial intensity gradients (Sobel X/Y magnitude) using beautiful colormaps.
- Motion Gradient Overlay: Color-codes optical flow vectors using an HSV-based direction gradient map.
- Futuristic OSD/HUD: High-fidelity dashboard displaying real-time metrics, interactive controls, and a color wheel legend.
- Multi-mode display toggles via keyboard.
"""

import cv2
import numpy as np
import time
import math
import sys

# Parameters for Shi-Tomasi corner detection
feature_params = dict(
    maxCorners=100,
    qualityLevel=0.05,
    minDistance=10,
    blockSize=9
)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03)
)


class FlowVisualizer:
    def __init__(self):
        self.mode = 0  # 0: Camera + Flow, 1: Sobel Gradient + Flow, 2: Blended Glow + Flow, 3: Motion Vector Field
        self.show_legend = True
        self.max_corners = 100
        self.fps = 0.0
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Color palette (BGR)
        self.color_cyan = (220, 220, 0)
        self.color_purple = (240, 32, 160)
        self.color_green = (0, 255, 0)
        self.color_red = (0, 0, 255)
        self.color_yellow = (0, 255, 255)
        self.color_hud_bg = (15, 10, 10) # Dark glassmorphism base

    def get_flow_color(self, angle_rad, magnitude, max_mag=30.0):
        """Map flow angle and magnitude to a beautiful BGR color using HSV."""
        angle_deg = math.degrees(angle_rad) % 360
        hue = int(angle_deg / 2)  # OpenCV Hue is 0-179
        val = int(min(255, (magnitude / max_mag) * 200 + 55))
        hsv = np.uint8([[[hue, 255, val]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
        return tuple(int(c) for c in bgr)

    def draw_color_wheel(self, frame, center, radius=35):
        """Draw a beautiful flow direction color wheel legend."""
        overlay = frame.copy()
        
        # Draw circular gradient
        for y in range(-radius, radius):
            for x in range(-radius, radius):
                dist = math.hypot(x, y)
                if dist <= radius:
                    angle = math.atan2(y, x)
                    # Smooth outer boundary anti-aliasing
                    alpha = 1.0 if dist < radius - 2 else (radius - dist) / 2.0
                    color = self.get_flow_color(angle, dist, radius)
                    
                    px, py = center[0] + x, center[1] + y
                    if 0 <= px < frame.shape[1] and 0 <= py < frame.shape[0]:
                        overlay[py, px] = [
                            int(alpha * color[i] + (1 - alpha) * frame[py, px][i])
                            for i in range(3)
                        ]
                        
        # Draw outer ring and crosshairs
        cv2.circle(overlay, center, radius, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.line(overlay, (center[0] - radius, center[1]), (center[0] + radius, center[1]), (80, 80, 80), 1)
        cv2.line(overlay, (center[0], center[1] - radius), (center[0], center[1] + radius), (80, 80, 80), 1)
        
        # Directions labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(overlay, "R", (center[0] + radius + 4, center[1] + 4), font, 0.35, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(overlay, "D", (center[0] - 3, center[1] + radius + 11), font, 0.35, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(overlay, "L", (center[0] - radius - 12, center[1] + 4), font, 0.35, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(overlay, "U", (center[0] - 3, center[1] - radius - 4), font, 0.35, (200, 200, 200), 1, cv2.LINE_AA)
        
        return overlay

    def compute_sobel_gradient(self, gray):
        """Compute visual spatial image gradient magnitude."""
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
        magnitude = np.uint8(np.clip(magnitude, 0, 255))
        return magnitude

    def draw_hud(self, frame, tracked_count, avg_mag):
        """Render a high-fidelity glassmorphic telemetry dashboard on top of the frame."""
        h, w = frame.shape[:2]
        
        # Top Header HUD bar
        hud_h = 65
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, hud_h), self.color_hud_bg, -1)
        
        # Bottom Help/Control bar
        cv2.rectangle(overlay, (0, h - 30), (w, h), self.color_hud_bg, -1)
        
        # Apply transparency for glassmorphic effect
        frame = cv2.addWeighted(overlay, 0.70, frame, 0.30, 0)
        
        # Add thin glowing border to HUD elements
        cv2.line(frame, (0, hud_h), (w, hud_h), (80, 80, 100), 1, cv2.LINE_AA)
        cv2.line(frame, (0, h - 30), (w, h - 30), (80, 80, 100), 1, cv2.LINE_AA)
        
        # OSD Info
        font = cv2.FONT_HERSHEY_SIMPLEX
        # Title
        cv2.putText(frame, "LUCAS-KANADE GRADIENT ENGINE", (15, 24), font, 0.55, self.color_cyan, 2, cv2.LINE_AA)
        
        # System parameters
        mode_names = [
            "1. CAMERA + FLOW VECTORS",
            "2. SOBEL GRADIENT MAGNITUDE",
            "3. HYBRID SPATIAL GLOW",
            "4. DENSE MOTION GRADIENT"
        ]
        cv2.putText(frame, f"MODE: {mode_names[self.mode]}", (15, 46), font, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
        
        # Live Stats
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (w - 360, 24), font, 0.45, self.color_green, 1, cv2.LINE_AA)
        cv2.putText(frame, f"TRACKED FEATURES: {tracked_count}/{self.max_corners}", (w - 360, 46), font, 0.45, self.color_yellow, 1, cv2.LINE_AA)
        cv2.putText(frame, f"AVG FLOW: {avg_mag:.2f} px", (w - 170, 24), font, 0.45, self.color_cyan, 1, cv2.LINE_AA)
        
        # Bottom controls help
        help_text = "[M] Change Mode  |  [R] Re-detect Features  |  [C] Toggle Color Wheel  |  [+/-] Adj Corners  |  [ESC] Exit"
        cv2.putText(frame, help_text, (15, h - 10), font, 0.38, (180, 180, 180), 1, cv2.LINE_AA)
        
        # Overlay color wheel legend if enabled
        if self.show_legend:
            frame = self.draw_color_wheel(frame, (w - 60, h - 90), radius=32)
            
        return frame


def run_pipeline():
    print("[INFO] Initializing Lucas-Kanade & Gradient Overlay Engine...")
    
    # Try to initialize camera
    cap = None
    is_picamera = False
    
    # Try Picamera2 first
    try:
        from picamera2 import Picamera2
        print("[INFO] Attempting to connect to Picamera2...")
        picam2 = Picamera2()
        camera_config = picam2.create_preview_configuration()
        picam2.configure(camera_config)
        picam2.start()
        is_picamera = True
        print("[SUCCESS] Connected to Picamera2.")
    except Exception as e:
        print(f"[INFO] Picamera2 not available: {e}. Falling back to standard webcam.")
        
    if not is_picamera:
        # Try standard OpenCV capture (index 0)
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[WARNING] Primary camera (index 0) failed to open. Trying index 1...")
            cap = cv2.VideoCapture(1)
            
        if not cap.isOpened():
            print("[ERROR] No hardware camera detected. Generating synthetic test pattern stream.")
            cap = None

    # Wait for sensor warmup
    time.sleep(1.0)
    
    # Create window
    window_name = "Lucas-Kanade Gradient Overlay"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    # Capture first frame
    if is_picamera:
        old_frame = picam2.capture_array()
    elif cap is not None:
        ret, old_frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read from webcam. Falling back to synthetic stream.")
            cap = None
    
    # Synthetic frame fallback generator
    if cap is None and not is_picamera:
        # Generate dummy synthetic scrolling texture
        h, w = 480, 640
        old_frame = np.zeros((h, w, 3), dtype=np.uint8)
        # Draw some high-contrast grid lines & circles
        for x in range(0, w, 40):
            cv2.line(old_frame, (x, 0), (x, h), (120, 120, 120), 2)
        for y in range(0, h, 40):
            cv2.line(old_frame, (0, y), (w, y), (120, 120, 120), 2)
        cv2.circle(old_frame, (w//2, h//2), 80, (200, 200, 200), -1)
        cv2.circle(old_frame, (w//4, h//3), 40, (150, 150, 150), -1)
        cv2.circle(old_frame, (3*w//4, 2*h//3), 60, (250, 250, 250), -1)
    
    h, w = old_frame.shape[:2]
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    
    # Detect initial features
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
    
    visualizer = FlowVisualizer()
    synthetic_angle = 0.0
    
    print("[INFO] Entering main processing loop. Press 'ESC' or 'q' to quit.")
    
    while True:
        start_time = time.perf_counter()
        visualizer.frame_count += 1
        
        # 1. Grab new frame
        if is_picamera:
            frame = picam2.capture_array()
        elif cap is not None:
            ret, frame = cap.read()
            if not ret:
                print("[WARNING] Frame capture dropped.")
                continue
        else:
            # Generate moving synthetic frame
            synthetic_angle += 0.04
            dx = int(math.cos(synthetic_angle) * 8)
            dy = int(math.sin(synthetic_angle) * 6)
            
            frame = np.zeros((h, w, 3), dtype=np.uint8)
            # Draw moving patterns
            for x in range(-80, w + 80, 40):
                cv2.line(frame, (x + dx, 0), (x + dx + dy, h), (100, 100, 100), 2)
            for y in range(-80, h + 80, 40):
                cv2.line(frame, (0, y + dy), (w, y + dy + dx), (100, 100, 100), 2)
            
            # Circles moving in orbits
            cx1 = int(w//2 + math.cos(synthetic_angle)*50)
            cy1 = int(h//2 + math.sin(synthetic_angle)*50)
            cv2.circle(frame, (cx1, cy1), 70, (220, 220, 220), -1)
            
            cx2 = int(w//3 - math.sin(synthetic_angle * 1.5)*30)
            cy2 = int(h//3 + math.cos(synthetic_angle * 1.5)*30)
            cv2.circle(frame, (cx2, cy2), 35, (140, 140, 140), -1)
            
            cx3 = int(3*w//4 + math.cos(synthetic_angle * 0.8)*60)
            cy3 = int(2*h//3 + math.sin(synthetic_angle * 0.8)*20)
            cv2.circle(frame, (cx3, cy3), 55, (240, 240, 240), -1)
            
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display_frame = frame.copy()
        
        # 2. Compute spatial image gradients
        spatial_grad = visualizer.compute_sobel_gradient(frame_gray)
        
        # 3. Calculate Lucas-Kanade Optical Flow
        tracked_count = 0
        avg_magnitude = 0.0
        good_new = []
        good_old = []
        
        if p0 is not None and len(p0) > 0:
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            if p1 is not None and st is not None:
                good_new = p1[st.flatten() == 1]
                good_old = p0[st.flatten() == 1]
                tracked_count = len(good_new)
                
        # 4. Prepare Background Visualization based on current Mode
        if visualizer.mode == 1:
            # Mode 1: Sobel Gradient Magnitude (beautifully colormapped)
            grad_color = cv2.applyColorMap(spatial_grad, cv2.COLORMAP_VIRIDIS)
            display_frame = grad_color
        elif visualizer.mode == 2:
            # Mode 2: Hybrid Spatial Glow (blend Sobel edges as glowing cyan/purple overlay onto raw frame)
            edges_bgr = cv2.merge([spatial_grad, np.zeros_like(spatial_grad), spatial_grad]) # Purple glow
            display_frame = cv2.addWeighted(frame, 0.75, edges_bgr, 0.65, 0)
        elif visualizer.mode == 3:
            # Mode 3: Dense-like Flow Gradient Map
            # Build a smooth color-gradient overlay from sparse vectors using radial basis interpolation or simple dense approximation
            dense_overlay = np.zeros_like(frame)
            if len(good_new) > 2:
                # Generate a low-res motion grid and upscale for a smooth visual flow gradient
                grid_sz = 16
                grid_h, grid_w = h // grid_sz, w // grid_sz
                grid_flow_x = np.zeros((grid_h, grid_w), dtype=np.float32)
                grid_flow_y = np.zeros((grid_h, grid_w), dtype=np.float32)
                
                # Distribute sparse flow vectors into grid cells
                for new, old in zip(good_new, good_old):
                    nx, ny = new.ravel()
                    ox, oy = old.ravel()
                    gx = int(nx / grid_sz)
                    gy = int(ny / grid_sz)
                    if 0 <= gx < grid_w and 0 <= gy < grid_h:
                        grid_flow_x[gy, gx] = nx - ox
                        grid_flow_y[gy, gx] = ny - oy
                        
                # Smooth/Blur the grid flow
                grid_flow_x = cv2.GaussianBlur(grid_flow_x, (5, 5), 0)
                grid_flow_y = cv2.GaussianBlur(grid_flow_y, (5, 5), 0)
                
                # Convert to HSV color gradient map
                mag, ang = cv2.cartToPolar(grid_flow_x, grid_flow_y)
                hsv = np.zeros((grid_h, grid_w, 3), dtype=np.uint8)
                hsv[..., 0] = ang * 90 / np.pi  # Hue
                hsv[..., 1] = 255               # Saturation
                hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX) # Value
                
                dense_overlay_small = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
                dense_overlay = cv2.resize(dense_overlay_small, (w, h), interpolation=cv2.INTER_CUBIC)
                
            display_frame = cv2.addWeighted(frame, 0.40, dense_overlay, 0.60, 0)

        # 5. Draw flow vectors and compute telemetry
        if len(good_new) > 0:
            magnitudes = []
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                ax, by = new.ravel()
                cx, dy = old.ravel()
                
                # Flow vector details
                dx, dy_val = ax - cx, by - dy
                magnitude = math.hypot(dx, dy_val)
                magnitudes.append(magnitude)
                angle = math.atan2(dy_val, dx)
                
                # Color code vector based on angle (direction) and magnitude
                vector_color = visualizer.get_flow_color(angle, magnitude)
                
                # Draw elegant arrow and tracker circle
                cv2.arrowedLine(
                    display_frame,
                    (int(cx), int(dy)),
                    (int(ax), int(by)),
                    vector_color,
                    2,
                    cv2.LINE_AA,
                    tipLength=0.3
                )
                cv2.circle(display_frame, (int(ax), int(by)), 4, vector_color, -1, cv2.LINE_AA)
                
            avg_magnitude = float(np.mean(magnitudes))
            
            # Prepare next iteration features
            # Re-seed if feature count falls below threshold
            if len(good_new) < visualizer.max_corners * 0.4:
                new_features = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                if new_features is not None:
                    good_new = np.concatenate(
                        (good_new.reshape(-1, 2), new_features.reshape(-1, 2)),
                        axis=0
                    )
            
            # Constrain to max feature count
            if len(good_new) > visualizer.max_corners:
                good_new = good_new[:visualizer.max_corners]
                
            p0 = good_new.reshape(-1, 1, 2)
        else:
            # Re-detect completely if all features are lost
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)

        # Update historical frame reference
        old_gray = frame_gray.copy()
        
        # 6. Render Glassmorphic Telemetry HUD
        display_frame = visualizer.draw_hud(display_frame, tracked_count, avg_magnitude)
        
        # 7. Render output window
        cv2.imshow(window_name, display_frame)
        
        # FPS Calculation
        curr_time = time.time()
        if curr_time - visualizer.last_fps_time >= 1.0:
            visualizer.fps = visualizer.frame_count / (curr_time - visualizer.last_fps_time)
            visualizer.frame_count = 0
            visualizer.last_fps_time = curr_time
            
        # Keyboard interface
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):  # ESC or Q
            break
        elif key == ord('m'):  # Cycle visual mode
            visualizer.mode = (visualizer.mode + 1) % 4
        elif key == ord('r'):  # Force reset features
            print("[INFO] Re-seeding features...")
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
        elif key == ord('c'):  # Toggle color wheel legend
            visualizer.show_legend = not visualizer.show_legend
        elif key == ord('+') or key == ord('='):  # Increase tracked corners
            visualizer.max_corners = min(200, visualizer.max_corners + 10)
            feature_params['maxCorners'] = visualizer.max_corners
            print(f"[INFO] Max tracking corners set to {visualizer.max_corners}")
        elif key == ord('-') or key == ord('_'):  # Decrease tracked corners
            visualizer.max_corners = max(10, visualizer.max_corners - 10)
            feature_params['maxCorners'] = visualizer.max_corners
            print(f"[INFO] Max tracking corners set to {visualizer.max_corners}")
            
    # Cleanup
    print("[INFO] Stopping engine and cleaning resources...")
    if cap is not None:
        cap.release()
    if is_picamera:
        picam2.stop()
    cv2.destroyAllWindows()
    print("[INFO] Engine stopped.")


if __name__ == '__main__':
    run_pipeline()
