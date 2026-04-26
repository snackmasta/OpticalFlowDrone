# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2

import cv2
import numpy as np
from picamera2 import Picamera2
import time
from datetime import datetime
from pathlib import Path

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS
RECORDINGS_DIR = Path("recordings")

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

# Optical flow parameters
TRACK_FEATURE_COUNT = 5
FLOW_SCALE = 0.5
feature_params = dict(maxCorners=TRACK_FEATURE_COUNT, qualityLevel=0.3, minDistance=5, blockSize=5)
lk_params = dict(
    winSize=(9, 9),
    maxLevel=0,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 8, 0.03),
)


def to_small_gray(frame):
    small = cv2.resize(frame, None, fx=FLOW_SCALE, fy=FLOW_SCALE, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)


def ensure_bgr(frame):
    if frame.ndim == 3 and frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame

old_frame = ensure_bgr(picam2.capture_array())
old_gray = to_small_gray(old_frame)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

RECORDINGS_DIR.mkdir(exist_ok=True)
output_path = RECORDINGS_DIR / f"optical_flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"

frame_height, frame_width = old_frame.shape[:2]
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
writer = cv2.VideoWriter(str(output_path), fourcc, TARGET_FPS, (frame_width, frame_height))

if not writer.isOpened():
    picam2.stop()
    raise RuntimeError(f"Failed to open video writer for {output_path}")

print(f"Recording optical flow to {output_path}")
print("Press Ctrl+C to stop recording")

def record_optical_flow():
    global old_gray, p0
    next_frame_time = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now < next_frame_time:
            time.sleep(next_frame_time - now)
        next_frame_time = time.perf_counter() + FRAME_INTERVAL_S

        frame = ensure_bgr(picam2.capture_array())
        frame_gray = to_small_gray(frame)
        img = frame.copy()
        draw_scale = 1.0 / FLOW_SCALE

        if p0 is not None and len(p0) > 0:
            p1, st, _ = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            if p1 is not None and st is not None:
                good_new = p1[st.flatten() == 1]
                good_old = p0[st.flatten() == 1]

                for new, old in zip(good_new, good_old):
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
        writer.write(img)

if __name__ == '__main__':
    try:
        record_optical_flow()
    except KeyboardInterrupt:
        print("\nStopping recording...")
    finally:
        writer.release()
        picam2.stop()
        print(f"Saved recording: {output_path}")
