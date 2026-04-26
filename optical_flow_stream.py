# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink

import cv2
import numpy as np
import threading
import subprocess
from picamera2 import Picamera2
import time
from datetime import datetime
from pathlib import Path
from pymavlink import mavutil

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS
RECORDINGS_DIR = Path("recordings")
PROJECT_ROOT = Path(__file__).resolve().parent
MEDIAMTX_BIN = PROJECT_ROOT / ".tools" / "mediamtx" / "mediamtx"
MAVLINK_CONNECTION_STRING = "udp:127.0.0.1:14551"

distance_state = {
    "current_distance": None,
    "last_update": 0.0,
}
distance_lock = threading.Lock()
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


def start_distance_sensor_reader():
    def reader():
        try:
            master = mavutil.mavlink_connection(MAVLINK_CONNECTION_STRING)
            master.wait_heartbeat()
            print(f"Connected to MAVLink on {MAVLINK_CONNECTION_STRING}")

            while True:
                msg = master.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)
                if msg is None:
                    continue

                with distance_lock:
                    distance_state["current_distance"] = msg.current_distance
                    distance_state["last_update"] = time.time()
        except Exception as e:
            print(f"MAVLink distance reader error: {e}")

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    return thread


def draw_osd(frame):
    with distance_lock:
        current_distance = distance_state["current_distance"]

    lines = [
        f"DIST: {current_distance if current_distance is not None else 'N/A'} cm",
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
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        y += 18

    return frame


def choose_output_mode():
    default_mode = "record"
    prompt = "Choose output mode: [r]ecord locally or [s]tream via RTSP? [r/s] "
    try:
        choice = input(prompt).strip().lower()
    except EOFError:
        choice = default_mode

    if choice in ("s", "stream", "rtsp"):
        return "rtsp"
    return default_mode


def choose_rtsp_url():
    default_url = "drone"
    try:
        value = input(f"RTSP stream name [{default_url}]: ").strip()
    except EOFError:
        value = ""
    return value or default_url


class FileSink:
    def __init__(self, frame_size, fps):
        RECORDINGS_DIR.mkdir(exist_ok=True)
        self.output_path = RECORDINGS_DIR / f"optical_flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, fps, frame_size)

        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {self.output_path}")

        print(f"Recording optical flow to {self.output_path}")

    def write(self, frame):
        self.writer.write(frame)

    def release(self):
        self.writer.release()
        print(f"Saved recording: {self.output_path}")


class RtspServer:
    def __init__(self):
        if not MEDIAMTX_BIN.exists():
            raise RuntimeError(f"MediaMTX binary not found at {MEDIAMTX_BIN}")

        self.process = subprocess.Popen(
            [str(MEDIAMTX_BIN)],
            cwd=str(MEDIAMTX_BIN.parent),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        time.sleep(1)
        if self.process.poll() is not None:
            raise RuntimeError("MediaMTX failed to start")

    def release(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except Exception:
                self.process.kill()


class RtspSink:
    def __init__(self, frame_size, fps, rtsp_url):
        frame_width, frame_height = frame_size
        self.rtsp_url = rtsp_url
        self.failed = False
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{frame_width}x{frame_height}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-pix_fmt",
                "yuv420p",
                "-f",
                "rtsp",
                "-rtsp_transport",
                "tcp",
                rtsp_url,
            ],
            stdin=subprocess.PIPE,
        )

        print(f"Streaming optical flow to {rtsp_url}")

    def write(self, frame):
        if self.failed:
            return
        if self.process.stdin is None:
            self.failed = True
            raise RuntimeError("RTSP stream is not writable")

        if self.process.poll() is not None:
            self.failed = True
            raise RuntimeError(f"RTSP server stopped for {self.rtsp_url}")

        try:
            self.process.stdin.write(frame.tobytes())
        except BrokenPipeError as exc:
            self.failed = True
            raise RuntimeError(f"RTSP stream dropped for {self.rtsp_url}") from exc
        except Exception:
            self.failed = True
            raise

    def is_failed(self):
        return self.failed

    def release(self):
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except Exception:
                pass

        try:
            self.process.wait(timeout=5)
        except Exception:
            self.process.kill()


def create_output_sink(frame_size, fps):
    mode = choose_output_mode()
    if mode == "rtsp":
        stream_name = choose_rtsp_url()
        server = RtspServer()
        publish_url = f"rtsp://127.0.0.1:8554/{stream_name}"
        print(f"Open this on Windows: rtsp://<drone-ip>:8554/{stream_name}")
        return RtspSink(frame_size, fps, publish_url), True, server
    return FileSink(frame_size, fps), False, None

old_frame = ensure_bgr(picam2.capture_array())
old_gray = to_small_gray(old_frame)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

start_distance_sensor_reader()

frame_height, frame_width = old_frame.shape[:2]
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
        img = draw_osd(img)
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
