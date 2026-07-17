# Lightweight optical flow recorder
# Requirements: opencv-python, numpy, picamera2, pymavlink, smbus2

import cv2
# Configure OpenCV threading to spread load across multiple CPU cores
try:
    initial_threads = cv2.getNumThreads()
    cpus = cv2.getNumberOfCPUs()
    cv2.setNumThreads(cpus)
    print(f"OpenCV Multi-threading: Configured threads from {initial_threads} to {cv2.getNumThreads()} (Available CPUs: {cpus})")
except Exception as e:
    print(f"Failed to configure OpenCV threads: {e}")

import numpy as np
import time
import math
import struct
import threading
import json
import os
import sys
import argparse

# Parse command line arguments
parser = argparse.ArgumentParser(description="Lightweight optical flow recorder and streamer.")
parser.add_argument("-stream", "--stream", action="store_true", help="Automatically run in RTSP stream mode.")
parser.add_argument("-record", "--record", action="store_true", help="Automatically run in local record mode.")
parser.add_argument("-duration", "--duration", "-d", type=float, default=None, help="Automatically run for this duration in seconds.")
args = parser.parse_args()

if args.stream and args.record:
    parser.error("Cannot specify both -stream and -record")

# Determine mode and RTSP name
mode = None
rtsp_name = None
if args.stream:
    mode = "rtsp"
    rtsp_name = "drone"
elif args.record:
    mode = "record"

import queue
from multiprocessing import shared_memory
from picamera2 import Picamera2

input_queue = queue.Queue()

def console_input_thread():
    """
    Reads commands from standard input in a loop and pushes them to input_queue.
    Runs in a background thread to avoid blocking the main execution loop.
    """
    while True:
        try:
            line = sys.stdin.readline().strip().lower()
            if line:
                input_queue.put(line)
        except Exception:
            break

def udp_command_listener():
    """
    Listens for UDP packets on port 5009 and processes commands like 'reset' or 'offset'.
    Pushes valid commands into the global input_queue.
    """
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 5009))
        while True:
            data, addr = sock.recvfrom(1024)
            cmd = data.decode("utf-8").strip().lower()
            if cmd == "reset":
                input_queue.put("r")
            elif cmd.startswith("offset "):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        ox = float(parts[1])
                        oy = float(parts[2])
                        input_queue.put(("offset", ox, oy))
                    except ValueError:
                        pass
    except Exception as e:
        print(f"UDP command listener error: {e}")

# Start the console input reader thread
# Moved to record_optical_flow() to prevent stdin conflict during startup prompts.

from optical_flow.sensor_readers import (
    start_distance_sensor_reader,
    distance_lock,
    distance_state,
    attitude_lock,
    attitude_state,
    is_gyro_calibrated
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

CALIBRATION_FILE = "tilt_calibration.json"
scale_x = FLOW_SCALE
scale_y = FLOW_SCALE
camera_offset_x = 0.0
camera_offset_y = 0.0

def load_calibration():
    """
    Loads tilt calibration and camera offsets from tilt_calibration.json.
    Falls back to default values if the file is not found or fails to parse.
    """
    global scale_x, scale_y, camera_offset_x, camera_offset_y
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE, "r") as f:
                data = json.load(f)
                scale_x = data.get("scale_x", FLOW_SCALE)
                scale_y = data.get("scale_y", FLOW_SCALE)
                camera_offset_x = data.get("camera_offset_x", 0.0)
                camera_offset_y = data.get("camera_offset_y", 0.0)
                print(f"Loaded calibration: scale_x={scale_x:.4f}, scale_y={scale_y:.4f}, camera_offset_x={camera_offset_x:.2f}, camera_offset_y={camera_offset_y:.2f}")
        except Exception as e:
            print(f"Failed to load calibration: {e}")
    else:
        print(f"No calibration file found, using defaults: scale_x={scale_x:.4f}, scale_y={scale_y:.4f}")

load_calibration()


TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS

# Shared memory configuration for optical flow stream
SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<11d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
SHM_SIZE = SHM_HEADER_SIZE + (MAX_SAMPLES * SHM_RECORD_SIZE)

flow_shm = None
flow_shm_lock = threading.Lock()


def attach_flow_stream_shm():
    """
    Attaches to or creates the shared memory block for optical flow stream data.
    Ensures that the shared memory size is correct and initializes the header.
    """
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

        try:
            from multiprocessing import resource_tracker
            resource_tracker.unregister(flow_shm._name, "shared_memory")
        except Exception:
            pass

        struct.pack_into(SHM_HEADER_FORMAT, flow_shm.buf, 0, SHM_MAGIC, 0, 0)
        return flow_shm


def write_flow_stream_sample(timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading):
    """
    Writes a single optical flow state sample into the circular buffer of the shared memory block.
    """
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
                float(x_cm),
                float(y_cm),
                float(x_raw_cm),
                float(y_raw_cm),
                float(vx),
                float(vy),
                float(vx_raw),
                float(vy_raw),
                float(alt),
                float(heading),
            )
            write_index = (write_index + 1) % MAX_SAMPLES
            sample_count = min(sample_count + 1, MAX_SAMPLES)
            struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)
    except Exception as exc:
        print(f"Failed to write to shared memory: {exc}")


def close_flow_stream_shm():
    """
    Closes and unlinks the shared memory resource when stopping the application.
    """
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

# Wait for gyro calibration to finish
print("Waiting for gyro calibration to complete...")
while not is_gyro_calibrated():
    time.sleep(0.1)

frame_height, frame_width = old_frame.shape[:2]
focal_length_x_px = focal_length_px(frame_width)
focal_length_y_px = focal_length_x_px

output_sink, rtsp_selected, rtsp_server = create_output_sink((frame_width, frame_height), TARGET_FPS, mode=mode, rtsp_name=rtsp_name)
fallback_sink = None
print("Press Ctrl+C to stop")


def switch_to_file_sink(frame_size, fps, reason):
    """
    Switches the output sink from RTSP stream to a local file recording.
    Triggered when the RTSP stream fails to initialize or write.
    """
    global fallback_sink, output_sink, rtsp_selected, rtsp_server
    if fallback_sink is None:
        print(reason)
        fallback_sink = FileSink(frame_size, fps)
    output_sink = fallback_sink
    rtsp_selected = False


def record_optical_flow():
    """
    Main optical flow recording and streaming loop.
    Captures video frames, estimates optical flow, applies tilt/offset compensation,
    updates state, and writes results to shared memory and video sink.
    """
    global old_gray, output_sink, scale_x, scale_y
    next_frame_time = time.perf_counter()
    previous_frame_ts = time.perf_counter()
    prev_roll_px = None
    prev_pitch_px = None
    x_raw_cm = 0.0
    y_raw_cm = 0.0
    vx_raw_prev = 0.0
    vy_raw_prev = 0.0
    
    is_calibrating = False
    calib_paused = False
    calib_samples_x = []
    calib_samples_y = []
    
    print("\n=======================================================")
    print("Type 'c' and press Enter to start tilt calibration.")
    print("Type 'r' and press Enter to reset positions to zero.")
    print("Press Ctrl+C to exit the program.")
    print("=======================================================\n")
    
    # Start the console input reader thread now that startup prompts are complete
    threading.Thread(target=console_input_thread, daemon=True).start()
    # Start the UDP command listener thread
    threading.Thread(target=udp_command_listener, daemon=True).start()
    
    start_time = time.perf_counter()
    
    while True:
        now = time.perf_counter()
        if args.duration is not None and (now - start_time) >= args.duration:
            print(f"\nDuration limit of {args.duration} seconds reached. Stopping...")
            break
            
        if now < next_frame_time:
            time.sleep(next_frame_time - now)
        frame_ts = time.perf_counter()
        dt_s = frame_ts - previous_frame_ts
        previous_frame_ts = frame_ts
        next_frame_time = frame_ts + FRAME_INTERVAL_S

        # Check for keyboard commands from the input queue non-blockingly
        command = None
        try:
            command = input_queue.get_nowait()
        except queue.Empty:
            pass

        if command is not None:
            if command == 'c' and not is_calibrating:
                is_calibrating = True
                calib_paused = False
                calib_samples_x = []
                calib_samples_y = []
                print("\n>>> TILT CALIBRATION STARTED. Please pitch and roll the camera/drone without translating it.")
                print(">>> Type 's' + Enter to save & exit, 'p' + Enter to pause/resume, 'e' + Enter to discard & exit.\n")
            elif is_calibrating:
                if command == 's':
                    is_calibrating = False
                    if calib_samples_x:
                        scale_x = float(np.median(calib_samples_x))
                    if calib_samples_y:
                        scale_y = float(np.median(calib_samples_y))
                    
                    try:
                        with open(CALIBRATION_FILE, "w") as f:
                            json.dump({"scale_x": scale_x, "scale_y": scale_y}, f, indent=4)
                        print(f"\n>>> TILT CALIBRATION COMPLETE & SAVED: scale_x={scale_x:.4f}, scale_y={scale_y:.4f} (from {len(calib_samples_x)}/{len(calib_samples_y)} samples)")
                    except Exception as e:
                        print(f"\n>>> Failed to save calibration to file: {e}")
                elif command == 'p':
                    calib_paused = not calib_paused
                    status = "PAUSED" if calib_paused else "RESUMED"
                    print(f"\n>>> TILT CALIBRATION {status}.")
                elif command == 'e':
                    is_calibrating = False
                    print("\n>>> TILT CALIBRATION DISCARDED.")
            
            if isinstance(command, tuple) and command[0] == "offset":
                global camera_offset_x, camera_offset_y
                _, ox, oy = command
                camera_offset_x = ox
                camera_offset_y = oy
                try:
                    calib_data = {}
                    if os.path.exists(CALIBRATION_FILE):
                        with open(CALIBRATION_FILE, "r") as f:
                            calib_data = json.load(f)
                    calib_data["camera_offset_x"] = camera_offset_x
                    calib_data["camera_offset_y"] = camera_offset_y
                    with open(CALIBRATION_FILE, "w") as f:
                        json.dump(calib_data, f, indent=4)
                    print(f"\n>>> CAMERA OFFSET UPDATED: x={camera_offset_x:.2f} cm, y={camera_offset_y:.2f} cm")
                except Exception as e:
                    print(f"Failed to save camera offset calibration: {e}")

            if command == 'r':
                x_raw_cm = 0.0
                y_raw_cm = 0.0
                vx_raw_prev = 0.0
                vy_raw_prev = 0.0
                with position_lock:
                    position_state["x_cm"] = 0.0
                    position_state["y_cm"] = 0.0
                    position_state["path"] = [(0.0, 0.0)]
                print("\n>>> POSITIONS RESET TO ZERO.")

        frame = ensure_bgr(picam2.capture_array())
        frame_gray = to_small_gray(frame)
        img = frame.copy()
        draw_scale = 1.0 / FLOW_SCALE

        dense_motion = estimate_dense_flow_and_motion(old_gray, frame_gray, step=8)

        # Calculate reticle displacement for tilt compensation
        with attitude_lock:
            roll_deg = attitude_state["roll_deg"]
            pitch_deg = attitude_state["pitch_deg"]
            yaw_deg = attitude_state["yaw_deg"]
            zgyro_dps = attitude_state.get("zgyro_dps", 0.0)

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

            # Record calibration samples if mode is active and not paused, and movement is sufficient
            if is_calibrating and not calib_paused:
                if abs(d_reticle_x) > 1.5:
                    calib_samples_x.append(tx / d_reticle_x)
                if abs(d_reticle_y) > 1.5:
                    calib_samples_y.append(ty / d_reticle_y)
                if len(calib_samples_x) % 20 == 0 or len(calib_samples_y) % 20 == 0:
                    print(f"\rCollected X: {len(calib_samples_x)}, Y: {len(calib_samples_y)} samples", end="", flush=True)

            # Apply reticle-based tilt compensation (subtracting expected displacement using calibrated scale)
            tx_comp = tx - (scale_x * d_reticle_x)
            ty_comp = ty - (scale_y * d_reticle_y)

            with distance_lock:
                altitude_cm = distance_state["current_distance"]

            # Calculate physical velocity using compensated translations (body frame)
            altitude_m = (altitude_cm / 100.0) if altitude_cm is not None else 1.5
            vx_mps_body = ((tx_comp * altitude_m) / (focal_length_x_px * dt_s))
            vy_mps_body = -((ty_comp * altitude_m) / (focal_length_y_px * dt_s))

            # Compensate for camera offset from center of rotation
            yaw_rate_rad = math.radians(zgyro_dps)
            v_offset_x = -yaw_rate_rad * (camera_offset_y / 100.0)
            v_offset_y = yaw_rate_rad * (camera_offset_x / 100.0)
            vx_mps_body_comp = vx_mps_body - v_offset_x
            vy_mps_body_comp = vy_mps_body - v_offset_y

            # Rotate compensated velocities to absolute frame (East/North) using actual compass heading
            yaw_actual_deg = -yaw_deg
            yaw_rad = math.radians(yaw_actual_deg)
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            vx_mps_calc = vx_mps_body_comp * cos_yaw + vy_mps_body_comp * sin_yaw
            vy_mps_calc = -vx_mps_body_comp * sin_yaw + vy_mps_body_comp * cos_yaw

            # Apply acceleration rate-limiter and velocity clamps to compensated velocity
            max_dv = 15.0 * dt_s
            prev_vx = velocity_state["vx_mps"]
            prev_vy = velocity_state["vy_mps"]
            vx_mps = np.clip(vx_mps_calc, prev_vx - max_dv, prev_vx + max_dv)
            vy_mps = np.clip(vy_mps_calc, prev_vy - max_dv, prev_vy + max_dv)
            vx_mps = np.clip(vx_mps, -5.0, 5.0)
            vy_mps = np.clip(vy_mps, -5.0, 5.0)

            # Calculate raw physical velocity (uncompensated, body frame)
            vx_raw_mps_body = ((tx * altitude_m) / (focal_length_x_px * dt_s))
            vy_raw_mps_body = -((ty * altitude_m) / (focal_length_y_px * dt_s))
            
            # Rotate raw velocities to absolute frame
            vx_raw_mps_calc = vx_raw_mps_body * cos_yaw + vy_raw_mps_body * sin_yaw
            vy_raw_mps_calc = -vx_raw_mps_body * sin_yaw + vy_raw_mps_body * cos_yaw


            # Apply same limits to raw velocity to prevent dashboard telemetry glitches
            vx_raw_mps = np.clip(vx_raw_mps_calc, vx_raw_prev - max_dv, vx_raw_prev + max_dv)
            vy_raw_mps = np.clip(vy_raw_mps_calc, vy_raw_prev - max_dv, vy_raw_prev + max_dv)
            vx_raw_mps = np.clip(vx_raw_mps, -5.0, 5.0)
            vy_raw_mps = np.clip(vy_raw_mps, -5.0, 5.0)
            
            vx_raw_prev = vx_raw_mps
            vy_raw_prev = vy_raw_mps

            velocity_state["vx_mps"] = vx_mps
            velocity_state["vy_mps"] = vy_mps
            velocity_state["speed_mps"] = float(math.hypot(vx_mps, vy_mps))
            velocity_state["inliers"] = tracked_count
            velocity_state["last_update"] = frame_ts

            with position_lock:
                position_state["x_cm"] += vx_mps * 100.0 * dt_s
                position_state["y_cm"] += vy_mps * 100.0 * dt_s
                position_state["path"].append((position_state["x_cm"], position_state["y_cm"]))
                if len(position_state["path"]) > 200:
                    position_state["path"].pop(0)

            x_raw_cm += vx_raw_mps * 100.0 * dt_s
            y_raw_cm += vy_raw_mps * 100.0 * dt_s

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
                position_state["path"].append((position_state["x_cm"], position_state["y_cm"]))
                if len(position_state["path"]) > 200:
                    position_state["path"].pop(0)

        # Stream current X, Y position, velocity, and altitude to shared memory
        with position_lock:
            current_x_cm = position_state["x_cm"]
            current_y_cm = position_state["y_cm"]
        current_vx = velocity_state["vx_mps"]
        current_vy = velocity_state["vy_mps"]
        with distance_lock:
            altitude_cm = distance_state["current_distance"]
        current_alt = (altitude_cm / 100.0) if altitude_cm is not None else 1.5
        write_flow_stream_sample(
            frame_ts,
            current_x_cm,
            current_y_cm,
            x_raw_cm,
            y_raw_cm,
            current_vx,
            current_vy,
            vx_raw_mps,
            vy_raw_mps,
            current_alt,
            (-yaw_deg) % 360.0
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

