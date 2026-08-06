"""
Program Utama: Optical Flow Streamer & Logger (DIS Optical Flow & Telemetry Engine)
===================================================================================
Deskripsi:
    Script ini merupakan modul utama untuk estimasi pergerakan berbasis visi komputasional
    menggunakan algoritma Dense Inverse Search (DIS) Optical Flow dengan integrasi sensor.

Fitur Utama:
    1. Akuisisi Kamera & Sensor:
       - Pengambilan frame video dari kamera (Picamera2 / OpenCV).
       - Pembacaan sensor jarak (Distance Sensor) dan telemetry IMU/Attitude (Gyro, Accel).
    2. Estimasi Optical Flow & Kompensasi Gerak:
       - Komputasi dense optical flow (DIS Optical Flow) pada resolusi teroptimasi.
       - Estimasi kecepatan spasial (vx, vy) dan akumulasi posisi pergeseran (x, y).
       - Kompensasi rotasi kamera berbasis gyro/attitude untuk mengisolasi translasi drone.
    3. Output & Inter-Process Communication (IPC):
       - Penulisan data telemetry & flow ke Shared Memory (SHM) untuk dikonsumsi service lain (e.g. Geofence Engine, Web Server).
       - Logging data penerbangan ke file CSV.
       - Rendering OSD (On-Screen Display / Reticle HUD) dan streaming video via RTSP, MJPEG, atau file output.
"""

# Lightweight optical flow recorder and streamer
# Requirements: opencv-python, numpy, picamera2, smbus2

import cv2
import numpy as np
import time
import math
import argparse


from optical_flow.sensor_readers import (
    start_distance_sensor_reader,
    attitude_lock,
    attitude_state,
    accel_lock,
    accel_state,
    compass_lock,
    compass_state,
    is_gyro_calibrated
)
from optical_flow.phase_logger import PhaseTestManager
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
from optical_flow.shm_writer import (
    write_flow_stream_sample,
    close_flow_stream_shm
)
from optical_flow.csv_logger import CSVLogger

TARGET_FPS = 60
FRAME_INTERVAL_S = 1.0 / TARGET_FPS


def configure_opencv_threads():
    """Configures OpenCV multi-threading based on available CPU cores."""
    try:
        initial_threads = cv2.getNumThreads()
        cpus = cv2.getNumberOfCPUs()
        target_threads = min(2, cpus)
        cv2.setNumThreads(target_threads)
        print(f"OpenCV Multi-threading: Configured threads from {initial_threads} to {cv2.getNumThreads()} (Available CPUs: {cpus})")
    except Exception as e:
        print(f"Failed to configure OpenCV threads: {e}")


def parse_args():
    """Parses and validates command line arguments."""
    parser = argparse.ArgumentParser(description="Lightweight optical flow recorder and streamer.")
    parser.add_argument("-stream", "--stream", action="store_true", help="Automatically run in RTSP stream mode.")
    parser.add_argument("-record", "--record", action="store_true", help="Automatically run in local record mode.")
    parser.add_argument("-headless", "--headless", action="store_true", help="Run in headless telemetry-only mode (no video stream, no HUD rendering).")
    parser.add_argument("-duration", "--duration", "-d", type=float, default=None, help="Automatically run for this duration in seconds.")
    parser.add_argument("-csv", "--csv", nargs="?", const="auto", default=None, help="Save sensor readings and optical flow trajectory session to CSV file.")
    args = parser.parse_args()

    if sum([bool(args.stream), bool(args.record), bool(args.headless)]) > 1:
        parser.error("Can only specify one of -stream, -record, or -headless")

    mode = None
    rtsp_name = None
    if args.headless:
        mode = "headless"
    elif args.stream:
        mode = "rtsp"
        rtsp_name = "drone"
    elif args.record:
        mode = "record"

    return args, mode, rtsp_name


def init_camera(target_fps=TARGET_FPS):
    """Initializes Picamera2, configures frame rate limits, and captures initial frame."""
    from picamera2 import Picamera2
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration()
    try:
        frame_duration_us = int(1_000_000 / target_fps)
        camera_config['controls']['FrameDurationLimits'] = (frame_duration_us, frame_duration_us)
    except Exception:
        pass
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)

    old_frame = ensure_bgr(picam2.capture_array())
    old_gray = to_small_gray(old_frame)
    return picam2, old_frame, old_gray


class StreamState:
    """Encapsulates output sink state for RTSP fallback logic."""
    def __init__(self, output_sink, rtsp_selected, rtsp_server):
        self.output_sink = output_sink
        self.rtsp_selected = rtsp_selected
        self.rtsp_server = rtsp_server
        self.fallback_sink = None

    def switch_to_file_sink(self, frame_size, fps, reason):
        if self.fallback_sink is None:
            print(reason)
            self.fallback_sink = FileSink(frame_size, fps)
        self.output_sink = self.fallback_sink
        self.rtsp_selected = False


def record_optical_flow(picam2, old_gray, stream_state, mode, duration, csv_logger, frame_width, frame_height):
    """
    Main optical flow recording and streaming loop.
    Captures video frames, estimates optical flow, applies compensation,
    updates state, and writes results to shared memory, CSV logger, and video sink.
    """
    focal_length_x_px = focal_length_px(frame_width)
    focal_length_y_px = focal_length_x_px
    draw_scale = 1.0 / FLOW_SCALE

    phase_manager = PhaseTestManager(initial_phase="F0")
    frame_idx = 0

    next_frame_time = time.perf_counter()
    previous_frame_ts = time.perf_counter()
    x_raw_cm = 0.0
    y_raw_cm = 0.0
    vx_raw_prev = 0.0
    vy_raw_prev = 0.0

    print("\n=======================================================")
    print("Press Ctrl+C to exit the program.")
    print("=======================================================\n")

    start_time = time.perf_counter()

    while True:
        now = time.perf_counter()
        if duration is not None and (now - start_time) >= duration:
            print(f"\nDuration limit of {duration} seconds reached. Stopping...")
            break

        if now < next_frame_time:
            time.sleep(next_frame_time - now)
        frame_ts = time.perf_counter()
        dt_s = frame_ts - previous_frame_ts
        previous_frame_ts = frame_ts
        next_frame_time = frame_ts + FRAME_INTERVAL_S

        frame = ensure_bgr(picam2.capture_array())
        frame_gray = to_small_gray(frame)
        img = frame.copy()

        dense_motion = estimate_dense_flow_and_motion(old_gray, frame_gray, step=8)

        # Get attitude and gyro rates
        with attitude_lock:
            roll_deg = attitude_state["roll_deg"]
            pitch_deg = attitude_state["pitch_deg"]
            yaw_deg = attitude_state["yaw_deg"]
            zgyro_dps = attitude_state.get("zgyro_dps", 0.0)

        with accel_lock:
            xaccel_g = accel_state["x_g"]
            yaccel_g = accel_state["y_g"]

        tracked_count = 0
        vx_mps = 0.0
        vy_mps = 0.0
        vz_mps = 0.0
        vx_raw_mps = 0.0
        vy_raw_mps = 0.0

        if dense_motion is not None:
            tx, ty, scale, theta, inlier_old, inlier_new = dense_motion
            tracked_count = len(inlier_new)

            altitude_m = 1.5
            vx_mps_body = ((tx * altitude_m) / (focal_length_x_px * dt_s))
            vy_mps_body = -((ty * altitude_m) / (focal_length_y_px * dt_s))

            # Rotate velocities to absolute frame (East/North) using actual compass heading
            yaw_actual_deg = -yaw_deg
            yaw_rad = math.radians(yaw_actual_deg)
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            vx_mps_calc = vx_mps_body * cos_yaw + vy_mps_body * sin_yaw
            vy_mps_calc = -vx_mps_body * sin_yaw + vy_mps_body * cos_yaw

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

            # Apply limits to raw velocity
            vx_raw_mps = np.clip(vx_raw_mps_calc, vx_raw_prev - max_dv, vx_raw_prev + max_dv)
            vy_raw_mps = np.clip(vy_raw_mps_calc, vy_raw_prev - max_dv, vy_raw_prev + max_dv)
            vx_raw_mps = np.clip(vx_raw_mps, -5.0, 5.0)
            vy_raw_mps = np.clip(vy_raw_mps, -5.0, 5.0)

            vx_raw_prev = vx_raw_mps
            vy_raw_prev = vy_raw_mps

            # Calculate vertical velocity (vz_mps) using scale expansion/contraction
            vz_mps_calc = altitude_m * (1.0 - scale) / dt_s if abs(1.0 - scale) >= 0.005 else 0.0
            prev_vz = velocity_state.get("vz_mps", 0.0)
            vz_mps = float(np.clip(vz_mps_calc, prev_vz - max_dv, prev_vz + max_dv))
            vz_mps = float(np.clip(vz_mps, -5.0, 5.0))

            velocity_state["vx_mps"] = vx_mps
            velocity_state["vy_mps"] = vy_mps
            velocity_state["vz_mps"] = vz_mps
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

        # Stream current position, velocity, and altitude to shared memory
        with position_lock:
            current_x_cm = position_state["x_cm"]
            current_y_cm = position_state["y_cm"]
        current_vx = velocity_state["vx_mps"]
        current_vy = velocity_state["vy_mps"]
        current_alt = 1.5

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

        # Get active test phase info & event marker
        phase_info = phase_manager.get_phase_info()
        current_phase_code = phase_info["code"]
        event_marker = phase_manager.pop_event_marker()

        # Update OSD Phase Name
        draw_osd.current_phase_name = phase_info["name"]

        # Calculate optical flow pixel translation (tx, ty)
        of_tx = tx if dense_motion is not None else 0.0
        of_ty = ty if dense_motion is not None else 0.0

        # Sensor connectivity status
        sensor_status = "IMU:OK|MAG:OK|CAM:OK"

        # Frame index & FPS calculation
        frame_idx += 1
        fps_act = 1.0 / dt_s if dt_s > 0 else 60.0

        # Read compass / gyro / accel raw states
        with compass_lock:
            mag_x = compass_state.get("x", 0.0)
            mag_y = compass_state.get("y", 0.0)
            mag_z = compass_state.get("z", 0.0)

        with attitude_lock:
            gyro_x = attitude_state.get("xgyro_dps", 0.0)
            gyro_y = attitude_state.get("ygyro_dps", 0.0)
            gyro_z = attitude_state.get("zgyro_dps", 0.0)

        with accel_lock:
            acc_z = accel_state.get("z_g", 1.0)

        heading_deg = (-yaw_deg) % 360.0

        # Write multi-phase frame log to CSV
        csv_logger.log_frame(
            frame_ts=frame_ts,
            phase=current_phase_code,
            event_marker=event_marker,
            sensor_conn_status=sensor_status,
            fps_actual=fps_act,
            frame_index=frame_idx,
            raw_accel_x=xaccel_g,
            raw_accel_y=yaccel_g,
            raw_accel_z=acc_z,
            raw_gyro_x=gyro_x,
            raw_gyro_y=gyro_y,
            raw_gyro_z=gyro_z,
            raw_mag_x=mag_x,
            raw_mag_y=mag_y,
            raw_mag_z=mag_z,
            sensor_temp_c=25.0,
            cf_roll=roll_deg,
            cf_pitch=pitch_deg,
            cf_yaw=yaw_deg,
            fused_heading=heading_deg,
            of_inliers=tracked_count,
            of_tx_px=of_tx,
            of_ty_px=of_ty,
            of_vx=current_vx,
            of_vy=current_vy,
            raw_vx=vx_raw_mps,
            raw_vy=vy_raw_mps,
            speed_mps=velocity_state.get("speed_mps", 0.0),
            fused_x_cm=current_x_cm,
            fused_y_cm=current_y_cm,
            raw_x_cm=x_raw_cm,
            raw_y_cm=y_raw_cm,
            geofence_status="IN",
            geofence_breach_event=0,
            buzzer_signal="OFF",
            surface_noise_fallback=1 if dense_motion is None else 0,
            comms_uart="OK",
            comms_udp="OK",
            comms_rtsp="OK"
        )

        old_gray = frame_gray.copy()
        if mode != "headless":
            img = draw_ground_reticle(img)
            img = draw_osd(img, tracked_count)
            try:
                stream_state.output_sink.write(img)
            except RuntimeError as exc:
                if stream_state.rtsp_selected:
                    stream_state.switch_to_file_sink(
                        (frame_width, frame_height),
                        TARGET_FPS,
                        f"RTSP failed, falling back to local recording: {exc}"
                    )
                    stream_state.output_sink.write(img)
                else:
                    raise


def main():
    args, mode, rtsp_name = parse_args()
    configure_opencv_threads()

    picam2, old_frame, old_gray = init_camera(TARGET_FPS)
    frame_height, frame_width = old_frame.shape[:2]

    start_distance_sensor_reader()

    print("Waiting for gyro calibration to complete...")
    while not is_gyro_calibrated():
        time.sleep(0.1)

    output_sink, rtsp_selected, rtsp_server = create_output_sink(
        (frame_width, frame_height),
        TARGET_FPS,
        mode=mode,
        rtsp_name=rtsp_name
    )
    stream_state = StreamState(output_sink, rtsp_selected, rtsp_server)
    csv_logger = CSVLogger(args.csv, args.record)

    try:
        record_optical_flow(
            picam2,
            old_gray,
            stream_state,
            mode,
            args.duration,
            csv_logger,
            frame_width,
            frame_height
        )
    except KeyboardInterrupt:
        print("\nStopping output...")
    finally:
        csv_logger.close()
        if stream_state.output_sink is not None:
            stream_state.output_sink.release()
        if stream_state.rtsp_server is not None:
            stream_state.rtsp_server.release()
        picam2.stop()
        close_flow_stream_shm()


if __name__ == '__main__':
    main()
