import cv2
import numpy as np
import time
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Standalone DIS Optical Flow Visualization"
    )
    parser.add_argument(
        "-input",
        "--input",
        type=str,
        default="picam",
        help="Video source: 'picam' for Raspberry Pi CSI camera, camera index (e.g. '0'), or path to video file.",
    )
    parser.add_argument(
        "-fps",
        "--fps",
        type=int,
        default=60,
        help="Target FPS for Pi Camera CSI.",
    )
    parser.add_argument(
        "-step",
        "--step",
        type=int,
        default=16,
        help="Grid sampling step in pixels for flow vector visualization.",
    )
    parser.add_argument(
        "-preset",
        "--preset",
        type=str,
        choices=["ultrafast", "fast", "medium"],
        default="ultrafast",
        help="DIS Optical Flow preset mode.",
    )
    parser.add_argument(
        "-scale",
        "--scale",
        type=float,
        default=0.5,
        help="Downscaling factor for processing frame (0.1 to 1.0).",
    )
    return parser.parse_args()


def get_dis_preset(preset_str):
    if preset_str == "fast":
        return cv2.DISOPTICAL_FLOW_PRESET_FAST
    elif preset_str == "medium":
        return cv2.DISOPTICAL_FLOW_PRESET_MEDIUM
    return cv2.DISOPTICAL_FLOW_PRESET_ULTRAFAST


def main():
    args = parse_args()

    use_picam = (args.input.lower() == "picam")
    picam2 = None
    cap = None

    if use_picam:
        try:
            from picamera2 import Picamera2
            picam2 = Picamera2()
            camera_config = picam2.create_preview_configuration()
            try:
                frame_duration_us = int(1_000_000 / args.fps)
                camera_config['controls']['FrameDurationLimits'] = (frame_duration_us, frame_duration_us)
            except Exception:
                pass
            picam2.configure(camera_config)
            picam2.start()
            time.sleep(2)
        except Exception as e:
            print(f"Error initializing Picamera2 CSI Camera: {e}")
            print("Falling back to OpenCV VideoCapture(0)...")
            use_picam = False

    if not use_picam:
        src = int(args.input) if args.input.isdigit() else args.input
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            print(f"Error: Could not open video source '{args.input}'.")
            return

    def capture_frame():
        if use_picam:
            frame = picam2.capture_array()
            # Ensure BGR format if 4 channels (BGRA) are returned by Picamera2
            if frame is not None and frame.ndim == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            return True, frame
        else:
            return cap.read()

    # Create DIS Optical Flow estimator
    preset = get_dis_preset(args.preset)
    dis_flow = cv2.DISOpticalFlow_create(preset)

    print("=======================================================")
    print(" Standalone DIS Optical Flow Viewer")
    print(f" Source: {'Raspberry Pi CSI Camera (Picamera2)' if use_picam else args.input}")
    print(f" Preset: {args.preset.upper()}")
    print(f" Step:   {args.step} px | Downscale: {args.scale}")
    print(" Press 'q' or ESC to quit.")
    print("=======================================================\n")

    ret, prev_frame = capture_frame()
    if not ret or prev_frame is None:
        print("Error: Failed to capture initial frame.")
        if cap:
            cap.release()
        if picam2:
            picam2.stop()
        return

    # Process initial frame
    h_orig, w_orig = prev_frame.shape[:2]
    proc_w = max(1, int(w_orig * args.scale))
    proc_h = max(1, int(h_orig * args.scale))

    prev_resized = cv2.resize(prev_frame, (proc_w, proc_h))
    prev_gray = cv2.cvtColor(prev_resized, cv2.COLOR_BGR2GRAY)

    fps_counter = 0
    fps = 0.0
    start_time = time.time()

    while True:
        ret, frame = capture_frame()
        if not ret or frame is None:
            print("End of video stream.")
            break

        frame_resized = cv2.resize(frame, (proc_w, proc_h))
        gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)

        # 1. Compute DIS Dense Optical Flow
        flow = dis_flow.calc(prev_gray, gray, None)

        # 2. Visualization - Dense HSV Color Map
        fx, fy = flow[..., 0], flow[..., 1]
        magnitude, angle = cv2.cartToPolar(fx, fy, angleInDegrees=True)

        hsv = np.zeros((proc_h, proc_w, 3), dtype=np.uint8)
        hsv[..., 0] = angle / 2
        hsv[..., 1] = 255
        hsv[..., 2] = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
        bgr_flow = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        # 3. Visualization - Vector Grid Overlay
        step = max(2, args.step)
        ys, xs = np.mgrid[step // 2 : proc_h : step, step // 2 : proc_w : step].astype(
            int
        )

        grid_u = flow[ys, xs, 0]
        grid_v = flow[ys, xs, 1]

        vis_frame = frame_resized.copy()
        for y, x, u, v in zip(
            ys.flatten(), xs.flatten(), grid_u.flatten(), grid_v.flatten()
        ):
            pt1 = (x, y)
            pt2 = (int(x + u), int(y + v))
            mag = np.hypot(u, v)
            if mag > 0.5:
                cv2.arrowedLine(
                    vis_frame,
                    pt1,
                    pt2,
                    (0, 255, 0),
                    1,
                    tipLength=0.3,
                    line_type=cv2.LINE_AA,
                )
                cv2.circle(vis_frame, pt1, 1, (0, 0, 255), -1)

        # 4. Calculate FPS
        fps_counter += 1
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps = fps_counter / elapsed
            fps_counter = 0
            start_time = time.time()

        # OSD Info Overlay
        cv2.putText(
            vis_frame,
            f"FPS: {fps:.1f} | Preset: {args.preset}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        # Side-by-side display: Vector Flow Overlay | HSV Dense Flow
        combined = np.hstack((vis_frame, bgr_flow))
        cv2.imshow("Standalone DIS Optical Flow (Left: Vector, Right: HSV)", combined)

        prev_gray = gray.copy()

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    if cap:
        cap.release()
    if picam2:
        picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
