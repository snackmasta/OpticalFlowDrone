import cv2
import numpy as np
import math

TRACK_FEATURE_COUNT = 10
FLOW_SCALE = 0.5
CAMERA_HORIZONTAL_FOV_DEG = 62.2
MAX_FLOW_STEP_PX = 80.0
MIN_INLIERS_FOR_VELOCITY = 3

feature_params = dict(
    maxCorners=TRACK_FEATURE_COUNT,
    qualityLevel=0.3,
    minDistance=5,
    blockSize=5
)

lk_params = dict(
    winSize=(9, 9),
    maxLevel=0,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 8, 0.03),
)

velocity_state = {
    "vx_mps": 0.0,
    "vy_mps": 0.0,
    "speed_mps": 0.0,
    "inliers": 0,
    "last_update": 0.0,
}


def to_small_gray(frame):
    small = cv2.resize(frame, None, fx=FLOW_SCALE, fy=FLOW_SCALE, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)


def ensure_bgr(frame):
    if frame.ndim == 3 and frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def focal_length_px(frame_width):
    return frame_width / (2.0 * math.tan(math.radians(CAMERA_HORIZONTAL_FOV_DEG / 2.0)))


def reject_outlier_tracks(good_old, good_new):
    good_old = np.asarray(good_old, dtype=np.float32).reshape(-1, 2)
    good_new = np.asarray(good_new, dtype=np.float32).reshape(-1, 2)

    if len(good_old) != len(good_new):
        pair_count = min(len(good_old), len(good_new))
        good_old = good_old[:pair_count]
        good_new = good_new[:pair_count]

    if len(good_new) < MIN_INLIERS_FOR_VELOCITY:
        return good_old, good_new

    motion = (good_new - good_old).astype(np.float32)
    magnitudes = np.linalg.norm(motion, axis=1)
    basic_mask = magnitudes < MAX_FLOW_STEP_PX
    if np.count_nonzero(basic_mask) < MIN_INLIERS_FOR_VELOCITY:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    old_filtered = good_old[basic_mask]
    new_filtered = good_new[basic_mask]
    affine_result = cv2.estimateAffinePartial2D(old_filtered, new_filtered, method=cv2.RANSAC, ransacReprojThreshold=2.0)
    if affine_result is None:
        return old_filtered, new_filtered

    _, inlier_mask = affine_result
    if inlier_mask is None:
        return old_filtered, new_filtered

    inlier_mask = inlier_mask.ravel().astype(bool)
    if np.count_nonzero(inlier_mask) < MIN_INLIERS_FOR_VELOCITY:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    return old_filtered[inlier_mask], new_filtered[inlier_mask]


def estimate_body_velocity_mps(good_old, good_new, altitude_cm, dt_s, fx_px, fy_px):
    if altitude_cm is None or altitude_cm <= 0 or dt_s <= 0:
        return None
    if len(good_new) < MIN_INLIERS_FOR_VELOCITY:
        return None

    displacement = good_new - good_old
    median_dx_px = float(np.median(displacement[:, 0]))
    median_dy_px = float(np.median(displacement[:, 1]))

    altitude_m = altitude_cm / 100.0
    vx_mps = (median_dx_px * altitude_m) / (fx_px * dt_s)
    vy_mps = (median_dy_px * altitude_m) / (fy_px * dt_s)
    return vx_mps, vy_mps
