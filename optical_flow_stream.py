# MJPEG streaming server for optical flow results
# Requirements: flask, opencv-python, numpy, picamera2

from flask import Flask, Response
import cv2
import numpy as np
from picamera2 import Picamera2
import time

app = Flask(__name__)

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
try:
    camera_config['controls']['FrameDurationLimits'] = (1000, 1000)
except Exception:
    pass
picam2.configure(camera_config)
picam2.start()
time.sleep(2)

# Optical flow parameters
feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

old_frame = picam2.capture_array()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
mask = np.zeros_like(old_frame)

def gen_frames():
    global old_gray, p0, mask
    while True:
        frame = picam2.capture_array()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        img = frame
        if p1 is not None and p0 is not None and err is not None:
            # Filter: lost points (st==1) and high error points
            error_threshold = 20.0  # You can adjust this value
            good_points = (st.flatten() == 1) & (err.flatten() < error_threshold)
            good_new = p1[good_points]
            good_old = p0[good_points]
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                img = cv2.circle(img, (int(a), int(b)), 5, (0, 0, 255), -1)
            img = cv2.add(img, mask)
            # Always keep detecting new features if number drops below threshold
            min_features = 30
            if len(good_new) < min_features:
                # Detect new features and add them to the current set
                new_features = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                if new_features is not None:
                    # Combine old and new features with correct shape
                    good_new = np.concatenate((good_new.reshape(-1, 2), new_features.reshape(-1, 2)), axis=0)
                    good_new = good_new.reshape(-1, 1, 2)
            if len(good_new) > 0:
                old_gray = frame_gray.copy()
                p0 = good_new.reshape(-1, 1, 2)
            else:
                # If all points lost, re-detect features
                p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
                old_gray = frame_gray.copy()
                mask = np.zeros_like(frame)
        else:
            p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **feature_params)
            old_gray = frame_gray.copy()
            mask = np.zeros_like(frame)
        # Encode as JPEG
        ret, buffer = cv2.imencode('.jpg', img)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h1>Optical Flow MJPEG Stream</h1><img src='/video_feed'>"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
