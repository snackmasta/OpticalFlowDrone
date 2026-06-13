#!/usr/bin/env python3
"""
HMC5883L Compass Dashboard
Real-time magnetic heading visualization via web interface.
"""

import math
import threading
import time
from collections import deque

from flask import Flask, jsonify, render_template
from smbus2 import SMBus


HMC5883L_ADDR = 0x1E
CONFIG_A = 0x00
CONFIG_B = 0x01
MODE = 0x02
DATA_X_MSB = 0x03
DATA_Z_MSB = 0x05
DATA_Y_MSB = 0x07

DECLINATION_DEGREES = 0.0
MAX_SAMPLES = 120

compass_data = {
    "timestamps": deque(maxlen=MAX_SAMPLES),
    "x": deque(maxlen=MAX_SAMPLES),
    "y": deque(maxlen=MAX_SAMPLES),
    "z": deque(maxlen=MAX_SAMPLES),
    "heading": deque(maxlen=MAX_SAMPLES),
}

latest_state = {
    "connected": False,
    "heading": None,
    "x": None,
    "y": None,
    "z": None,
    "cardinal": "N",
    "error": None,
}

app = Flask(__name__)
data_lock = threading.Lock()


def read_word(bus, reg):
    high = bus.read_byte_data(HMC5883L_ADDR, reg)
    low = bus.read_byte_data(HMC5883L_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


def compute_heading(x, y):
    heading = math.degrees(math.atan2(y, x))
    heading += DECLINATION_DEGREES
    if heading < 0:
        heading += 360
    if heading >= 360:
        heading -= 360
    return heading


def cardinal_from_heading(heading):
    directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    index = int((heading + 22.5) // 45) % 8
    return directions[index]


def compass_thread_worker():
    bus = None
    while True:
        try:
            if bus is None:
                bus = SMBus(1)
                bus.write_byte_data(HMC5883L_ADDR, CONFIG_A, 0x70)
                bus.write_byte_data(HMC5883L_ADDR, CONFIG_B, 0x20)
                bus.write_byte_data(HMC5883L_ADDR, MODE, 0x00)
                time.sleep(0.2)

            x = read_word(bus, DATA_X_MSB)
            z = read_word(bus, DATA_Z_MSB)
            y = read_word(bus, DATA_Y_MSB)
            heading = compute_heading(x, y)
            cardinal = cardinal_from_heading(heading)

            timestamp = round(time.time(), 2)
            with data_lock:
                compass_data["timestamps"].append(timestamp)
                compass_data["x"].append(x)
                compass_data["y"].append(y)
                compass_data["z"].append(z)
                compass_data["heading"].append(round(heading, 2))

                latest_state.update({
                    "connected": True,
                    "heading": round(heading, 2),
                    "x": x,
                    "y": y,
                    "z": z,
                    "cardinal": cardinal,
                    "error": None,
                })

            time.sleep(0.1)
        except Exception as exc:
            with data_lock:
                latest_state["connected"] = False
                latest_state["error"] = str(exc)

            print(f"Compass read error: {exc}")
            time.sleep(1)
            if bus is not None:
                try:
                    bus.close()
                except Exception:
                    pass
                bus = None


@app.route("/")
def index():
    return render_template("compass_dashboard.html")


@app.route("/api/compass-data")
def get_compass_data():
    with data_lock:
        return jsonify({
            "timestamps": list(compass_data["timestamps"]),
            "x": list(compass_data["x"]),
            "y": list(compass_data["y"]),
            "z": list(compass_data["z"]),
            "heading": list(compass_data["heading"]),
            "latest": latest_state.copy(),
        })


def main():
    sensor_thread = threading.Thread(target=compass_thread_worker, daemon=True)
    sensor_thread.start()

    print("\n" + "=" * 60)
    print("HMC5883L Compass Dashboard")
    print("=" * 60)
    print("Starting Flask dashboard on http://0.0.0.0:5002")
    print("Open your browser to: http://localhost:5002")
    print("=" * 60 + "\n")

    app.run(host="0.0.0.0", port=5002, debug=False, threaded=True)


if __name__ == "__main__":
    main()