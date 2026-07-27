import os
import threading
import serial
import pynmea2
from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gps_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

SERIAL_PORT = '/dev/ttyAMA2'
BAUD_RATE = 9600

# Global telemetry state
gps_data = {
    "lat": None,
    "lng": None,
    "alt": "N/A",
    "speed": "0.0 km/h",
    "sats": "0",
    "status": "SEARCHING FOR SATELLITES...",
    "fix": "No Fix"
}

def parse_coordinate(value, direction, is_lon=False):
    if not value or not direction:
        return None
    try:
        split_idx = 3 if is_lon else 2
        deg = float(value[:split_idx])
        minutes = float(value[split_idx:])
        dec = deg + (minutes / 60.0)
        if direction in ['S', 'W']:
            dec = -dec
        return round(dec, 6)
    except ValueError:
        return None

def gps_reader():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('$'):
                continue

            msg = pynmea2.parse(line)

            # GGA: Fix quality, satellites, altitude
            if isinstance(msg, pynmea2.types.talker.GGA):
                gps_data["sats"] = msg.num_sats if msg.num_sats else "0"
                if msg.gps_qual and int(msg.gps_qual) > 0:
                    gps_data["fix"] = "GPS Fix" if msg.gps_qual == '1' else "DGPS Fix"
                    gps_data["status"] = "FIX ACQUIRED"
                else:
                    gps_data["fix"] = "No Fix"
                    gps_data["status"] = "SEARCHING FOR SATELLITES..."

                if msg.altitude:
                    gps_data["alt"] = f"{msg.altitude} {msg.altitude_units}"

                # Parse coordinates from GGA if available
                if msg.lat and msg.lat_dir:
                    lat = parse_coordinate(msg.lat, msg.lat_dir)
                    if lat: gps_data["lat"] = lat
                if msg.lon and msg.lon_dir:
                    lng = parse_coordinate(msg.lon, msg.lon_dir, is_lon=True)
                    if lng: gps_data["lng"] = lng

            # RMC: Validated Coordinates & Speed
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.status == 'A':
                    lat = parse_coordinate(msg.lat, msg.lat_dir)
                    lng = parse_coordinate(msg.lon, msg.lon_dir, is_lon=True)
                    if lat and lng:
                        gps_data["lat"] = lat
                        gps_data["lng"] = lng
                    if msg.spd_over_grnd is not None:
                        gps_data["speed"] = f"{float(msg.spd_over_grnd) * 1.852:.1f} km/h"

            # Broadcast update over WebSocket
            socketio.emit('gps_update', gps_data)

        except (pynmea2.ParseError, Exception):
            pass

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    # Start GPS background thread
    t = threading.Thread(target=gps_reader, daemon=True)
    t.start()
    
    # Run web server on all network interfaces port 5000
    print("Server running at http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
