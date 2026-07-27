import os
import time
import serial
import pynmea2

SERIAL_PORT = '/dev/ttyAMA2'
BAUD_RATE = 9600

def convert_latitude(lat, lat_dir):
    if not lat or not lat_dir:
        return "N/A"
    try:
        # Convert NMEA format (DDMM.MMMM) to Decimal Degrees
        deg = float(lat[:2])
        minutes = float(lat[2:])
        dec_deg = deg + (minutes / 60.0)
        if lat_dir == 'S':
            dec_deg = -dec_deg
        return f"{dec_deg:.6f}° ({lat_dir})"
    except ValueError:
        return "N/A"

def convert_longitude(lon, lon_dir):
    if not lon or not lon_dir:
        return "N/A"
    try:
        # Convert NMEA format (DDDMM.MMMM) to Decimal Degrees
        deg = float(lon[:3])
        minutes = float(lon[3:])
        dec_deg = deg + (minutes / 60.0)
        if lon_dir == 'W':
            dec_deg = -dec_deg
        return f"{dec_deg:.6f}° ({lon_dir})"
    except ValueError:
        return "N/A"

def monitor_gps():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return

    # Data state
    status = "SEARCHING FOR SATELLITES..."
    satellites = "0"
    latitude = "N/A"
    longitude = "N/A"
    altitude = "N/A"
    speed = "N/A"
    fix_type = "No Fix"

    print("Connecting to GPS module...")

    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('$'):
                continue

            msg = pynmea2.parse(line)

            # GGA Sentence: Fix quality, Satellites, Altitude
            if isinstance(msg, pynmea2.types.talker.GGA):
                satellites = msg.num_sats if msg.num_sats else "0"
                if msg.gps_qual and int(msg.gps_qual) > 0:
                    fix_type = "GPS Fix" if msg.gps_qual == '1' else "DGPS Fix"
                    status = "3D FIX ACQUIRED"
                else:
                    fix_type = "No Fix"
                    status = "SEARCHING FOR SATELLITES..."

                if msg.altitude:
                    altitude = f"{msg.altitude} {msg.altitude_units}"

            # RMC Sentence: Coordinates, Speed
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.status == 'A':  # 'A' means Active/Valid
                    latitude = convert_latitude(msg.lat, msg.lat_dir)
                    longitude = convert_longitude(msg.lon, msg.lon_dir)
                    if msg.spd_over_grnd is not None:
                        # Convert knots to km/h
                        kmh = float(msg.spd_over_grnd) * 1.852
                        speed = f"{kmh:.1f} km/h"

            # Refresh terminal UI
            os.system('clear')
            print("==================================================")
            print("         GY-GPS6MV2 (u-blox) MONITOR              ")
            print("==================================================")
            print(f" Port:        {SERIAL_PORT} @ {BAUD_RATE} baud")
            print(f" Status:      {status}")
            print(f" Fix Type:    {fix_type}")
            print(f" Satellites:  {satellites} in view/use")
            print("--------------------------------------------------")
            print(f" Latitude:    {latitude}")
            print(f" Longitude:   {longitude}")
            print(f" Altitude:    {altitude}")
            print(f" Speed:       {speed}")
            print("==================================================")
            print(" Press Ctrl+C to exit.")

        except pynmea2.ParseError:
            pass  # Ignore incomplete sentences
        except KeyboardInterrupt:
            print("\nExiting monitor.")
            ser.close()
            break

if __name__ == "__main__":
    monitor_gps()
