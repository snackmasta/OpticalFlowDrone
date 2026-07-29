#!/usr/bin/env python3
"""
GPS NMEA Telemetry Logging Script.
Reads incoming NMEA serial stream from GPS module, parses latitude, longitude,
altitude, velocity, heading, and fix status, and logs telemetry sessions to CSV.
"""

import os
import sys
import time
import math
import csv
import argparse
import random

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


DEFAULT_PORT = '/dev/ttyAMA2'
DEFAULT_BAUD = 9600
OUTPUT_DIR = "recordings"

FIX_QUALITY_MAP = {
    '0': 'No Fix',
    '1': 'GPS 3D Fix',
    '2': 'DGPS Fix',
    '4': 'RTK Fixed',
    '5': 'RTK Float'
}


def nmea_to_decimal(raw_val, direction):
    """Converts NMEA raw coordinate format (DDMM.MMMM / DDDMM.MMMM) to Decimal Degrees."""
    if not raw_val or '.' not in raw_val:
        return None
    try:
        dot_idx = raw_val.find('.')
        deg_len = dot_idx - 2
        if deg_len <= 0:
            return None
        degrees = float(raw_val[:deg_len])
        minutes = float(raw_val[deg_len:])
        decimal = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal
    except ValueError:
        return None


def format_utc_time(raw_time):
    """Formats raw NMEA UTC timestamp (HHMMSS.SS) into HH:MM:SS UTC format."""
    if not raw_time or len(raw_time) < 6:
        return "N/A"
    return f"{raw_time[:2]}:{raw_time[2:4]}:{raw_time[4:6]} UTC"


class GPSLogger:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, csv_path=None, duration=None, sim=False):
        self.port = port
        self.baud = baud
        self.duration = duration
        self.sim = sim
        self.current_state = {
            "utc_time": "N/A",
            "fix_code": "0",
            "fix_status": "No Fix",
            "satellites": 0,
            "lat": None,
            "lon": None,
            "alt_m": None,
            "speed_mps": 0.0,
            "speed_kmh": 0.0,
            "heading_deg": None,
            "hdop": None
        }

        # Resolve CSV output path
        if not csv_path or csv_path == "auto":
            os.makedirs(OUTPUT_DIR, exist_ok=True)
            timestamp_str = time.strftime("%Y%m%d_%H%M%S")
            self.csv_path = os.path.join(OUTPUT_DIR, f"gps_session_{timestamp_str}.csv")
        else:
            self.csv_path = csv_path
            csv_dir = os.path.dirname(os.path.abspath(self.csv_path))
            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)

    def parse_nmea_line(self, line):
        line = line.strip()
        if not line.startswith('$'):
            return False

        parts = line.split(',')
        sentence_type = parts[0]

        updated = False

        # Parse $GNGGA / $GPGGA (Fix data, altitude, sat count, HDOP)
        if sentence_type in ['$GNGGA', '$GPGGA'] and len(parts) >= 10:
            if parts[1]:
                self.current_state["utc_time"] = format_utc_time(parts[1])
            raw_lat, lat_dir = parts[2], parts[3]
            raw_lon, lon_dir = parts[4], parts[5]
            fix_code = parts[6] if len(parts) > 6 and parts[6] else '0'
            num_sats = int(parts[7]) if len(parts) > 7 and parts[7].isdigit() else 0
            hdop = float(parts[8]) if len(parts) > 8 and parts[8] else None
            alt = float(parts[9]) if len(parts) > 9 and parts[9] else None

            self.current_state["fix_code"] = fix_code
            self.current_state["fix_status"] = FIX_QUALITY_MAP.get(fix_code, 'Unknown')
            self.current_state["satellites"] = num_sats
            self.current_state["hdop"] = hdop
            if alt is not None:
                self.current_state["alt_m"] = alt

            lat = nmea_to_decimal(raw_lat, lat_dir)
            lon = nmea_to_decimal(raw_lon, lon_dir)
            if lat is not None and lon is not None:
                self.current_state["lat"] = lat
                self.current_state["lon"] = lon
                updated = True

        # Parse $GNRMC / $GPRMC (Recommended Minimum data, speed, heading)
        elif sentence_type in ['$GNRMC', '$GPRMC'] and len(parts) >= 9:
            if parts[1]:
                self.current_state["utc_time"] = format_utc_time(parts[1])
            status = parts[2]  # 'A' = Valid, 'V' = Void
            raw_lat, lat_dir = parts[3], parts[4]
            raw_lon, lon_dir = parts[5], parts[6]
            speed_knots = float(parts[7]) if len(parts) > 7 and parts[7] else 0.0
            course_deg = float(parts[8]) if len(parts) > 8 and parts[8] else None

            speed_mps = speed_knots * 0.514444
            speed_kmh = speed_knots * 1.852
            self.current_state["speed_mps"] = speed_mps
            self.current_state["speed_kmh"] = speed_kmh
            if course_deg is not None:
                self.current_state["heading_deg"] = course_deg

            if status == 'A':
                lat = nmea_to_decimal(raw_lat, lat_dir)
                lon = nmea_to_decimal(raw_lon, lon_dir)
                if lat is not None and lon is not None:
                    self.current_state["lat"] = lat
                    self.current_state["lon"] = lon
                    updated = True

        return updated

    def run_simulation(self, csv_writer, csv_file):
        """Simulates GPS NMEA stream for testing on PC without serial hardware."""
        print(">>> Running in GPS Simulation Mode...")
        start_time = time.time()
        base_lat = -6.2088
        base_lon = 106.8456
        sample_counter = 0

        while True:
            now = time.time()
            elapsed = now - start_time
            if self.duration is not None and elapsed >= self.duration:
                break

            # Simulate circular flight path
            r_deg = 0.0005
            theta = elapsed * 0.2
            lat = base_lat + r_deg * math.cos(theta) + random.gauss(0, 0.00001)
            lon = base_lon + r_deg * math.sin(theta) + random.gauss(0, 0.00001)
            alt = 15.0 + math.sin(elapsed * 0.1) * 2.0
            speed_mps = 2.5 + random.gauss(0, 0.2)
            heading = (theta * 180.0 / math.pi) % 360.0

            utc_str = time.strftime("%H:%M:%S UTC", time.gmtime(now))

            csv_writer.writerow([
                f"{now:.4f}",
                utc_str,
                "1",
                "GPS 3D Fix (Simulated)",
                8,
                f"{lat:.7f}",
                f"{lon:.7f}",
                f"{alt:.2f}",
                f"{speed_mps:.2f}",
                f"{speed_mps * 3.6:.2f}",
                f"{heading:.1f}",
                1.2
            ])
            csv_file.flush()
            sample_counter += 1

            if sample_counter % 10 == 0:
                print(f"[{utc_str}] Logged sample #{sample_counter}: Lat={lat:.6f}°, Lon={lon:.6f}°, Alt={alt:.1f}m, Speed={speed_mps:.1f}m/s")

            time.sleep(0.2)  # 5Hz simulation

    def start_logging(self):
        print("=" * 65)
        print("              GPS NMEA TELEMETRY LOGGER              ")
        print("=" * 65)
        print(f"Target CSV File : {self.csv_path}")

        try:
            csv_file = open(self.csv_path, "w", newline="")
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "Timestamp (s)",
                "UTC Time",
                "Fix Code",
                "Fix Status",
                "Satellites",
                "Latitude (deg)",
                "Longitude (deg)",
                "Altitude (m)",
                "Speed (m/s)",
                "Speed (km/h)",
                "Heading (deg)",
                "HDOP"
            ])
            csv_file.flush()
        except Exception as e:
            print(f"Failed to open CSV log file: {e}")
            return

        if self.sim:
            try:
                self.run_simulation(csv_writer, csv_file)
            except KeyboardInterrupt:
                print("\nGPS Simulation stopped manually.")
            finally:
                csv_file.close()
                print(f"GPS session log saved to: {self.csv_path}")
            return

        if not SERIAL_AVAILABLE:
            print("Error: 'pyserial' package is not installed.")
            print("To run in simulation mode on PC, use the --sim flag: python log_gps.py --sim")
            csv_file.close()
            return

        print(f"Connecting to GPS module on {self.port} at {self.baud} baud...")
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            print("Serial connection established. Logging GPS stream (Press Ctrl+C to stop)...\n")
        except Exception as e:
            print(f"Failed to open serial port {self.port}: {e}")
            print("To test on PC without serial hardware, run: python log_gps.py --sim")
            csv_file.close()
            return

        sample_counter = 0
        start_time = time.time()

        try:
            while True:
                now = time.time()
                elapsed = now - start_time
                if self.duration is not None and elapsed >= self.duration:
                    print(f"\nDuration limit of {self.duration}s reached. Stopping logging...")
                    break

                raw_line = ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode('ascii', errors='replace').strip()
                updated = self.parse_nmea_line(line)

                if updated and self.current_state["lat"] is not None:
                    st = self.current_state
                    csv_writer.writerow([
                        f"{now:.4f}",
                        st["utc_time"],
                        st["fix_code"],
                        st["fix_status"],
                        st["satellites"],
                        f"{st['lat']:.7f}",
                        f"{st['lon']:.7f}",
                        f"{st['alt_m']:.2f}" if st['alt_m'] is not None else "N/A",
                        f"{st['speed_mps']:.2f}",
                        f"{st['speed_kmh']:.2f}",
                        f"{st['heading_deg']:.1f}" if st['heading_deg'] is not None else "N/A",
                        f"{st['hdop']:.2f}" if st['hdop'] is not None else "N/A"
                    ])
                    csv_file.flush()
                    sample_counter += 1

                    if sample_counter % 10 == 0:
                        print(f"[{st['utc_time']}] Logged sample #{sample_counter}: {st['lat']:.6f}°, {st['lon']:.6f}° | Sats: {st['satellites']} | Alt: {st['alt_m']}m")

        except KeyboardInterrupt:
            print("\nGPS logging stopped manually.")
        finally:
            ser.close()
            csv_file.close()
            print(f"\nCompleted! Saved total {sample_counter} samples to: {self.csv_path}")


def main():
    parser = argparse.ArgumentParser(description="GPS NMEA stream logger to CSV.")
    parser.add_argument("-port", "--port", "-p", default=DEFAULT_PORT, help="Serial port (default: /dev/ttyAMA2).")
    parser.add_argument("-baud", "--baud", "-b", type=int, default=DEFAULT_BAUD, help="Serial baud rate (default: 9600).")
    parser.add_argument("-csv", "--csv", "-c", default="auto", help="Output CSV file path.")
    parser.add_argument("-duration", "--duration", "-d", type=float, default=None, help="Duration limit in seconds.")
    parser.add_argument("-sim", "--sim", action="store_true", help="Run in GPS simulation mode for PC testing.")
    args = parser.parse_args()

    logger = GPSLogger(port=args.port, baud=args.baud, csv_path=args.csv, duration=args.duration, sim=args.sim)
    logger.start_logging()


if __name__ == "__main__":
    main()
