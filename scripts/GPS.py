import serial
import time

def nmea_to_decimal(raw_val, direction):
    """Converts NMEA raw coordinate format (DDMM.MMMM / DDDMM.MMMM) to Decimal Degrees."""
    if not raw_val or '.' not in raw_val:
        return None
    try:
        dot_idx = raw_val.find('.')
        deg_len = dot_idx - 2  # 2 digits for Latitude (DD), 3 digits for Longitude (DDD)
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
    hh = raw_time[:2]
    mm = raw_time[2:4]
    ss = raw_time[4:6]
    return f"{hh}:{mm}:{ss} UTC"

# Serial port configuration for Pi / Controller
SERIAL_PORT = '/dev/ttyAMA2'
BAUD_RATE = 9600

FIX_QUALITY_MAP = {
    '0': 'No Fix (Searching for satellites...)',
    '1': 'GPS Fix (Standard 3D Fix)',
    '2': 'DGPS Fix (Differential GPS)',
    '4': 'RTK Fixed',
    '5': 'RTK Float'
}

def parse_and_display(line):
    line = line.strip()
    if not line.startswith('$'):
        return

    parts = line.split(',')
    sentence_type = parts[0]

    # Process GGA Sentence (Global Positioning System Fix Data)
    if sentence_type in ['$GNGGA', '$GPGGA'] and len(parts) >= 10:
        utc_time = format_utc_time(parts[1])
        raw_lat, lat_dir = parts[2], parts[3]
        raw_lon, lon_dir = parts[4], parts[5]
        fix_code = parts[6] if len(parts) > 6 else '0'
        num_sats = parts[7] if len(parts) > 7 and parts[7] else '0'
        alt = parts[9] if len(parts) > 9 and parts[9] else 'N/A'
        
        fix_status = FIX_QUALITY_MAP.get(fix_code, 'Unknown Fix Status')
        lat = nmea_to_decimal(raw_lat, lat_dir)
        lon = nmea_to_decimal(raw_lon, lon_dir)

        print("=" * 60)
        print(f" [GPS TELEMETRY READOUT] Message: {sentence_type} | Time: {utc_time}")
        print("-" * 60)
        print(f"  Fix Status  : {fix_status}")
        print(f"  Satellites  : {num_sats} connected")

        if lat is not None and lon is not None:
            lat_cardinal = f"{abs(lat):.6f}° {'N' if lat >= 0 else 'S'}"
            lon_cardinal = f"{abs(lon):.6f}° {'E' if lon >= 0 else 'W'}"
            print(f"  Latitude    : {lat_cardinal} ({lat:.6f})")
            print(f"  Longitude   : {lon_cardinal} ({lon:.6f})")
            if alt != 'N/A':
                print(f"  Altitude    : {alt} meters MSL")
            print(f"  Google Maps : https://maps.google.com/?q={lat:.6f},{lon:.6f}")
        else:
            print("  Coordinates : Waiting for valid satellite lock...")
        print("=" * 60 + "\n")

    # Process RMC Sentence (Recommended Minimum Specific GPS Data)
    elif sentence_type in ['$GNRMC', '$GPRMC'] and len(parts) >= 9:
        utc_time = format_utc_time(parts[1])
        status = parts[2] # 'A' = Valid, 'V' = Void
        raw_lat, lat_dir = parts[3], parts[4]
        raw_lon, lon_dir = parts[5], parts[6]
        speed_knots = parts[7] if len(parts) > 7 and parts[7] else '0.0'
        course_deg = parts[8] if len(parts) > 8 and parts[8] else 'N/A'

        lat = nmea_to_decimal(raw_lat, lat_dir)
        lon = nmea_to_decimal(raw_lon, lon_dir)

        status_text = "Valid Fix (Active)" if status == 'A' else "Void (No Satellite Fix)"

        try:
            speed_kmh = float(speed_knots) * 1.852
            speed_text = f"{float(speed_knots):.1f} knots ({speed_kmh:.1f} km/h)"
        except ValueError:
            speed_text = "N/A"

        print("=" * 60)
        print(f" [GPS TELEMETRY READOUT] Message: {sentence_type} | Time: {utc_time}")
        print("-" * 60)
        print(f"  Data Status : {status_text}")
        print(f"  Speed       : {speed_text}")
        if course_deg != 'N/A':
            print(f"  Heading     : {course_deg}° True North")

        if lat is not None and lon is not None:
            lat_cardinal = f"{abs(lat):.6f}° {'N' if lat >= 0 else 'S'}"
            lon_cardinal = f"{abs(lon):.6f}° {'E' if lon >= 0 else 'W'}"
            print(f"  Latitude    : {lat_cardinal} ({lat:.6f})")
            print(f"  Longitude   : {lon_cardinal} ({lon:.6f})")
            print(f"  Google Maps : https://maps.google.com/?q={lat:.6f},{lon:.6f}")
        else:
            print("  Coordinates : Waiting for valid satellite lock...")
        print("=" * 60 + "\n")


def main():
    print(f"Connecting to GPS Module on {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("GPS Serial Connection Established. Listening for incoming NMEA stream...\n")
        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
            parse_and_display(line)
    except KeyboardInterrupt:
        print("\nStopping GPS Reader.")
    except Exception as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")

if __name__ == "__main__":
    main()
