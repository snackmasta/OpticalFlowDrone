import serial

def nmea_to_decimal(raw_val, direction):
    if not raw_val:
        return None
    dot_idx = raw_val.find('.')
    deg_len = dot_idx - 2  # 2 digits for Lat (DD), 3 digits for Lon (DDD)
    degrees = float(raw_val[:deg_len])
    minutes = float(raw_val[deg_len:])
    decimal = degrees + (minutes / 60.0)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

ser = serial.Serial('/dev/ttyAMA2', 9600, timeout=1)

while True:
    line = ser.readline().decode('ascii', errors='replace').strip()
    parts = line.split(',')
    
    # Check for GNGGA or GNRMC sentences
    if parts[0] in ['$GNGGA', '$GNRMC'] and len(parts) > 5:
        raw_lat, lat_dir = parts[2], parts[3]
        raw_lon, lon_dir = parts[4], parts[5]
        
        if raw_lat and raw_lon:
            lat = nmea_to_decimal(raw_lat, lat_dir)
            lon = nmea_to_decimal(raw_lon, lon_dir)
            print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")
        else:
            print("Searching for satellites (No fix)...")
