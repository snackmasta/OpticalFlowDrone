import serial
import time
import struct
from datetime import datetime
from multiprocessing import shared_memory

ser = serial.Serial(
    "/dev/ttyAMA2",
    38400
)

# Rectangle movement configuration
START_LAT = -6.9175    # Bandung Latitude
START_LON = 107.6191   # Bandung Longitude
LAT_SIZE = 0.0005  # Height of rectangle in degrees
LON_SIZE = 0.0005  # Width of rectangle in degrees
STEP = 0.00002     # Speed of movement per step
MAX_LAPS = 0       # Stop moving after this many laps (set to None for infinite)

lat = START_LAT
lon = START_LON
state = 0  # 0: Go East, 1: Go North, 2: Go West, 3: Go South
laps_completed = 0


# Compass shared memory configuration
COMPASS_SHM_NAME = "compass_heading_stream"
COMPASS_SHM_MAGIC = b"CHDG"
COMPASS_SHM_HEADER_FORMAT = "<4sII"
COMPASS_SHM_RECORD_FORMAT = "<6d"
COMPASS_SHM_HEADER_SIZE = struct.calcsize(COMPASS_SHM_HEADER_FORMAT)
COMPASS_SHM_RECORD_SIZE = struct.calcsize(COMPASS_SHM_RECORD_FORMAT)
COMPASS_MAX_SAMPLES = 120

compass_shm = None

# Optical flow shared memory configuration
FLOW_SHM_NAME = "optical_flow_stream"
FLOW_SHM_MAGIC = b"FLOW"
FLOW_SHM_HEADER_FORMAT = "<4sII"
FLOW_SHM_RECORD_FORMAT = "<11d"
FLOW_SHM_HEADER_SIZE = struct.calcsize(FLOW_SHM_HEADER_FORMAT)
FLOW_SHM_RECORD_SIZE = struct.calcsize(FLOW_SHM_RECORD_FORMAT)
FLOW_MAX_SAMPLES = 120

flow_shm = None


def get_latest_flow_position():
    global flow_shm
    if flow_shm is None:
        try:
            flow_shm = shared_memory.SharedMemory(name=FLOW_SHM_NAME)
            try:
                from multiprocessing import resource_tracker
                resource_tracker.unregister(flow_shm._name, "shared_memory")
            except Exception:
                pass
        except FileNotFoundError:
            return None
    try:
        magic, write_index, sample_count = struct.unpack_from(FLOW_SHM_HEADER_FORMAT, flow_shm.buf, 0)
        if magic != FLOW_SHM_MAGIC or sample_count == 0:
            return None
        
        latest_index = (write_index - 1) % FLOW_MAX_SAMPLES
        offset = FLOW_SHM_HEADER_SIZE + (latest_index * FLOW_SHM_RECORD_SIZE)
        # Fields: timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt
        values = struct.unpack_from(FLOW_SHM_RECORD_FORMAT, flow_shm.buf, offset)
        x_cm, y_cm = values[1], values[2]
        return x_cm / 100.0, y_cm / 100.0 # Convert x_cm, y_cm to x_m, y_m
    except Exception:
        try:
            flow_shm.close()
        except Exception:
            pass
        flow_shm = None
        return None


def get_latest_compass_heading():
    global compass_shm
    if compass_shm is None:
        try:
            compass_shm = shared_memory.SharedMemory(name=COMPASS_SHM_NAME)
            try:
                from multiprocessing import resource_tracker
                resource_tracker.unregister(compass_shm._name, "shared_memory")
            except Exception:
                pass
        except FileNotFoundError:
            return None
    try:
        magic, write_index, sample_count = struct.unpack_from(COMPASS_SHM_HEADER_FORMAT, compass_shm.buf, 0)
        if magic != COMPASS_SHM_MAGIC or sample_count == 0:
            return None
        
        latest_index = (write_index - 1) % COMPASS_MAX_SAMPLES
        record_offset = COMPASS_SHM_HEADER_SIZE + (latest_index * COMPASS_SHM_RECORD_SIZE)
        # Record: timestamp, raw_heading, heading, x, y, z
        _, _, heading, _, _, _ = struct.unpack_from(COMPASS_SHM_RECORD_FORMAT, compass_shm.buf, record_offset)
        return heading
    except Exception:
        try:
            compass_shm.close()
        except Exception:
            pass
        compass_shm = None
        return None


def checksum(s):
    c = 0
    for x in s:
        c ^= ord(x)
    return f"{c:02X}"


def to_nmea_lat(lat):
    deg = int(abs(lat))
    minutes = (abs(lat)-deg)*60
    ns = "S" if lat < 0 else "N"
    return f"{deg:02d}{minutes:09.6f}", ns


def to_nmea_lon(lon):
    deg = int(abs(lon))
    minutes = (abs(lon)-deg)*60
    ew = "W" if lon < 0 else "E"
    return f"{deg:03d}{minutes:09.6f}", ew


# GPS Packet default values
FIX_QUALITY = "4"      # RTK Fixed
NUM_SATELLITES = "30"
HDOP = "0.1"           # Excellent HDOP for RTK
PDOP = "1.2"
VDOP = "0.9"           # Excellent VDOP for RTK altitude
ALTITUDE = "100.0"       # altitude neutral
ALTITUDE_UNIT = "M"
GEOIDAL_HEIGHT = "0.0"
GEOIDAL_HEIGHT_UNIT = "M"

RMC_STATUS = "A"
SPEED_OVER_GROUND = "0.0"
TRACK_ANGLE = "0.0"
MAG_VAR = ""
MAG_VAR_DIR = ""
MODE_INDICATOR = "A"


while True:

    now = datetime.utcnow()

    # Format UTC time with fractional seconds (hhmmss.ss) for high-rate GPS updates
    utc = now.strftime("%H%M%S.%f")[:-4]
    date = now.strftime("%d%m%y")

    # Read the latest flow position (in absolute meters: East = x, North = y)
    import math
    flow_pos = get_latest_flow_position()
    if flow_pos is not None:
        x_m, y_m = flow_pos
        # Earth radius in meters
        EARTH_RADIUS = 6378137.0
        # Convert meters displacement to degrees latitude and longitude
        lat = START_LAT + (y_m / EARTH_RADIUS) * (180.0 / math.pi)
        lon = START_LON + (x_m / EARTH_RADIUS) / math.cos(math.radians(lat)) * (180.0 / math.pi)

    nmea_lat, ns = to_nmea_lat(lat)
    nmea_lon, ew = to_nmea_lon(lon)

    gga = (
        f"GPGGA,"
        f"{utc},"
        f"{nmea_lat},{ns},"
        f"{nmea_lon},{ew},"
        f"{FIX_QUALITY},"
        f"{NUM_SATELLITES},"
        f"{HDOP},"
        f"{ALTITUDE},{ALTITUDE_UNIT},"
        f"{GEOIDAL_HEIGHT},{GEOIDAL_HEIGHT_UNIT},,"
    )

    rmc = (
        f"GPRMC,"
        f"{utc},"
        f"{RMC_STATUS},"
        f"{nmea_lat},{ns},"
        f"{nmea_lon},{ew},"
        f"{SPEED_OVER_GROUND},"
        f"{TRACK_ANGLE},"
        f"{date},"
        f"{MAG_VAR},"
        f"{MAG_VAR_DIR},"
        f"{MODE_INDICATOR}"
    )

    gsa = (
        f"GPGSA,"
        f"A,"
        f"3,"
        f"01,02,03,04,05,06,07,08,09,10,,,"
        f"{PDOP},"
        f"{HDOP},"
        f"{VDOP}"
    )

    # Read the latest heading from compass shared memory, falling back to "0.0" if unavailable
    heading = get_latest_compass_heading()
    if heading is not None:
        heading = (360.0 - heading) % 360.0
    current_yaw = f"{heading:.1f}" if heading is not None else "0.0"


    print(f"Lat: {lat:.7f}, Lon: {lon:.7f}, Yaw: {current_yaw}")

    hdt = (
        f"GPHDT,"
        f"{current_yaw},"
        f"T"
    )

    for msg in (gga, rmc, gsa, hdt):
        line = f"${msg}*{checksum(msg)}\r\n"
        ser.write(line.encode())

    time.sleep(0.1)