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


def get_latest_compass_heading():
    global compass_shm
    if compass_shm is None:
        try:
            compass_shm = shared_memory.SharedMemory(name=COMPASS_SHM_NAME)
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
    return f"{deg:02d}{minutes:07.4f}", ns


def to_nmea_lon(lon):
    deg = int(abs(lon))
    minutes = (abs(lon)-deg)*60
    ew = "W" if lon < 0 else "E"
    return f"{deg:03d}{minutes:07.4f}", ew


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

    # Update position along rectangle path if max laps not reached
    if MAX_LAPS is None or laps_completed < MAX_LAPS:
        if state == 0:
            lon += STEP
            if lon >= START_LON + LON_SIZE:
                lon = START_LON + LON_SIZE
                state = 1
        elif state == 1:
            lat += STEP
            if lat >= START_LAT + LAT_SIZE:
                lat = START_LAT + LAT_SIZE
                state = 2
        elif state == 2:
            lon -= STEP
            if lon <= START_LON:
                lon = START_LON
                state = 3
        elif state == 3:
            lat -= STEP
            if lat <= START_LAT:
                lat = START_LAT
                state = 0
                laps_completed += 1

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
    current_yaw = f"{heading:.1f}" if heading is not None else "0.0"

    hdt = (
        f"GPHDT,"
        f"{current_yaw},"
        f"T"
    )

    for msg in (gga, rmc, gsa, hdt):
        line = f"${msg}*{checksum(msg)}\r\n"
        ser.write(line.encode())

    time.sleep(0.2)