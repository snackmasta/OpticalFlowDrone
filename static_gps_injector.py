import serial
import time
from datetime import datetime

ser = serial.Serial(
    "/dev/ttyAMA2",   # change UART
    38400
)

LAT = 25.410000
LON = 121.210000
ALT = 0


def checksum(s):
    c = 0
    for x in s:
        c ^= ord(x)
    return f"{c:02X}"


def to_nmea_lat(lat):
    deg = int(abs(lat))
    minutes = (abs(lat)-deg)*60
    hemi = "N" if lat >= 0 else "S"
    return f"{deg:02d}{minutes:07.4f}", hemi


def to_nmea_lon(lon):
    deg = int(abs(lon))
    minutes = (abs(lon)-deg)*60
    hemi = "E" if lon >= 0 else "W"
    return f"{deg:03d}{minutes:07.4f}", hemi


while True:

    now = datetime.utcnow()

    t = now.strftime("%H%M%S")

    lat, ns = to_nmea_lat(LAT)
    lon, ew = to_nmea_lon(LON)

    gga = (
        f"GPGGA,{t},{lat},{ns},{lon},{ew},1,10,1.0,"
        f"{ALT:.1f},M,0,M,,"
    )

    rmc = (
        f"GPRMC,{t},A,{lat},{ns},{lon},{ew},"
        f"0.0,0.0,{now.strftime('%d%m%y')},,,A"
    )

    for msg in [gga, rmc]:

        line = f"${msg}*{checksum(msg)}\r\n"

        ser.write(line.encode())

        print(line.strip())

    time.sleep(0.2)