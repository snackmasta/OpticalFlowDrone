import time

from smbus2 import SMBus


def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value


bus = SMBus(1)
addr = 0x68

bus.write_byte_data(addr, 0x6B, 0)
time.sleep(0.2)

print("Connected")

while True:
    ax = read_word(bus, addr, 0x3B)
    ay = read_word(bus, addr, 0x3D)
    az = read_word(bus, addr, 0x3F)
    gx = read_word(bus, addr, 0x43)
    gy = read_word(bus, addr, 0x45)
    gz = read_word(bus, addr, 0x47)

    print(ax, ay, az, gx, gy, gz)
    print(ax, ay, az)
    time.sleep(0.1)
