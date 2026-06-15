from smbus2 import SMBus
import time

bus = SMBus(1)
addr = 0x68

bus.write_byte_data(addr, 0x6B, 0)
time.sleep(0.2)

def read_word(reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg+1)
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 65536
    return val

while True:
    # ax = read_word(0x3B)
    # ay = read_word(0x3D)
    # az = read_word(0x3F)
    gx = read_word(0x43)
    gy = read_word(0x45)
    gz = read_word(0x47)

    print(gx, gy, gz)
    # print(ax, ay, az)
    time.sleep(0.1)
