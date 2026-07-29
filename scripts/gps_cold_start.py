import serial
import time
import sys

# Serial port settings matching workspace configuration
SERIAL_PORT = '/dev/ttyAMA2'
BAUD_RATE = 9600

def calc_ubx_checksum(payload):
    """Calculates 2-byte Fletcher-16 checksum for a UBX payload."""
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def create_ubx_packet(msg_class, msg_id, payload):
    """Constructs a complete UBX packet given class, id, and payload."""
    header = b'\xB5\x62'
    cls_id = bytes([msg_class, msg_id])
    length = len(payload).to_bytes(2, byteorder='little')
    raw_packet = cls_id + length + payload
    checksum = calc_ubx_checksum(raw_packet)
    return header + raw_packet + checksum

def send_ubx_command(ser, name, msg_class, msg_id, payload):
    """Sends a UBX command over serial with status feedback."""
    packet = create_ubx_packet(msg_class, msg_id, payload)
    print(f"Sending [{name}] command ({len(packet)} bytes)...")
    ser.write(packet)
    ser.flush()
    time.sleep(0.5)

def cold_start_gps(port=SERIAL_PORT, baud=BAUD_RATE):
    print("=" * 65)
    print("      u-blox GPS Cold Start & Multi-Constellation Config tool")
    print("=" * 65)

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to GPS on {port} @ {baud} baud.\n")
    except Exception as e:
        print(f"Error opening serial port {port}: {e}")
        print("Note: Ensure your GPS module is connected and serial permissions are set.")
        return

    # 1. UBX-CFG-GNSS (0x06 0x3E): Enable All Constellations (GPS, SBAS, Galileo, BeiDou, GLONASS)
    # Message format: msgVer (0x00), numTrkChHw (0x00), numTrkChUse (0xFF), numConfigBlocks (0x05)
    # Block 1: GPS (ID 0)
    # Block 2: SBAS (ID 1)
    # Block 3: Galileo (ID 2)
    # Block 4: BeiDou (ID 3)
    # Block 5: GLONASS (ID 6)
    gnss_payload = bytearray([
        0x00, 0x00, 0xFF, 0x05,  # Header
        # System 0: GPS (enable min 8, max 16 channels)
        0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
        # System 1: SBAS (enable min 1, max 3 channels)
        0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01,
        # System 2: Galileo (enable min 4, max 8 channels)
        0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
        # System 3: BeiDou (enable min 2, max 8 channels)
        0x03, 0x02, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01,
        # System 6: GLONASS (enable min 8, max 14 channels)
        0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01
    ])
    send_ubx_command(ser, "Enable All Constellations (GPS+GLONASS+Galileo+BeiDou)", 0x06, 0x3E, gnss_payload)

    # 2. UBX-CFG-RST (0x06 0x04): Cold Start - Clear Ephemeris, Almanac, UTC, Position & Clock Cache
    # navBbrMask = 0xFFFF (Cold start clear all BBR memory)
    # resetMode = 0x01 (Controlled GNSS reset)
    rst_payload = b'\xFF\xFF\x01\x00'
    send_ubx_command(ser, "COLD START & Clear BBR Cache (Ephemeris/Almanac/Position)", 0x06, 0x04, rst_payload)

    print("\n------------------------------------------------------------")
    print("Cold Start command issued successfully!")
    print("The GPS module memory has been wiped clean.")
    print("It is now searching for satellite signals across all constellations.")
    print("------------------------------------------------------------")
    print("Listening to NMEA stream for lock status (Press Ctrl+C to exit)...\n")

    try:
        start_time = time.time()
        while True:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line:
                elapsed = int(time.time() - start_time)
                if line.startswith('$GNGGA') or line.startswith('$GPGGA') or line.startswith('$GNRMC'):
                    print(f"[{elapsed:3d}s] {line}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting monitoring.")
        ser.close()

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    cold_start_gps(port)
