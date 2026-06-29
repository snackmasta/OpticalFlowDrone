# Hardware Communication Protocols

This document details the low-level physical interfaces, hardware protocols, register mappings, and configuration settings used by the companion computer to interact with the drone's sensors and flight controller.

---

## 1. Physical Serial Communication (UART)

The system relies on two hardware UART interfaces to exchange data with the flight controller (autopilot):

```
                     +----------------------------+
                     |       Raspberry Pi         |
                     +----+------------------+----+
                          |                  |
             UART0 (/dev/serial0)       UART2 (/dev/ttyAMA2)
             [MAVLink Telemetry]        [Synthetic GPS NMEA]
                          |                  |
                          v                  v
                     +----+------------------+----+
                     |       Flight Controller    |
                     +----------------------------+
```

### A. Autopilot MAVLink Interface (`/dev/serial0`)
* **Hardware Port:** Raspberry Pi UART0.
* **Pin Configuration:** Physical Pin 8 (TXD0) & Physical Pin 10 (RXD0).
* **Connection Target:** Pixhawk Telemetry 1 (TELEM 1) port.
* **Serial Settings:** Baud Rate: **921600**, Data Bits: 8, Parity: None, Stop Bits: 1 (8N1).
* **Protocol:** MAVLink v2. Used to passively stream sensor data (Rangefinder metrics) and status data (arm state, battery telemetry) to companion processes.

### B. GPS Injection Interface (`/dev/ttyAMA2`)
* **Hardware Port:** Raspberry Pi UART2 (enabled via overlay configurations).
* **Pin Configuration:** Physical Pin 27 (TXD2) & Physical Pin 28 (RXD2).
* **Connection Target:** Pixhawk GPS 1 / GPS 2 port.
* **Serial Settings:** Baud Rate: **38400**, Data Bits: 8, Parity: None, Stop Bits: 1 (8N1).
* **Protocol:** NMEA 0183. Emulates a hardware GPS receiver to feed coordinates calculated via visual dead reckoning back to the flight controller.

---

## 2. I2C Bus Communication (SMBus Protocol)

The companion computer uses the I2C serial bus interface to read raw physical data from local sensors. Both sensors share **I2C Bus 1** on Physical Pin 3 (SDA / GPIO 2) and Physical Pin 5 (SCL / GPIO 3), configured at **400 kHz (Fast Mode)**.

```
                  +-----------------------------------+
                  |         I2C Bus 1 (Pins 3/5)      |
                  +-------+-------------------+-------+
                          |                   |
                          v                   v
                     [Addr 0x68]         [Addr 0x1E]
                     MPU6050 IMU      HMC5883L Magnetometer
```

### A. MPU6050 6-Axis IMU
* **Device Address:** `0x68`
* **Configuration Sequence:**
  * Wake up the device by writing `0` to register `0x6B` (PWR_MGMT_1) to exit sleep mode.
* **Register Memory Map:**
  | Register | Size | Description | Unit Scale |
  | :--- | :--- | :--- | :--- |
  | `0x3B` (H) / `0x3C` (L) | 16-bit | Accelerometer X Axis | $16384.0 \text{ LSB/g}$ |
  | `0x3D` (H) / `0x3E` (L) | 16-bit | Accelerometer Y Axis | $16384.0 \text{ LSB/g}$ |
  | `0x3F` (H) / `0x40` (L) | 16-bit | Accelerometer Z Axis | $16384.0 \text{ LSB/g}$ |
  | `0x43` (H) / `0x44` (L) | 16-bit | Gyroscope X Axis | $131.0 \text{ LSB/dps}$ |
  | `0x45` (H) / `0x46` (L) | 16-bit | Gyroscope Y Axis | $131.0 \text{ LSB/dps}$ |
  | `0x47` (H) / `0x48` (L) | 16-bit | Gyroscope Z Axis | $131.0 \text{ LSB/dps}$ |

### B. HMC5883L 3-Axis Magnetometer (Compass)
* **Device Address:** `0x1E`
* **Configuration Registers:**
  * `0x00` (Config Register A): Configures sensor sampling rate and averaging parameters.
  * `0x01` (Config Register B): Configures measurement gain level.
  * `0x02` (Mode Register): Writes `0` to configure continuous measurement mode.
* **Register Memory Map:**
  | Register | Size | Description |
  | :--- | :--- | :--- |
  | `0x03` (MSB) / `0x04` (LSB) | 16-bit | Magnetometer X Axis |
  | `0x05` (MSB) / `0x06` (LSB) | 16-bit | Magnetometer Z Axis |
  | `0x07` (MSB) / `0x08` (LSB) | 16-bit | Magnetometer Y Axis |

---

## 3. MIPI CSI Camera Interface

* **Physical Connection:** 15-pin MIPI CSI (Camera Serial Interface) ribbon cable linking the Pi's CSI port directly to the image sensor's processor.
* **Software API:** Raspberry Pi `Picamera2` pipeline.
* **Channel Characteristics:**
  * Raw sensor data is moved from the camera hardware module directly into system memory (RAM) via DMA (Direct Memory Access).
  * Framerates and resolutions (e.g. 640x480 at 30 FPS) are configured at the GPU/V4L2 hardware driver layer before image frames are converted to numpy arrays for optical flow analysis.
