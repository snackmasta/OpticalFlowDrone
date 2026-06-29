# Raspberry Pi 4 Model B Configuration Guide

This document provides a detailed walkthrough for configuring a clean **Raspberry Pi 4 Model B** to host and run the drone's optical flow, telemetry, and GPS injection services.

---

## 1. Operating System Installation
It is highly recommended to run the 64-bit version of **Raspberry Pi OS Lite** (Bookworm or Bullseye) to minimize GPU and GUI memory overhead.

1. Flash the OS using **Raspberry Pi Imager**.
2. Configure a default user named `raspi` during the flashing setup.
3. Enable **SSH** under the services tab.

---

## 2. Linux User Group Permissions

To allow python scripts to access hardware pins (I2C, Serial UARTs, and the CSI camera) without requiring root/sudo privileges:

```bash
sudo usermod -a -G i2c,dialout,video raspi
```
* **`i2c`**: Grants access to read and write to the MPU6050 and HMC5883L devices via `/dev/i2c-1`.
* **`dialout`**: Grants access to open and write to UART ports (`/dev/serial0` and `/dev/ttyAMA2`).
* **`video`**: Grants access to control the CSI camera stream via V4L2.

*Note: You must log out and log back in for group changes to take effect.*

---

## 3. Boot Overlay Settings (`config.txt`)

You must configure hardware overlays in the boot configuration file (located at `/boot/config.txt` or `/boot/firmware/config.txt` on Bookworm OS). Open the file:

```bash
sudo nano /boot/firmware/config.txt
```

Append or verify the following configuration blocks at the bottom of the file:

```ini
# Enable primary UART (serial0) for MAVLink telemetry
enable_uart=1

# Enable secondary UART2 (ttyAMA2) on GPIO pins 8 and 9
dtoverlay=uart2

# Enable hardware I2C Bus 1
dtparam=i2c_arm=on
# Overclock the I2C bus to 400kHz Fast Mode (improves IMU read rate)
dtparam=i2c_arm_baudrate=400000

# Enable Raspberry Pi CSI Camera autodetect
camera_auto_detect=1
```

---

## 4. Disabling Linux Serial Console

By default, Raspberry Pi OS spawns a root login shell on the primary UART (`serial0`), which will spam boot messages to the autopilot and crash the MAVLink pipeline.

1. Open the boot command-line parameters configuration:
   ```bash
   sudo nano /boot/firmware/cmdline.txt
   ```
2. Find the console mapping parameters (e.g., `console=serial0,115200` or `console=ttyAMA0,115200`).
3. **Delete** that specific console argument from the string. Do not split the parameters onto new lines; keep everything as a single line of space-separated arguments.
4. Save and exit.
5. Disable the systemd serial terminal service:
   ```bash
   sudo systemctl stop serial-getty@ttyAMA0.service
   sudo systemctl disable serial-getty@ttyAMA0.service
   ```

---

## 5. Software Stack Setup

Once the hardware parameters are configured, clone this repository to `/home/raspi/Desktop/drone` and initialize the software stack:

```bash
# Update local packages
sudo apt update && sudo apt upgrade -y

# Install required system libraries
sudo apt install -y python3-pip python3-venv python3-numpy python3-opencv libcap-dev gstreamer1.0-plugins-bad gstreamer1.0-libav

# Initialize the Python Virtual Environment
cd /home/raspi/Desktop/drone
python3 -m venv venv
source venv/bin/activate

# Install Python requirements
pip install --upgrade pip
pip install -r requirements.txt
```

---

## 6. Verification Commands

To verify that the hardware configurations succeeded after a reboot (`sudo reboot`):

* **Check I2C Devices**: Run `i2cdetect -y 1`. You should see address `1E` (HMC5883L) and address `68` (MPU6050) active in the matrix.
* **Verify UART2**: Run `ls -l /dev/ttyAMA2`. It should show the device mapped to group `dialout`.
* **Verify Camera**: Run `rpicam-still --list-cameras` (on Bookworm) to ensure the CSI camera is detected.
* **Start Services**: Launch `./manage_services.sh` to run the stack.
