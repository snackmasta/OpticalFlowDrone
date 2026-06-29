# Service Persistence and Management (`manage_services.sh`)

This document describes in detail how the companion computer manages the runtime lifecycles, background execution, logging redirection, boot persistence, and interactive toggles of the autonomous drone subsystem.

---

## 1. Directory and Path Setup

The script [manage_services.sh](file:///e:/OptFlowDrone/OpticalFlowDrone/manage_services.sh) uses explicit path mappings configured at the top of the file:
* **`PROJECT_DIR`**: `/home/raspi/Desktop/drone` (The absolute path to the workspace root directory).
* **`PYTHON_BIN`**: `$PROJECT_DIR/venv/bin/python` (Ensures dependencies are resolved within the configured python virtual environment).
* **`LOG_DIR`**: `$PROJECT_DIR/logs` (Redirect target for stdout and stderr streams).

On launch, the script guarantees log folder existence:
```bash
mkdir -p "$LOG_DIR"
```

---

## 2. Boot Synchronization Guard (`wait_for_wlan0`)

To prevent python script failures (such as binding errors on Flask servers or connection timeouts on UDP sockets), the services do not start until the local network interface is initialized.

```bash
wait_for_wlan0() {
    echo "Waiting for wlan0 connection (IP address)..."
    local count=1
    while true; do
        if ip addr show dev wlan0 2>/dev/null | grep -q "inet "; then
            # Extract IP address safely using grep/awk
            ip_addr=$(ip addr show dev wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)
            echo "wlan0 is connected with IP: $ip_addr"
            return 0
        fi
        echo "wlan0 not ready yet, retrying in 1s (Attempt: $count)..."
        count=$((count + 1))
        sleep 1
    done
}
```
* **Mechanism:** Polls the state of interface `wlan0` at **1 Hz**.
* **IP Parsing:** Uses a Perl-compatible regular expression `(?<=inet\s)\d+(\.\d+){3}` to extract the IPv4 address.
* **Service Block:** The `start_services` function blocks execution on this loop, ensuring that all dependent servers receive valid bindings.

---

## 3. Detailed Service Commands & Backgrounding

When started, services are launched as decoupled background processes with output streams redirected to files:

### A. MAVProxy
* **Command:** `tail -f /dev/null | bash "$PROJECT_DIR/mavproxy.sh" > "$LOG_DIR/mavproxy.log" 2>&1 &`
* **Details:** Keeps a persistent stdin pipeline open (`tail -f /dev/null`) to prevent MAVProxy from shutting down when standard input closes. Redirects both stdout (`>`) and stderr (`2>&1`) to `logs/mavproxy.log`.

### B. Static GPS Injector
* **Command:** `"$PYTHON_BIN" "$PROJECT_DIR/static_gps_injector.py" > "$LOG_DIR/static_gps_injector.log" 2>&1 &`
* **Details:** Starts the injection loop in the background (`&`), outputting serial NMEA logs to `logs/static_gps_injector.log`.

### C. HMC5883L Compass Driver
* **Command:** `"$PYTHON_BIN" "$PROJECT_DIR/hmc5883l.py" > "$LOG_DIR/hmc5883l.log" 2>&1 &`
* **Details:** Reads magnetometer hardware data via I2C and writes to the shared memory stream. Logs go to `logs/hmc5883l.log`.

### D. Flask Telemetry Dashboard
* **Command:** `"$PYTHON_BIN" "$PROJECT_DIR/read_shared_memory.py" --dashboard --port 5003 > "$LOG_DIR/read_shared_memory.log" 2>&1 &`
* **Details:** Hosts the Flask web dashboard on port `5003`. Redirects all HTTP requests and API debug messages to `logs/read_shared_memory.log`.

### E. Optical Flow Stream
* **Command:** `"$PYTHON_BIN" "$PROJECT_DIR/optical_flow_stream.py" -stream > "$LOG_DIR/optical_flow_stream.log" 2>&1 &`
* **Details:** Starts the Picamera2 capture interface and streams video data. Output redirection is mapped to `logs/optical_flow_stream.log`.

---

## 4. Service Shutdown Protocol (`stop_services`)

To safely stop background processes and close hardware interfaces (like camera locks or I2C channels), the shutdown routine targets process command lines:

```bash
stop_services() {
    echo "Stopping all drone services..."
    pkill -f "mavproxy.py"
    pkill -f "static_gps_injector.py"
    pkill -f "hmc5883l.py"
    pkill -f "read_shared_memory.py --dashboard"
    pkill -f "optical_flow_stream.py -stream"
    killall mediamtx 2>/dev/null || true
    echo "All services stopped."
}
```
* **`pkill -f`**: Matches the pattern against the entire command line arguments, rather than just the process name, ensuring that only the specific Python scripts are terminated.
* **`killall mediamtx`**: Terminates the background RTSP media streaming server.

---

## 5. Systemd Service Integration (`drone.service`)

The boot persistence is configured by creating a systemd service descriptor at `/etc/systemd/system/drone.service`.

### Enable Autorun
Invoking `./manage_services.sh enable-boot` performs the following steps:
1. Writes the configuration:
   ```ini
   [Unit]
   Description=Drone Autonomous Services
   After=network.target network-online.target wpa_supplicant.service
   Wants=network-online.target

   [Service]
   Type=forking
   User=raspi
   WorkingDirectory=/home/raspi/Desktop/drone
   ExecStart=/home/raspi/Desktop/drone/manage_services.sh start
   ExecStop=/home/raspi/Desktop/drone/manage_services.sh stop
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```
2. Reloads systemd daemon rules:
   ```bash
   sudo systemctl daemon-reload
   ```
3. Enables the service:
   ```bash
   sudo systemctl enable drone.service
   ```

### Disable Autorun
Invoking `./manage_services.sh disable-boot` disables and clean up files:
```bash
sudo systemctl disable drone.service || true
sudo rm -f /etc/systemd/system/drone.service
sudo systemctl daemon-reload
```

---

## 6. Interactive Process Toggling

The script provides a loop to selectively start and stop individual services:
* Checks if a process is active using:
  ```bash
  pgrep -f "<script_name>"
  ```
* If running, it displays the status as green `RUNNING` and gives the option to stop it.
* If stopped, it displays the status as red `STOPPED` and gives the option to start it.
* Performs the corresponding start/stop commands in the background on selection.
