#!/bin/bash

## Configuration
PROJECT_DIR="/home/raspi/Desktop/OpticalFlowDrone"
SERVICES_DIR="$PROJECT_DIR/services"
PYTHON_BIN="$PROJECT_DIR/venv/bin/python"
LOG_DIR="$PROJECT_DIR/logs"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

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

start_services() {
    FLOW_MODE="${1:---headless}"

    # Block until wlan0 is connected
    wait_for_wlan0

    echo "Starting HMC5883L Compass..."
    "$PYTHON_BIN" "$SERVICES_DIR/hmc5883l.py" > "$LOG_DIR/hmc5883l.log" 2>&1 &

    echo "Starting High-Priority Optical Flow Stream (Mode: $FLOW_MODE, Priority: nice -n -5)..."
    nice -n -5 "$PYTHON_BIN" "$SERVICES_DIR/optical_flow_stream.py" $FLOW_MODE > "$LOG_DIR/optical_flow_stream.log" 2>&1 &

    echo "Starting Geofence Buzzer Listener..."
    "$PYTHON_BIN" "$SERVICES_DIR/geofence_buzzer_listener.py" > "$LOG_DIR/geofence_buzzer_listener.log" 2>&1 &

    echo "Starting Telemetry UDP Bridge..."
    "$PYTHON_BIN" "$SERVICES_DIR/send_attitude_udp.py" --ip 127.0.0.1 --port 5005 > "$LOG_DIR/send_attitude_udp.log" 2>&1 &

    echo "Starting 3D Geofence Web Server & Dashboard..."
    "$PYTHON_BIN" "$SERVICES_DIR/web_server.py" > "$LOG_DIR/web_server.log" 2>&1 &

    echo "All active Raspberry Pi services started."
}

stop_services() {
    echo "Stopping all active Raspberry Pi system services..."
    pkill -f "hmc5883l.py"
    pkill -f "optical_flow_stream.py"
    pkill -f "geofence_buzzer_listener.py"
    pkill -f "send_attitude_udp.py"
    pkill -f "web_server.py"
    killall mediamtx 2>/dev/null || true
    echo "All services stopped."
}

check_status() {
    echo "=== Active Raspberry Pi Service Status ==="
    for service in "hmc5883l.py" "optical_flow_stream.py" "geofence_buzzer_listener.py" "send_attitude_udp.py" "web_server.py"; do
        if pgrep -f "$service" > /dev/null; then
            echo -e "  $service: \e[32mRUNNING\e[0m"
        else
            echo -e "  $service: \e[31mSTOPPED\e[0m"
        fi
    done
}

enable_boot() {
    echo "Configuring systemd service for autorun..."
    SERVICE_FILE="/etc/systemd/system/opticalflow.service"
    
    sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=Optical Flow Autonomous System Services
After=network.target network-online.target wpa_supplicant.service
Wants=network-online.target

[Service]
Type=forking
User=raspi
WorkingDirectory=$PROJECT_DIR
ExecStart=$SERVICES_DIR/manage_services.sh start
ExecStop=$SERVICES_DIR/manage_services.sh stop
RemainAfterExit=yes`

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable opticalflow.service
    echo "Systemd service enabled. It will run automatically on boot."
}

disable_boot() {
    echo "Disabling systemd service..."
    sudo systemctl disable opticalflow.service || true
    sudo rm -f /etc/systemd/system/opticalflow.service
    sudo systemctl daemon-reload
    echo "Systemd service disabled and removed."
}

# Handle command-line arguments if provided
if [ -n "$1" ]; then
    case "$1" in
        start|start-headless|--headless)
            start_services "--headless"
            ;;
        start-stream|-stream)
            start_services "-stream"
            ;;
        stop)
            stop_services
            ;;
        restart)
            stop_services
            sleep 2
            start_services "--headless"
            ;;
        status)
            check_status
            ;;
        enable-boot)
            enable_boot
            ;;
        disable-boot)
            disable_boot
            ;;
        *)
            echo "Usage: $0 {start|start-headless|start-stream|stop|restart|status|enable-boot|disable-boot}"
            exit 1
            ;;
    esac
    exit 0
fi

toggle_individual_services() {
    while true; do
        echo ""
        echo "============================================="
        echo "          TOGGLE INDIVIDUAL SERVICES         "
        echo "============================================="
        
        # 1. HMC5883L Compass
        if pgrep -f "hmc5883l.py" > /dev/null; then
            echo -e " 1) HMC5883L Compass: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 1) HMC5883L Compass: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 2. Optical Flow Stream
        if pgrep -f "optical_flow_stream.py -stream" > /dev/null; then
            echo -e " 2) Optical Flow Stream: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 2) Optical Flow Stream: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 3. Geofence Buzzer Listener
        if pgrep -f "geofence_buzzer_listener.py" > /dev/null; then
            echo -e " 3) Geofence Buzzer Listener: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 3) Geofence Buzzer Listener: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 4. Telemetry UDP Bridge
        if pgrep -f "send_attitude_udp.py" > /dev/null; then
            echo -e " 4) Telemetry UDP Bridge: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 4) Telemetry UDP Bridge: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 5. 3D Geofence Web Server
        if pgrep -f "web_server.py" > /dev/null; then
            echo -e " 5) 3D Geofence Web Server: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 5) 3D Geofence Web Server: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        echo " 6) Back to main menu"
        echo "============================================="
        read -rp "Select a service to toggle [1-6]: " choice

        case $choice in
            1)
                if pgrep -f "hmc5883l.py" > /dev/null; then
                    echo "Stopping HMC5883L Compass..."
                    pkill -f "hmc5883l.py"
                else
                    echo "Starting HMC5883L Compass..."
                    "$PYTHON_BIN" "$SERVICES_DIR/hmc5883l.py" > "$LOG_DIR/hmc5883l.log" 2>&1 &
                fi
                ;;
            2)
                if pgrep -f "optical_flow_stream.py -stream" > /dev/null; then
                    echo "Stopping Optical Flow Stream..."
                    pkill -f "optical_flow_stream.py -stream"
                    killall mediamtx 2>/dev/null || true
                else
                    echo "Starting Optical Flow Stream..."
                    "$PYTHON_BIN" "$SERVICES_DIR/optical_flow_stream.py" -stream > "$LOG_DIR/optical_flow_stream.log" 2>&1 &
                fi
                ;;
            3)
                if pgrep -f "geofence_buzzer_listener.py" > /dev/null; then
                    echo "Stopping Geofence Buzzer Listener..."
                    pkill -f "geofence_buzzer_listener.py"
                else
                    echo "Starting Geofence Buzzer Listener..."
                    "$PYTHON_BIN" "$SERVICES_DIR/geofence_buzzer_listener.py" > "$LOG_DIR/geofence_buzzer_listener.log" 2>&1 &
                fi
                ;;
            4)
                if pgrep -f "send_attitude_udp.py" > /dev/null; then
                    echo "Stopping Telemetry UDP Bridge..."
                    pkill -f "send_attitude_udp.py"
                else
                    echo "Starting Telemetry UDP Bridge..."
                    "$PYTHON_BIN" "$SERVICES_DIR/send_attitude_udp.py" --ip 127.0.0.1 --port 5005 > "$LOG_DIR/send_attitude_udp.log" 2>&1 &
                fi
                ;;
            5)
                if pgrep -f "web_server.py" > /dev/null; then
                    echo "Stopping 3D Geofence Web Server..."
                    pkill -f "web_server.py"
                else
                    echo "Starting 3D Geofence Web Server..."
                    "$PYTHON_BIN" "$SERVICES_DIR/web_server.py" > "$LOG_DIR/web_server.log" 2>&1 &
                fi
                ;;
            6)
                return 0
                ;;
            *)
                echo "Invalid option."
                ;;
        esac
        sleep 1
    done
}

# Fallback to interactive CLI menu if no arguments are provided
while true; do
    echo ""
    echo "============================================="
    echo "         SYSTEM SERVICE MANAGER              "
    echo "============================================="
    echo " 1) Start all active services"
    echo " 2) Stop all active services"
    echo " 3) Restart all active services"
    echo " 4) Check services status"
    echo " 5) Toggle individual services (start/stop)"
    echo " 6) Enable autorun on boot (systemd)"
    echo " 7) Disable autorun on boot"
    echo " 8) Exit"
    echo "============================================="
    read -rp "Choose an option [1-8]: " opt

    case $opt in
        1)
            start_services
            ;;
        2)
            stop_services
            ;;
        3)
            stop_services
            sleep 2
            start_services
            ;;
        4)
            check_status
            ;;
        5)
            toggle_individual_services
            ;;
        6)
            enable_boot
            ;;
        7)
            disable_boot
            ;;
        8)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo "Invalid option."
            ;;
    esac
done
