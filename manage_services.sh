#!/bin/bash

# Configuration
PROJECT_DIR="/home/raspi/Desktop/drone"
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
    # Block until wlan0 is connected
    wait_for_wlan0

    echo "Starting MAVProxy..."
    bash "$PROJECT_DIR/mavproxy.sh" > "$LOG_DIR/mavproxy.log" 2>&1 &

    echo "Starting Static GPS Injector..."
    "$PYTHON_BIN" "$PROJECT_DIR/static_gps_injector.py" > "$LOG_DIR/static_gps_injector.log" 2>&1 &

    echo "Starting HMC5883L Compass..."
    "$PYTHON_BIN" "$PROJECT_DIR/hmc5883l.py" > "$LOG_DIR/hmc5883l.log" 2>&1 &

    echo "Starting Shared Memory Dashboard..."
    "$PYTHON_BIN" "$PROJECT_DIR/read_shared_memory.py" --dashboard --port 5003 > "$LOG_DIR/read_shared_memory.log" 2>&1 &

    echo "Starting Optical Flow Stream..."
    "$PYTHON_BIN" "$PROJECT_DIR/optical_flow_stream.py" -stream > "$LOG_DIR/optical_flow_stream.log" 2>&1 &

    echo "All services started."
}

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

check_status() {
    echo "=== Service Status ==="
    for service in "mavproxy.py" "static_gps_injector.py" "hmc5883l.py" "read_shared_memory.py" "optical_flow_stream.py"; do
        if pgrep -f "$service" > /dev/null; then
            echo -e "  $service: \e[32mRUNNING\e[0m"
        else
            echo -e "  $service: \e[31mSTOPPED\e[0m"
        fi
    done
}

enable_boot() {
    echo "Configuring systemd service for autorun..."
    SERVICE_FILE="/etc/systemd/system/drone.service"
    
    sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=Drone Autonomous Services
After=network.target network-online.target wpa_supplicant.service
Wants=network-online.target

[Service]
Type=forking
User=raspi
WorkingDirectory=$PROJECT_DIR
ExecStart=$PROJECT_DIR/manage_services.sh start
ExecStop=$PROJECT_DIR/manage_services.sh stop
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable drone.service
    echo "Systemd service enabled. It will run automatically on boot."
}

disable_boot() {
    echo "Disabling systemd service..."
    sudo systemctl disable drone.service || true
    sudo rm -f /etc/systemd/system/drone.service
    sudo systemctl daemon-reload
    echo "Systemd service disabled and removed."
}

# Handle command-line arguments if provided
if [ -n "$1" ]; then
    case "$1" in
        start)
            start_services
            ;;
        stop)
            stop_services
            ;;
        restart)
            stop_services
            sleep 2
            start_services
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
            echo "Usage: $0 {start|stop|restart|status|enable-boot|disable-boot}"
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
        
        # 1. MAVProxy
        if pgrep -f "mavproxy.py" > /dev/null; then
            echo -e " 1) MAVProxy: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 1) MAVProxy: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 2. Static GPS Injector
        if pgrep -f "static_gps_injector.py" > /dev/null; then
            echo -e " 2) Static GPS Injector: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 2) Static GPS Injector: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 3. HMC5883L Compass
        if pgrep -f "hmc5883l.py" > /dev/null; then
            echo -e " 3) HMC5883L Compass: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 3) HMC5883L Compass: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 4. Shared Memory Dashboard
        if pgrep -f "read_shared_memory.py --dashboard" > /dev/null; then
            echo -e " 4) Shared Memory Dashboard: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 4) Shared Memory Dashboard: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        # 5. Optical Flow Stream
        if pgrep -f "optical_flow_stream.py -stream" > /dev/null; then
            echo -e " 5) Optical Flow Stream: \e[32mRUNNING\e[0m (Select to STOP)"
        else
            echo -e " 5) Optical Flow Stream: \e[31mSTOPPED\e[0m (Select to START)"
        fi

        echo " 6) Back to main menu"
        echo "============================================="
        read -rp "Select a service to toggle [1-6]: " choice

        case $choice in
            1)
                if pgrep -f "mavproxy.py" > /dev/null; then
                    echo "Stopping MAVProxy..."
                    pkill -f "mavproxy.py"
                else
                    echo "Starting MAVProxy..."
                    bash "$PROJECT_DIR/mavproxy.sh" > "$LOG_DIR/mavproxy.log" 2>&1 &
                fi
                ;;
            2)
                if pgrep -f "static_gps_injector.py" > /dev/null; then
                    echo "Stopping Static GPS Injector..."
                    pkill -f "static_gps_injector.py"
                else
                    echo "Starting Static GPS Injector..."
                    "$PYTHON_BIN" "$PROJECT_DIR/static_gps_injector.py" > "$LOG_DIR/static_gps_injector.log" 2>&1 &
                fi
                ;;
            3)
                if pgrep -f "hmc5883l.py" > /dev/null; then
                    echo "Stopping HMC5883L Compass..."
                    pkill -f "hmc5883l.py"
                else
                    echo "Starting HMC5883L Compass..."
                    "$PYTHON_BIN" "$PROJECT_DIR/hmc5883l.py" > "$LOG_DIR/hmc5883l.log" 2>&1 &
                fi
                ;;
            4)
                if pgrep -f "read_shared_memory.py --dashboard" > /dev/null; then
                    echo "Stopping Shared Memory Dashboard..."
                    pkill -f "read_shared_memory.py --dashboard"
                else
                    echo "Starting Shared Memory Dashboard..."
                    "$PYTHON_BIN" "$PROJECT_DIR/read_shared_memory.py" --dashboard --port 5003 > "$LOG_DIR/read_shared_memory.log" 2>&1 &
                fi
                ;;
            5)
                if pgrep -f "optical_flow_stream.py -stream" > /dev/null; then
                    echo "Stopping Optical Flow Stream..."
                    pkill -f "optical_flow_stream.py -stream"
                    killall mediamtx 2>/dev/null || true
                else
                    echo "Starting Optical Flow Stream..."
                    "$PYTHON_BIN" "$PROJECT_DIR/optical_flow_stream.py" -stream > "$LOG_DIR/optical_flow_stream.log" 2>&1 &
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
    echo "          DRONE SERVICE MANAGER              "
    echo "============================================="
    echo " 1) Start all services"
    echo " 2) Stop all services (delete all)"
    echo " 3) Restart all services"
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
