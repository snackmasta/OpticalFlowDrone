#!/bin/bash

# Configuration
PROJECT_DIR="/home/raspi/Desktop/drone"
PYTHON_BIN="$PROJECT_DIR/venv/bin/python"
LOG_DIR="$PROJECT_DIR/logs"

# Ensure log directory exists
mkdir -p "$LOG_DIR"

start_services() {
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
After=network.target

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
    echo " 5) Enable autorun on boot (systemd)"
    echo " 6) Disable autorun on boot"
    echo " 7) Exit"
    echo "============================================="
    read -rp "Choose an option [1-7]: " opt

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
            enable_boot
            ;;
        6)
            disable_boot
            ;;
        7)
            echo "Exiting."
            exit 0
            ;;
        *)
            echo "Invalid option."
            ;;
    esac
done
