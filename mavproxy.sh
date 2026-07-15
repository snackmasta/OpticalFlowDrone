#!/bin/bash

# Exit immediately if a command fails

set -e

# Navigate to project directory

cd ~/Desktop/drone

# Activate virtual environment

source venv/bin/activate

# Run MAVProxy

python venv/bin/mavproxy.py --master=/dev/ttyAMA0 --baudrate 460800 --out=udp:192.168.137.1:14555 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --out=udp:127.0.0.1:14552

