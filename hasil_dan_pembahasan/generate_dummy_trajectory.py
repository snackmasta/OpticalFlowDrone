#!/usr/bin/env python3
"""
Utility script to generate a simulated 3D flight trajectory log.
Creates a spiral climb trajectory for testing the 3D analysis script.
"""

import os
import csv
import math
import random

OUTPUT_DIR = "hasil_dan_pembahasan"
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "optical_flow_shm_log.csv")

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

sample_count = 500
dt = 0.05
radius_cm = 100.0
freq_hz = 0.05  # 20 seconds per circle

with open(OUTPUT_CSV, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "Timestamp (s)",
        "X Position (cm)", "Y Position (cm)",
        "Raw X Position (cm)", "Raw Y Position (cm)",
        "VX (m/s)", "VY (m/s)",
        "Raw VX (m/s)", "Raw VY (m/s)",
        "Altitude (m)", "Heading (deg)"
    ])
    
    for i in range(sample_count):
        t = i * dt
        theta = 2.0 * math.pi * freq_hz * t
        
        # Spiral motion: radius expands slightly as time increases
        r = radius_cm * (1.0 + 0.05 * t)
        x_pos = r * math.cos(theta)
        y_pos = r * math.sin(theta)
        
        x_raw = x_pos + random.gauss(0, 2.0)
        y_raw = y_pos + random.gauss(0, 2.0)
        
        vx = -2.0 * math.pi * freq_hz * (r / 100.0) * math.sin(theta)
        vy = 2.0 * math.pi * freq_hz * (r / 100.0) * math.cos(theta)
        vx_raw = vx + random.gauss(0, 0.05)
        vy_raw = vy + random.gauss(0, 0.05)
        
        # Climbing altitude from 0.5m to 2.0m
        alt = 0.5 + 1.5 * (t / (sample_count * dt))
        heading = (theta * 180.0 / math.pi) % 360.0
        
        writer.writerow([
            f"{t:.3f}",
            f"{x_pos:.4f}", f"{y_pos:.4f}",
            f"{x_raw:.4f}", f"{y_raw:.4f}",
            f"{vx:.4f}", f"{vy:.4f}",
            f"{vx_raw:.4f}", f"{vy_raw:.4f}",
            f"{alt:.4f}", f"{heading:.4f}"
        ])

print(f"Berhasil menggenerasi dummy flight trajectory log di: {OUTPUT_CSV}")
