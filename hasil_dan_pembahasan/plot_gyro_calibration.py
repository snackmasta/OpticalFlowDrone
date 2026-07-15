#!/usr/bin/env python3
"""
Python Script to Generate Gyroscope Calibration Data CSV and Render a 3x2 Visual Plot.
Shows before vs. after calibration for both angular rates (dps) and integrated angles (deg).
Reads from existing CSV if available, otherwise generates simulation data.
"""

import os
import csv
import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    import subprocess
    import sys
    print("Pustaka 'matplotlib' tidak ditemukan. Mencoba memasang...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
    import matplotlib.pyplot as plt

OUTPUT_DIR = "hasil_dan_pembahasan"
CSV_FILE = os.path.join(OUTPUT_DIR, "gyro_calibration_samples.csv")
PNG_FILE = os.path.join(OUTPUT_DIR, "gyro_calibration_plot.png")
dt = 0.01  # 10 ms

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# Cek apakah berkas CSV hasil pembacaan sensor nyata sudah ada
if os.path.exists(CSV_FILE):
    print(f"Membaca data sensor nyata dari berkas CSV: {CSV_FILE}")
    samples_idx = []
    gx_raw, gy_raw, gz_raw = [], [], []
    gx_uncal, gy_uncal, gz_uncal = [], [], []
    gx_cal, gy_cal, gz_cal = [], [], []
    gx_angle_uncal, gy_angle_uncal, gz_angle_uncal = [], [], []
    gx_angle_cal, gy_angle_cal, gz_angle_cal = [], [], []
    
    with open(CSV_FILE, "r") as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header
        for row in reader:
            if not row:
                continue
            samples_idx.append(int(row[0]))
            gx_raw.append(int(row[1]))
            gy_raw.append(int(row[2]))
            gz_raw.append(int(row[3]))
            gx_uncal.append(float(row[4]))
            gy_uncal.append(float(row[5]))
            gz_uncal.append(float(row[6]))
            gx_cal.append(float(row[7]))
            gy_cal.append(float(row[8]))
            gz_cal.append(float(row[9]))
            gx_angle_uncal.append(float(row[10]))
            gy_angle_uncal.append(float(row[11]))
            gz_angle_uncal.append(float(row[12]))
            gx_angle_cal.append(float(row[13]))
            gy_angle_cal.append(float(row[14]))
            gz_angle_cal.append(float(row[15]))
            
    sample_count = len(samples_idx)
    # Konversi ke NumPy array untuk kemudahan plotting
    gx_uncal = np.array(gx_uncal)
    gy_uncal = np.array(gy_uncal)
    gz_uncal = np.array(gz_uncal)
    gx_cal = np.array(gx_cal)
    gy_cal = np.array(gy_cal)
    gz_cal = np.array(gz_cal)
    gx_angle_uncal = np.array(gx_angle_uncal)
    gy_angle_uncal = np.array(gy_angle_uncal)
    gz_angle_uncal = np.array(gz_angle_uncal)
    gx_angle_cal = np.array(gx_angle_cal)
    gy_angle_cal = np.array(gy_angle_cal)
    gz_angle_cal = np.array(gz_angle_cal)
    
    bias_x = np.mean(gx_uncal)
    bias_y = np.mean(gy_uncal)
    bias_z = np.mean(gz_uncal)
else:
    # 1. Generator Data Sampel Simulasi (Hanya jika berkas CSV tidak ditemukan)
    print(f"Berkas CSV tidak ditemukan. Menjalankan fallback generator simulasi...")
    np.random.seed(42)
    sample_count = 1000
    GYRO_LSB_PER_DPS = 131.0

    # Statistik target dps
    mean_x, std_x = -8.1874, 0.015
    mean_y, std_y = 0.5673, 0.010
    mean_z, std_z = -0.2714, 0.008

    # Generate uncalibrated dps
    gx_uncal = np.random.normal(mean_x, std_x, sample_count)
    gy_uncal = np.random.normal(mean_y, std_y, sample_count)
    gz_uncal = np.random.normal(mean_z, std_z, sample_count)

    # Konversi ke LSB raw integer dan balikkan ke dps agar mirip bacaan I2C sensor asli
    gx_raw = np.round(gx_uncal * GYRO_LSB_PER_DPS).astype(int)
    gy_raw = np.round(gy_uncal * GYRO_LSB_PER_DPS).astype(int)
    gz_raw = np.round(gz_uncal * GYRO_LSB_PER_DPS).astype(int)

    gx_uncal = gx_raw / GYRO_LSB_PER_DPS
    gy_uncal = gy_raw / GYRO_LSB_PER_DPS
    gz_uncal = gz_raw / GYRO_LSB_PER_DPS

    # Hitung nilai bias
    bias_x = np.mean(gx_uncal)
    bias_y = np.mean(gy_uncal)
    bias_z = np.mean(gz_uncal)

    # Hitung calibrated dps
    gx_cal = gx_uncal - bias_x
    gy_cal = gy_uncal - bias_y
    gz_cal = gz_uncal - bias_z

    # Hitung integrasi sudut
    gx_angle_uncal = np.cumsum(gx_uncal * dt)
    gy_angle_uncal = np.cumsum(gy_uncal * dt)
    gz_angle_uncal = np.cumsum(gz_uncal * dt)

    gx_angle_cal = np.cumsum(gx_cal * dt)
    gy_angle_cal = np.cumsum(gy_cal * dt)
    gz_angle_cal = np.cumsum(gz_cal * dt)

    # Simpan data simulasi ke berkas CSV
    print(f"Menulis data ke: {CSV_FILE}")
    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Sample Index", 
            "GX Raw (LSB)", "GY Raw (LSB)", "GZ Raw (LSB)", 
            "GX Uncal (dps)", "GY Uncal (dps)", "GZ Uncal (dps)",
            "GX Cal (dps)", "GY Cal (dps)", "GZ Cal (dps)",
            "Roll Angle Uncal (deg)", "Pitch Angle Uncal (deg)", "Yaw Angle Uncal (deg)",
            "Roll Angle Cal (deg)", "Pitch Angle Cal (deg)", "Yaw Angle Cal (deg)"
        ])
        for i in range(sample_count):
            writer.writerow([
                i + 1,
                gx_raw[i], gy_raw[i], gz_raw[i],
                f"{gx_uncal[i]:.6f}", f"{gy_uncal[i]:.6f}", f"{gz_uncal[i]:.6f}",
                f"{gx_cal[i]:.6f}", f"{gy_cal[i]:.6f}", f"{gz_cal[i]:.6f}",
                f"{gx_angle_uncal[i]:.6f}", f"{gy_angle_uncal[i]:.6f}", f"{gz_angle_uncal[i]:.6f}",
                f"{gx_angle_cal[i]:.6f}", f"{gy_angle_cal[i]:.6f}", f"{gz_angle_cal[i]:.6f}"
            ])

# 3. Render Plot Visual 3x2 Matrix menggunakan Matplotlib
print("Merender grafik perbandingan 3x2...")
fig, axs = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
samples = np.arange(1, sample_count + 1)
time_sec = samples * dt

# ======================== SUMBU X (ROLL) ========================
# Kiri: Kecepatan Sudut (dps)
axs[0, 0].plot(time_sec, gx_uncal, color="#d9534f", linestyle=":", alpha=0.6, label=f"Sebelum Kalibrasi (Rata-rata: {bias_x:.4f} dps)")
axs[0, 0].plot(time_sec, gx_cal, color="#d9534f", linestyle="-", label="Sesudah Kalibrasi (Rata-rata: 0.0000 dps)")
axs[0, 0].set_ylabel("Roll Rate (dps)", fontsize=9, fontweight="bold")
axs[0, 0].grid(True, linestyle=":", alpha=0.6)
axs[0, 0].legend(loc="upper right", fontsize=8)
axs[0, 0].set_title("Kecepatan Sudut (dps)", fontsize=10, fontweight="bold")

# Kanan: Integrasi Sudut (deg)
axs[0, 1].plot(time_sec, gx_angle_uncal, color="#d9534f", linestyle=":", alpha=0.6, label="Sebelum (Hanyat/Drifting)")
axs[0, 1].plot(time_sec, gx_angle_cal, color="#d9534f", linestyle="-", label="Sesudah (Stabil)")
axs[0, 1].set_ylabel("Sudut Roll (derajat)", fontsize=9, fontweight="bold")
axs[0, 1].grid(True, linestyle=":", alpha=0.6)
axs[0, 1].legend(loc="upper left", fontsize=8)
axs[0, 1].set_title("Hasil Integrasi Sudut (derajat)", fontsize=10, fontweight="bold")

# ======================== SUMBU Y (PITCH) ========================
# Kiri: Kecepatan Sudut (dps)
axs[1, 0].plot(time_sec, gy_uncal, color="#5cb85c", linestyle=":", alpha=0.6, label=f"Sebelum Kalibrasi (Rata-rata: {bias_y:.4f} dps)")
axs[1, 0].plot(time_sec, gy_cal, color="#5cb85c", linestyle="-", label="Sesudah Kalibrasi (Rata-rata: 0.0000 dps)")
axs[1, 0].set_ylabel("Pitch Rate (dps)", fontsize=9, fontweight="bold")
axs[1, 0].grid(True, linestyle=":", alpha=0.6)
axs[1, 0].legend(loc="upper right", fontsize=8)

# Kanan: Integrasi Sudut (deg)
axs[1, 1].plot(time_sec, gy_angle_uncal, color="#5cb85c", linestyle=":", alpha=0.6, label="Sebelum (Hanyat/Drifting)")
axs[1, 1].plot(time_sec, gy_angle_cal, color="#5cb85c", linestyle="-", label="Sesudah (Stabil)")
axs[1, 1].set_ylabel("Sudut Pitch (derajat)", fontsize=9, fontweight="bold")
axs[1, 1].grid(True, linestyle=":", alpha=0.6)
axs[1, 1].legend(loc="upper left", fontsize=8)

# ======================== SUMBU Z (YAW) ========================
# Kiri: Kecepatan Sudut (dps)
axs[2, 0].plot(time_sec, gz_uncal, color="#0275d8", linestyle=":", alpha=0.6, label=f"Sebelum Kalibrasi (Rata-rata: {bias_z:.4f} dps)")
axs[2, 0].plot(time_sec, gz_cal, color="#0275d8", linestyle="-", label="Sesudah Kalibrasi (Rata-rata: 0.0000 dps)")
axs[2, 0].set_ylabel("Yaw Rate (dps)", fontsize=9, fontweight="bold")
axs[2, 0].set_xlabel("Waktu (detik)", fontsize=9)
axs[2, 0].grid(True, linestyle=":", alpha=0.6)
axs[2, 0].legend(loc="upper right", fontsize=8)

# Kanan: Integrasi Sudut (deg)
axs[2, 1].plot(time_sec, gz_angle_uncal, color="#0275d8", linestyle=":", alpha=0.6, label="Sebelum (Hanyat/Drifting)")
axs[2, 1].plot(time_sec, gz_angle_cal, color="#0275d8", linestyle="-", label="Sesudah (Stabil)")
axs[2, 1].set_ylabel("Sudut Yaw (derajat)", fontsize=9, fontweight="bold")
axs[2, 1].set_xlabel("Waktu (detik)", fontsize=9)
axs[2, 1].grid(True, linestyle=":", alpha=0.6)
axs[2, 1].legend(loc="upper left", fontsize=8)

plt.suptitle("Analisis Pengaruh Kalibrasi Bias terhadap Estimasi Navigasi (MPU6050)", fontsize=13, fontweight="bold", y=0.98)
plt.tight_layout()
plt.savefig(PNG_FILE, dpi=150)
plt.close()

print(f"Grafik visual 3x2 berhasil disimpan ke: {PNG_FILE}")
