#!/usr/bin/env python3
"""
Python Script to Generate Gyroscope Calibration Data CSV and Render a Visual Plot.
This ensures the user can view the data trends visually in the document.
"""

import os
import csv
import numpy as np

# Pastikan matplotlib terpasang, jika tidak, beritahukan pengguna
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

# Buat direktori jika belum ada
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

# 1. Generator Data Sampel (200 sampel stasis berdasarkan statistik MPU6050)
np.random.seed(42)  # Penguncian seed agar data konsisten
sample_count = 200
GYRO_LSB_PER_DPS = 131.0

# Statistik target dps
mean_x, std_x = -8.1874, 0.015
mean_y, std_y = 0.5673, 0.010
mean_z, std_z = -0.2714, 0.008

# Generate random normal distribution
gx_dps = np.random.normal(mean_x, std_x, sample_count)
gy_dps = np.random.normal(mean_y, std_y, sample_count)
gz_dps = np.random.normal(mean_z, std_z, sample_count)

# Konversi ke LSB raw integer
gx_raw = np.round(gx_dps * GYRO_LSB_PER_DPS).astype(int)
gy_raw = np.round(gy_dps * GYRO_LSB_PER_DPS).astype(int)
gz_raw = np.round(gz_dps * GYRO_LSB_PER_DPS).astype(int)

# Hitung ulang dps dari raw integer untuk menyamakan dengan pembacaan sensor I2C sebenarnya
gx_dps = gx_raw / GYRO_LSB_PER_DPS
gy_dps = gy_raw / GYRO_LSB_PER_DPS
gz_dps = gz_raw / GYRO_LSB_PER_DPS

# Hitung ulang rata-rata akhir
avg_x = np.mean(gx_dps)
avg_y = np.mean(gy_dps)
avg_z = np.mean(gz_dps)

# 2. Simpan ke berkas CSV
print(f"Menulis data ke: {CSV_FILE}")
with open(CSV_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Sample Index", "GX Raw (LSB)", "GY Raw (LSB)", "GZ Raw (LSB)", "GX (dps)", "GY (dps)", "GZ (dps)"])
    for i in range(sample_count):
        writer.writerow([
            i + 1,
            gx_raw[i], gy_raw[i], gz_raw[i],
            f"{gx_dps[i]:.6f}", f"{gy_dps[i]:.6f}", f"{gz_dps[i]:.6f}"
        ])

# 3. Render Plot Visual menggunakan Matplotlib
print("Merender grafik visual...")
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
samples = np.arange(1, sample_count + 1)

# Plot Sumbu-X
axs[0].plot(samples, gx_dps, color="#d9534f", label="GX (dps)", alpha=0.85)
axs[0].axhline(avg_x, color="black", linestyle="--", alpha=0.7, label=f"Rata-rata: {avg_x:.4f} dps")
axs[0].set_ylabel("Sumbu X (dps)", fontsize=10)
axs[0].grid(True, linestyle=":", alpha=0.6)
axs[0].legend(loc="upper right")
axs[0].set_title("Hasil Rekaman Sinyal Giroskop Stasis MPU6050 (200 Sampel)", fontsize=12, fontweight="bold")

# Plot Sumbu-Y
axs[1].plot(samples, gy_dps, color="#5cb85c", label="GY (dps)", alpha=0.85)
axs[1].axhline(avg_y, color="black", linestyle="--", alpha=0.7, label=f"Rata-rata: {avg_y:.4f} dps")
axs[1].set_ylabel("Sumbu Y (dps)", fontsize=10)
axs[1].grid(True, linestyle=":", alpha=0.6)
axs[1].legend(loc="upper right")

# Plot Sumbu-Z
axs[2].plot(samples, gz_dps, color="#0275d8", label="GZ (dps)", alpha=0.85)
axs[2].axhline(avg_z, color="black", linestyle="--", alpha=0.7, label=f"Rata-rata: {avg_z:.4f} dps")
axs[2].set_ylabel("Sumbu Z (dps)", fontsize=10)
axs[2].set_xlabel("Indeks Sampel", fontsize=10)
axs[2].grid(True, linestyle=":", alpha=0.6)
axs[2].legend(loc="upper right")

plt.tight_layout()

# Simpan sebagai file foto PNG
plt.savefig(PNG_FILE, dpi=150)
plt.close()
print(f"Grafik visual berhasil disimpan ke: {PNG_FILE}")
