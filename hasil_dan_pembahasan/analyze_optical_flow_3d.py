#!/usr/bin/env python3
"""
Python Script to Analyze and Plot 3D Trajectory from Optical Flow SHM Log.
Plots X (m), Y (m), and Altitude (m) with a 2D floor projection shadow.
"""

import os
import pandas as pd
import numpy as np

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    import subprocess
    import sys
    print("Pustaka 'matplotlib' tidak ditemukan. Mencoba memasang...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

# Konfigurasi Path
INPUT_DIR = "hasil_dan_pembahasan"
CSV_FILE = os.path.join(INPUT_DIR, "optical_flow_shm_log.csv")
PNG_FILE = os.path.join(INPUT_DIR, "optical_flow_3d_trajectory.png")

def main():
    print("=================================================================")
    print("      ANALISIS TRAJEKTORI 3D ALIRAN OPTIK UNTUK LAMPIRAN SKRIPSI ")
    print("=================================================================")
    
    if not os.path.exists(CSV_FILE):
        print(f"Error: Berkas log {CSV_FILE} tidak ditemukan.")
        print("Pastikan Anda sudah menjalankan 'log_optical_flow_shm.py' terlebih dahulu.")
        return

    # 1. Memuat Data
    print(f"Membaca berkas log: {CSV_FILE} ...")
    df = pd.read_csv(CSV_FILE)
    
    # Ambil data kolom dan konversi X, Y dari cm ke meter agar seragam dengan Altitude (m)
    x_m = df["X Position (cm)"].values / 100.0
    y_m = df["Y Position (cm)"].values / 100.0
    z_m = df["Altitude (m)"].values
    
    vx = df["VX (m/s)"].values
    vy = df["VY (m/s)"].values
    time_s = df["Timestamp (s)"].values
    duration = time_s[-1] - time_s[0]
    
    # 2. Kalkulasi Parameter Trajektori
    # Hitung kecepatan translasi 2D
    v_mag = np.sqrt(vx**2 + vy**2)
    max_v = np.max(v_mag)
    avg_v = np.mean(v_mag)
    
    # Hitung panjang lintasan total dalam 3D (arc length)
    dx = np.diff(x_m)
    dy = np.diff(y_m)
    dz = np.diff(z_m)
    step_distances = np.sqrt(dx**2 + dy**2 + dz**2)
    total_distance = np.sum(step_distances)
    
    # Statistik Ketinggian
    max_alt = np.max(z_m)
    min_alt = np.min(z_m)
    avg_alt = np.mean(z_m)
    
    print("\n---------------------- LAPORAN METRIK PENERBANGAN ----------------------")
    print(f"Durasi Log Penerbangan    : {duration:.2f} detik")
    print(f"Jumlah Titik Sampel       : {len(df)} titik")
    print(f"Panjang Lintasan Total 3D : {total_distance:.4f} meter")
    print("")
    print("A. Statistik Kecepatan Linier:")
    print(f"   - Kecepatan Maksimum   : {max_v:.4f} m/s")
    print(f"   - Kecepatan Rata-rata  : {avg_v:.4f} m/s")
    print("")
    print("B. Statistik Ketinggian (LiDAR Altitude):")
    print(f"   - Ketinggian Maksimum  : {max_alt:.4f} meter")
    print(f"   - Ketinggian Minimum   : {min_alt:.4f} meter")
    print(f"   - Ketinggian Rata-rata : {avg_alt:.4f} meter")
    print("")
    print("C. Cakupan Area Terbang (Bounding Box):")
    print(f"   - Rentang Sumbu-X      : {np.min(x_m):+.2f} m s.d. {np.max(x_m):+.2f} m")
    print(f"   - Rentang Sumbu-Y      : {np.min(y_m):+.2f} m s.d. {np.max(y_m):+.2f} m")
    print("------------------------------------------------------------------------")
    
    # 3. Pembuatan Plot 3D
    print("\nMerender grafik lintasan 3D...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot lintasan penerbangan utama dalam 3D
    # Menggunakan colormap untuk menunjukkan variasi kecepatan sepanjang lintasan
    sc = ax.scatter(x_m, y_m, z_m, c=v_mag, cmap='viridis', s=10, label='Posisi Drone (Warna: Kecepatan)')
    ax.plot(x_m, y_m, z_m, color='#555555', alpha=0.5, linewidth=1.5, label='Garis Lintasan 3D')
    
    # Tambahkan proyeksi bayangan lintasan pada lantai (Z = min_alt) untuk membantu visualisasi 3D
    z_floor = np.min(z_m) - 0.05
    ax.plot(x_m, y_m, np.full_like(z_m, z_floor), color='#d3d3d3', linestyle='--', alpha=0.7, label='Proyeksi Lintasan pada Lantai')
    
    # Pengaturan Label Sumbu
    ax.set_xlabel('Posisi X (meter)', fontsize=10, fontweight='bold', labelpad=8)
    ax.set_ylabel('Posisi Y (meter)', fontsize=10, fontweight="bold", labelpad=8)
    ax.set_zlabel('Ketinggian Z (meter)', fontsize=10, fontweight="bold", labelpad=8)
    
    # Konfigurasi Tampilan Grid & Box
    ax.grid(True, linestyle=":", alpha=0.5)
    
    # Tambahkan colorbar untuk informasi kecepatan
    cbar = fig.colorbar(sc, ax=ax, shrink=0.5, aspect=10, pad=0.1)
    cbar.set_label('Kecepatan Linier (m/s)', fontsize=9, fontweight='bold')
    
    # Berikan titik start (hijau) dan stop (merah)
    ax.scatter(x_m[0], y_m[0], z_m[0], color='green', marker='^', s=100, label='Titik Mulai (Start)')
    ax.scatter(x_m[-1], y_m[-1], z_m[-1], color='red', marker='v', s=100, label='Titik Akhir (Stop)')
    
    ax.legend(loc='upper right', fontsize=8)
    plt.title("Visualisasi Trajektori Penerbangan 3D Estimasi Aliran Optik", fontsize=12, fontweight='bold', y=0.95)
    
    # Atur sudut kamera awal yang ideal untuk melihat 3D
    ax.view_init(elev=25, azim=-45)
    
    plt.tight_layout()
    plt.savefig(PNG_FILE, dpi=300)
    plt.close()
    
    print(f"Visualisasi trajektori 3D berhasil disimpan di: {PNG_FILE}")

if __name__ == "__main__":
    main()
