#!/usr/bin/env python3
"""
Python Script to Analyze Sensor Data for Thesis (Skripsi).
Calculates RMSE, Drift Rates, and generates roll/pitch comparison plots.
"""

import os
import pandas as pd
import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    import subprocess
    import sys
    print("Pustaka 'matplotlib' tidak ditemukan. Mencoba memasang...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
    import matplotlib.pyplot as plt

# Konfigurasi Path
INPUT_DIR = "hasil_dan_pembahasan"
CSV_FILE = os.path.join(INPUT_DIR, "sensor_comparison_log.csv")
PLOT_ROLL_FILE = os.path.join(INPUT_DIR, "roll_comparison_plot.png")
PLOT_PITCH_FILE = os.path.join(INPUT_DIR, "pitch_comparison_plot.png")

def main():
    print("=================================================================")
    print("       ANALISIS DATA SENSOR FUSI IMU UNTUK LAMPIRAN SKRIPSI      ")
    print("=================================================================")

    if not os.path.exists(CSV_FILE):
        print(f"Error: Berkas data {CSV_FILE} tidak ditemukan. Harap jalankan script pengumpul data terlebih dahulu.")
        return

    # 1. Memuat Data Log CSV
    print(f"Membaca berkas log: {CSV_FILE} ...")
    df = pd.read_csv(CSV_FILE)
    
    time = df["Time (s)"].values
    roll_gyro = df["Roll Gyro Pure (deg)"].values
    roll_accel = df["Roll Accel (deg)"].values
    roll_cf = df["Roll CF (deg)"].values

    pitch_gyro = df["Pitch Gyro Pure (deg)"].values
    pitch_accel = df["Pitch Accel (deg)"].values
    pitch_cf = df["Pitch CF (deg)"].values

    # 2. Perhitungan Statistik Evaluasi
    # Hitung Root Mean Square Error (RMSE) antara filter komplementer dan akselerometer
    rmse_roll = np.sqrt(np.mean((roll_cf - roll_accel) ** 2))
    rmse_pitch = np.sqrt(np.mean((pitch_cf - pitch_accel) ** 2))

    # Hitung Hanyatan Akumulatif (Drift Rate) dari Giroskop Murni dibanding Filter Komplementer
    duration = time[-1] - time[0]
    drift_roll_total = roll_gyro[-1] - roll_cf[-1]
    drift_pitch_total = pitch_gyro[-1] - pitch_cf[-1]
    
    drift_rate_roll = abs(drift_roll_total) / duration
    drift_rate_pitch = abs(drift_pitch_total) / duration

    # 3. Menampilkan Laporan Statistik dalam Format Akademik Skripsi
    print("\n---------------------- HASIL ANALISIS STATISTIKA ----------------------")
    print(f"Durasi Data Pengujian      : {duration:.2f} detik")
    print(f"Jumlah Sampel Teranalisis  : {len(df)} sampel")
    print("")
    print("A. Evaluasi Akurasi Sudut (Fusi Filter vs Akselerometer):")
    print(f"   - RMSE Sudut Roll       : {rmse_roll:.4f}°")
    print(f"   - RMSE Sudut Pitch      : {rmse_pitch:.4f}°")
    print("   (Catatan: RMSE rendah menunjukkan filter mampu mengikuti tren dinamis akselerometer)")
    print("")
    print("B. Analisis Galat Hanyatan Giroskop Murni (Gyro Drift Analysis):")
    print(f"   - Total Drift Roll      : {drift_roll_total:+.4f}° dalam {duration:.1f} detik")
    print(f"   - Laju Drift Roll       : {drift_rate_roll:.4f}°/detik")
    print(f"   - Total Drift Pitch     : {drift_pitch_total:+.4f}° dalam {duration:.1f} detik")
    print(f"   - Laju Drift Pitch      : {drift_rate_pitch:.4f}°/detik")
    print("   (Catatan: Laju drift menunjukkan seberapa cepat galat akumulasi membesar tanpa filter)")
    print("-----------------------------------------------------------------------")

    # 4. Pembuatan Plot Perbandingan Sudut Roll (Skripsi Style)
    print("\nMerender plot perbandingan sudut Roll...")
    plt.figure(figsize=(10, 5))
    plt.plot(time, roll_gyro, color="#d9534f", linestyle="-.", label="Integrasi Giroskop Murni (Drifting)", alpha=0.8)
    plt.plot(time, roll_accel, color="#f0ad4e", linestyle=":", label="Akselerometer Mentah (Noise Tinggi)", alpha=0.5)
    plt.plot(time, roll_cf, color="#0275d8", linestyle="-", linewidth=2, label="Filter Komplementer (Fused & Stable)")
    plt.title("Perbandingan Estimasi Sudut Roll (Φ) terhadap Waktu", fontsize=12, fontweight="bold")
    plt.xlabel("Waktu (detik)", fontsize=10)
    plt.ylabel("Sudut Roll (derajat)", fontsize=10)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig(PLOT_ROLL_FILE, dpi=300)  # Resolusi tinggi 300 dpi untuk cetak skripsi
    plt.close()
    print(f"Plot Roll disimpan di: {PLOT_ROLL_FILE}")

    # 5. Pembuatan Plot Perbandingan Sudut Pitch (Skripsi Style)
    print("Merender plot perbandingan sudut Pitch...")
    plt.figure(figsize=(10, 5))
    plt.plot(time, pitch_gyro, color="#d9534f", linestyle="-.", label="Integrasi Giroskop Murni (Drifting)", alpha=0.8)
    plt.plot(time, pitch_accel, color="#f0ad4e", linestyle=":", label="Akselerometer Mentah (Noise Tinggi)", alpha=0.5)
    plt.plot(time, pitch_cf, color="#0275d8", linestyle="-", linewidth=2, label="Filter Komplementer (Fused & Stable)")
    plt.title("Perbandingan Estimasi Sudut Pitch (θ) terhadap Waktu", fontsize=12, fontweight="bold")
    plt.xlabel("Waktu (detik)", fontsize=10)
    plt.ylabel("Sudut Pitch (derajat)", fontsize=10)
    plt.grid(True, linestyle=":", alpha=0.6)
    plt.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig(PLOT_PITCH_FILE, dpi=300)
    plt.close()
    print(f"Plot Pitch disimpan di: {PLOT_PITCH_FILE}")
    print("\nProses selesai! Seluruh grafik dan data siap disematkan pada laporan skripsi.")

if __name__ == "__main__":
    main()
