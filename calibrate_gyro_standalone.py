#!/usr/bin/env python3
"""
Standalone Calibration Script for MPU6050 Gyroscope Bias with CSV Logging.
Logs raw, uncalibrated, and calibrated values to demonstrate calibration effect.
"""

import time
import sys
import os
import csv

try:
    from smbus2 import SMBus
except ImportError:
    print("Error: Pustaka 'smbus2' tidak ditemukan. Pasang terlebih dahulu dengan: pip install smbus2")
    sys.exit(1)

# Konstanta MPU6050
IMU_I2C_BUS = 1
IMU_I2C_ADDR = 0x68
IMU_PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
GYRO_LSB_PER_DPS = 131.0  # LSB per derajat/detik

# Path Output Log
OUTPUT_DIR = "hasil_dan_pembahasan"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "gyro_calibration_samples.csv")

def read_i2c_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value

def main():
    print("=================================================================")
    print("      PROSES KALIBRASI BIAS GIROSKOP MPU6050 (STANDALONE)        ")
    print("=================================================================")
    print("PENTING: Letakkan sensor/drone diam sempurna di atas meja datar.")
    print("Jangan menggerakkan sensor selama proses kalibrasi berlangsung.")
    print("Menghubungkan ke I2C bus 1...")
    
    try:
        bus = SMBus(IMU_I2C_BUS)
        bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
        time.sleep(0.2)
        print("Koneksi berhasil! MPU6050 terbangun.")
    except Exception as e:
        print(f"Gagal menghubungkan ke sensor: {e}")
        sys.exit(1)
        
    sample_count = 1000
    dt = 0.01  # Interval 10 ms
    gx_total = 0.0
    gy_total = 0.0
    gz_total = 0.0
    
    # Pastikan direktori output ada
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    samples_data = []
    print(f"\nMemulai pengambilan {sample_count} sampel data (durasi ~2 detik)...")
    
    for i in range(sample_count):
        try:
            gx_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H)
            gy_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H)
            gz_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H)
            
            # Konversi uncalibrated dps
            gx_uncal = gx_raw / GYRO_LSB_PER_DPS
            gy_uncal = gy_raw / GYRO_LSB_PER_DPS
            gz_uncal = gz_raw / GYRO_LSB_PER_DPS
            
            gx_total += gx_uncal
            gy_total += gy_uncal
            gz_total += gz_uncal
            
            samples_data.append({
                "sample_index": i + 1,
                "gx_raw": gx_raw,
                "gy_raw": gy_raw,
                "gz_raw": gz_raw,
                "gx_uncal": gx_uncal,
                "gy_uncal": gy_uncal,
                "gz_uncal": gz_uncal
            })
            
            if (i + 1) % 20 == 0:
                print(f"Progress: {i + 1}/{sample_count} sampel terkumpul...")
                
            time.sleep(dt)
            
        except Exception as e:
            print(f"\nError saat membaca sensor pada sampel ke-{i+1}: {e}")
            sys.exit(1)
            
    # Hitung rata-rata bias
    bias_x = gx_total / sample_count
    bias_y = gy_total / sample_count
    bias_z = gz_total / sample_count
    
    print("\n========================= HASIL KALIBRASI =======================")
    print(f"Bias Sumbu-X (dps): {bias_x:+.4f} dps")
    print(f"Bias Sumbu-Y (dps): {bias_y:+.4f} dps")
    print(f"Bias Sumbu-Z (dps): {bias_z:+.4f} dps")
    print("=================================================================")
    
    # Hitung data terkalibrasi dan integrasi
    x_int_uncal, y_int_uncal, z_int_uncal = 0.0, 0.0, 0.0
    x_int_cal, y_int_cal, z_int_cal = 0.0, 0.0, 0.0
    
    for row in samples_data:
        # Terapkan koreksi bias
        row["gx_cal"] = row["gx_uncal"] - bias_x
        row["gy_cal"] = row["gy_uncal"] - bias_y
        row["gz_cal"] = row["gz_uncal"] - bias_z
        
        # Hitung integrasi sudut (kecepatan * dt)
        x_int_uncal += row["gx_uncal"] * dt
        y_int_uncal += row["gy_uncal"] * dt
        z_int_uncal += row["gz_uncal"] * dt
        
        x_int_cal += row["gx_cal"] * dt
        y_int_cal += row["gy_cal"] * dt
        z_int_cal += row["gz_cal"] * dt
        
        row["x_angle_uncal"] = x_int_uncal
        row["y_angle_uncal"] = y_int_uncal
        row["z_angle_uncal"] = z_int_uncal
        
        row["x_angle_cal"] = x_int_cal
        row["y_angle_cal"] = y_int_cal
        row["z_angle_cal"] = z_int_cal

    # Simpan sampel ke berkas CSV
    print(f"Menulis data sampel ke: {OUTPUT_FILE} ...")
    try:
        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Sample Index", 
                "GX Raw (LSB)", "GY Raw (LSB)", "GZ Raw (LSB)", 
                "GX Uncal (dps)", "GY Uncal (dps)", "GZ Uncal (dps)",
                "GX Cal (dps)", "GY Cal (dps)", "GZ Cal (dps)",
                "Roll Angle Uncal (deg)", "Pitch Angle Uncal (deg)", "Yaw Angle Uncal (deg)",
                "Roll Angle Cal (deg)", "Pitch Angle Cal (deg)", "Yaw Angle Cal (deg)"
            ])
            for r in samples_data:
                writer.writerow([
                    r["sample_index"],
                    r["gx_raw"], r["gy_raw"], r["gz_raw"],
                    f"{r['gx_uncal']:.6f}", f"{r['gy_uncal']:.6f}", f"{r['gz_uncal']:.6f}",
                    f"{r['gx_cal']:.6f}", f"{r['gy_cal']:.6f}", f"{r['gz_cal']:.6f}",
                    f"{r['x_angle_uncal']:.6f}", f"{r['y_angle_uncal']:.6f}", f"{r['z_angle_uncal']:.6f}",
                    f"{r['x_angle_cal']:.6f}", f"{r['y_angle_cal']:.6f}", f"{r['z_angle_cal']:.6f}"
                ])
        print("Penulisan CSV berhasil!")
    except Exception as e:
        print(f"Gagal menulis file CSV: {e}")
        
    print("\nFormat penulisan untuk konfigurasi Python:")
    print(f"gyro_bias = {{\n    'x': {bias_x:.6f},\n    'y': {bias_y:.6f},\n    'z': {bias_z:.6f}\n}}")
    print("=================================================================")

if __name__ == "__main__":
    main()
