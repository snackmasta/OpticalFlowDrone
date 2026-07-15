#!/usr/bin/env python3
"""
Standalone Calibration Script for MPU6050 Gyroscope Bias with CSV Logging.
This script reads raw data from MPU6050, averages 200 samples, computes the bias offsets,
and logs all individual samples to a CSV file.
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
GYRO_LSB_PER_DPS = 131.0  # LSB per derajat/detik untuk skala +/- 250 dps

# Path Output Log
OUTPUT_DIR = "hasil_dan_pembahasan"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "gyro_calibration_samples.csv")

def read_i2c_word(bus, addr, reg):
    """Membaca 2 byte register I2C dan mengonversinya menjadi integer 16-bit bertanda."""
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
        # Bangunkan MPU6050 (menulis 0 ke PWR_MGMT_1)
        bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
        time.sleep(0.2)
        print("Koneksi berhasil! MPU6050 terbangun.")
    except Exception as e:
        print(f"Gagal menghubungkan ke sensor: {e}")
        print("Pastikan koneksi kabel I2C (SDA, SCL) dan alamat 0x68 sudah benar.")
        sys.exit(1)
        
    sample_count = 200
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
            
            # Konversi langsung ke dps (derajat per detik)
            gx_dps = gx_raw / GYRO_LSB_PER_DPS
            gy_dps = gy_raw / GYRO_LSB_PER_DPS
            gz_dps = gz_raw / GYRO_LSB_PER_DPS
            
            gx_total += gx_dps
            gy_total += gy_dps
            gz_total += gz_dps
            
            # Simpan data sampel
            samples_data.append({
                "sample_index": i + 1,
                "gx_raw": gx_raw,
                "gy_raw": gy_raw,
                "gz_raw": gz_raw,
                "gx_dps": gx_dps,
                "gy_dps": gy_dps,
                "gz_dps": gz_dps
            })
            
            # Print progres secara interaktif
            if (i + 1) % 20 == 0:
                print(f"Progress: {i + 1}/{sample_count} sampel terkumpul...")
                
            time.sleep(0.01)  # Jeda 10 ms antar-sampel
            
        except Exception as e:
            print(f"\nError saat membaca sensor pada sampel ke-{i+1}: {e}")
            sys.exit(1)
            
    # Hitung rata-rata bias
    bias_x = gx_total / sample_count
    bias_y = gy_total / sample_count
    bias_z = gz_total / sample_count
    
    print("\n========================= HASIL KALIBRASI =======================")
    print(f"Sampel Terproses  : {sample_count}")
    print(f"Bias Sumbu-X (dps): {bias_x:+.4f} dps")
    print(f"Bias Sumbu-Y (dps): {bias_y:+.4f} dps")
    print(f"Bias Sumbu-Z (dps): {bias_z:+.4f} dps")
    print("=================================================================")
    
    # Simpan sampel ke berkas CSV
    print(f"Menulis data sampel ke: {OUTPUT_FILE} ...")
    try:
        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            # Tulis header
            writer.writerow([
                "Sample Index", 
                "GX Raw (LSB)", "GY Raw (LSB)", "GZ Raw (LSB)", 
                "GX (dps)", "GY (dps)", "GZ (dps)"
            ])
            # Tulis baris data
            for row in samples_data:
                writer.writerow([
                    row["sample_index"],
                    row["gx_raw"], row["gy_raw"], row["gz_raw"],
                    f"{row['gx_dps']:.6f}", f"{row['gy_dps']:.6f}", f"{row['gz_dps']:.6f}"
                ])
        print("Penulisan CSV berhasil!")
    except Exception as e:
        print(f"Gagal menulis file CSV: {e}")
        
    print("\nFormat penulisan untuk konfigurasi Python:")
    print(f"gyro_bias = {{\n    'x': {bias_x:.6f},\n    'y': {bias_y:.6f},\n    'z': {bias_z:.6f}\n}}")
    print("=================================================================")

if __name__ == "__main__":
    main()
