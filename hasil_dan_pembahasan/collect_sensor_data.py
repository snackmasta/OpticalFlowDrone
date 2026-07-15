#!/usr/bin/env python3
"""
Python Script to Collect and Compare Raw Sensor Data, Pure Gyro Integration,
and Complementary Filter Outputs. Logs to CSV for evaluation.
"""

import time
import math
import struct
import csv
import os
import sys
from multiprocessing import shared_memory

# Buat pustaka smbus2 opsional agar dapat diuji di PC non-Pi
try:
    from smbus2 import SMBus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

# Konstanta Sensor
IMU_I2C_BUS = 1
IMU_I2C_ADDR = 0x68
IMU_PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_LSB_PER_G = 16384.0
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
GYRO_LSB_PER_DPS = 131.0
COMPLEMENTARY_FILTER_ALPHA = 0.96

# Nilai Bias Terkalibrasi (dari evaluasi)
GYRO_BIAS_X = -8.1874
GYRO_BIAS_Y = 0.5673
GYRO_BIAS_Z = -0.2714

# Shared Memory Kompas
COMPASS_SHM_NAME = "compass_heading_stream"
COMPASS_SHM_MAGIC = b"CHDG"
COMPASS_SHM_HEADER_FORMAT = "<4sII"
COMPASS_SHM_RECORD_FORMAT = "<6d"
COMPASS_SHM_HEADER_SIZE = struct.calcsize(COMPASS_SHM_HEADER_FORMAT)
COMPASS_SHM_RECORD_SIZE = struct.calcsize(COMPASS_SHM_RECORD_FORMAT)
COMPASS_MAX_SAMPLES = 120

# Output Path
OUTPUT_DIR = "hasil_dan_pembahasan"
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "sensor_comparison_log.csv")

def read_i2c_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value

def get_compass_heading(compass_shm):
    if compass_shm is None:
        return 0.0
    try:
        magic, write_index, sample_count = struct.unpack_from(COMPASS_SHM_HEADER_FORMAT, compass_shm.buf, 0)
        if magic != COMPASS_SHM_MAGIC or sample_count == 0:
            return 0.0
        latest_index = (write_index - 1) % COMPASS_MAX_SAMPLES
        record_offset = COMPASS_SHM_HEADER_SIZE + (latest_index * COMPASS_SHM_RECORD_SIZE)
        # Record: timestamp, raw_heading, heading, x, y, z
        _, _, heading, _, _, _ = struct.unpack_from(COMPASS_SHM_RECORD_FORMAT, compass_shm.buf, record_offset)
        return heading
    except Exception:
        return 0.0

def main():
    print("=================================================================")
    print("         DATA COLLECTION SCRIPT FOR SENSOR FUSION EVALUATION      ")
    print("=================================================================")
    
    bus = None
    if SMBUS_AVAILABLE:
        print("Menghubungkan ke sensor IMU via I2C...")
        try:
            bus = SMBus(IMU_I2C_BUS)
            bus.write_byte_data(IMU_I2C_ADDR, IMU_PWR_MGMT_1, 0)
            time.sleep(0.2)
            print("Koneksi MPU6050 Berhasil.")
        except Exception as e:
            print(f"Gagal koneksi I2C IMU: {e}. Mengaktifkan mode simulasi.")
            bus = None
    else:
        print("Pustaka 'smbus2' tidak tersedia. Mengaktifkan mode simulasi gerakan.")

    # Hubungkan ke Shared Memory Kompas HMC5883L
    compass_shm = None
    try:
        compass_shm = shared_memory.SharedMemory(name=COMPASS_SHM_NAME)
        print("Berhasil terhubung ke Shared Memory Kompas.")
    except FileNotFoundError:
        print("Peringatan: Shared memory kompas tidak aktif. Menggunakan nilai fallback 0.0.")

    # Inisialisasi variabel integrasi
    roll_gyro_pure = 0.0
    pitch_gyro_pure = 0.0
    
    roll_cf = 0.0
    pitch_cf = 0.0
    
    log_data = []
    duration = 10.0  # Durasi pengumpulan data: 10 detik
    rate_hz = 50     # Frekuensi pengambilan data: 50 Hz
    dt = 1.0 / rate_hz
    
    print(f"\nMemulai pengumpulan data selama {duration} detik pada rate {rate_hz} Hz...")
    if bus is None:
        print("STATUS: Menjalankan simulasi gerakan harmonik (Roll/Pitch sinewave)...")
    else:
        print("Silakan gerakkan drone (roll dan pitch) untuk melihat perbedaan integrasinya.")
    
    start_time = time.time()
    
    try:
        while (time.time() - start_time) < duration:
            loop_start = time.time()
            now = loop_start - start_time
            
            # 1. Baca data sensor (Gunakan simulasi jika I2C tidak terdeteksi)
            if bus is not None:
                try:
                    ax_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_XOUT_H)
                    ay_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_XOUT_H + 2)
                    az_raw = read_i2c_word(bus, IMU_I2C_ADDR, ACCEL_XOUT_H + 4)
                    gx_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_XOUT_H)
                    gy_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_YOUT_H)
                    gz_raw = read_i2c_word(bus, IMU_I2C_ADDR, GYRO_ZOUT_H)
                    
                    ax_g = ax_raw / ACCEL_LSB_PER_G
                    ay_g = ay_raw / ACCEL_LSB_PER_G
                    az_g = az_raw / ACCEL_LSB_PER_G
                    gx_dps = (gx_raw / GYRO_LSB_PER_DPS) - GYRO_BIAS_X
                    gy_dps = (gy_raw / GYRO_LSB_PER_DPS) - GYRO_BIAS_Y
                    gz_dps = (gz_raw / GYRO_LSB_PER_DPS) - GYRO_BIAS_Z
                except Exception as e:
                    print(f"Gagal membaca sensor: {e}")
                    break
            else:
                # Mode Simulasi Gerak Harmonik Sederhana (Untuk PC Pengembang)
                t_val = now * 2.0 * math.pi * 0.5  # 0.5 Hz sinewave
                ax_g = 0.1 * math.sin(t_val)
                ay_g = 0.15 * math.cos(t_val)
                az_g = 0.98  # Gaya gravitasi bumi
                gx_dps = 15.0 * math.cos(t_val)  # Kecepatan sudut X
                gy_dps = -10.0 * math.sin(t_val) # Kecepatan sudut Y
                gz_dps = 0.0

            # 2. Hitung Sudut dari Akselerometer
            roll_accel = math.degrees(math.atan2(ay_g, az_g))
            pitch_accel = math.degrees(math.atan2(-ax_g, math.sqrt((ay_g * ay_g) + (az_g * az_g))))

            # 3. Integrasi Murni Giroskop (Tanpa Filter)
            roll_gyro_pure += gx_dps * dt
            pitch_gyro_pure += gy_dps * dt

            # 4. Filter Komplementer (Siklus Update Fusi)
            roll_cf = (COMPLEMENTARY_FILTER_ALPHA * (roll_cf + gx_dps * dt)) + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel)
            pitch_cf = (COMPLEMENTARY_FILTER_ALPHA * (pitch_cf + gy_dps * dt)) + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel)

            # 5. Dapatkan Arah Hadap Kompas dari SHM
            heading = get_compass_heading(compass_shm)

            # Simpan data ke memori
            log_data.append([
                f"{now:.3f}",
                f"{ax_g:.4f}", f"{ay_g:.4f}", f"{az_g:.4f}",
                f"{gx_dps:.4f}", f"{gy_dps:.4f}", f"{gz_dps:.4f}",
                f"{roll_gyro_pure:.4f}", f"{pitch_gyro_pure:.4f}",
                f"{roll_accel:.4f}", f"{pitch_accel:.4f}",
                f"{roll_cf:.4f}", f"{pitch_cf:.4f}",
                f"{heading:.2f}"
            ])
            
            # Tunggu hingga siklus 50Hz terpenuhi
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nPengumpulan data dihentikan secara manual.")

    # 6. Tulis ke CSV
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    print(f"\nMenulis data log ke CSV: {OUTPUT_CSV} ...")
    try:
        with open(OUTPUT_CSV, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Time (s)",
                "AX (g)", "AY (g)", "AZ (g)",
                "GX (dps)", "GY (dps)", "GZ (dps)",
                "Roll Gyro Pure (deg)", "Pitch Gyro Pure (deg)",
                "Roll Accel (deg)", "Pitch Accel (deg)",
                "Roll CF (deg)", "Pitch CF (deg)",
                "Compass Heading (deg)"
            ])
            writer.writerows(log_data)
        print("Berhasil menyimpan log perbandingan sensor!")
    except Exception as e:
        print(f"Gagal menulis CSV: {e}")

    if compass_shm is not None:
        try:
            compass_shm.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
