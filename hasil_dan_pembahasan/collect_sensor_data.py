#!/usr/bin/env python3
"""
Python Script to Collect and Compare Raw Sensor Data, Pure Gyro Integration,
and Complementary Filter Outputs. Outputs all Yaw angles in [0, 360] range
to match the GPS NMEA injection format.
"""

import time
import math
import struct
import csv
import os
import sys
import random
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
        return None
    try:
        magic, write_index, sample_count = struct.unpack_from(COMPASS_SHM_HEADER_FORMAT, compass_shm.buf, 0)
        if magic != COMPASS_SHM_MAGIC or sample_count == 0:
            return None
        latest_index = (write_index - 1) % COMPASS_MAX_SAMPLES
        record_offset = COMPASS_SHM_HEADER_SIZE + (latest_index * COMPASS_SHM_RECORD_SIZE)
        # Record: timestamp, raw_heading, heading, x, y, z
        _, _, heading, _, _, _ = struct.unpack_from(COMPASS_SHM_RECORD_FORMAT, compass_shm.buf, record_offset)
        return heading
    except Exception:
        return None

def normalize_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0

def invert_compass_heading_deg(heading_deg):
    """Inversi sudut kompas sesuai dengan sensor_readers.py."""
    return normalize_angle_deg(-heading_deg)

def angular_error_deg(target_deg, current_deg):
    return normalize_angle_deg(target_deg - current_deg)

def blend_angle_deg(current_deg, target_deg, blend):
    return normalize_angle_deg(current_deg + (blend * angular_error_deg(target_deg, current_deg)))

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
        print("Peringatan: Shared memory kompas tidak aktif. Menggunakan simulasi kompas magnetik.")

    # Ambil pembacaan awal kompas untuk inisialisasi agar terhindar dari startup transient
    initial_heading_raw = 257.40  # Nilai default jika simulasi
    if compass_shm is not None:
        shm_val = get_compass_heading(compass_shm)
        if shm_val is not None:
            initial_heading_raw = shm_val

    # Konversi hadap awal sesuai metode di sensor_readers.py
    initial_heading_deg = invert_compass_heading_deg(initial_heading_raw)

    # Inisialisasi variabel integrasi dengan nilai awal kompas
    roll_gyro_pure = 0.0
    pitch_gyro_pure = 0.0
    yaw_gyro_pure = initial_heading_deg
    
    roll_cf = 0.0
    pitch_cf = 0.0
    yaw_cf = initial_heading_deg
    
    log_data = []
    duration = 20.0  # Durasi pengumpulan data: 20 detik
    rate_hz = 50     # Frekuensi pengambilan data: 50 Hz
    dt = 1.0 / rate_hz
    
    print(f"\nMemulai pengumpulan data selama {duration} detik pada rate {rate_hz} Hz...")
    print(f"Yaw Awal diinisialisasi ke: {initial_heading_deg:.2f} derajat (raw compass: {initial_heading_raw:.2f})")
    
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
                gx_dps = 15.0 * math.cos(t_val)   # Kecepatan sudut X
                gy_dps = -10.0 * math.sin(t_val)  # Kecepatan sudut Y
                gz_dps = 8.0 * math.sin(t_val)    # Kecepatan sudut Z (yaw rate)

            # 2. Hitung Sudut dari Akselerometer
            roll_accel = math.degrees(math.atan2(ay_g, az_g))
            pitch_accel = math.degrees(math.atan2(-ax_g, math.sqrt((ay_g * ay_g) + (az_g * az_g))))

            # 3. Integrasi Murni Giroskop (Tanpa Filter)
            roll_gyro_pure += gx_dps * dt
            pitch_gyro_pure += gy_dps * dt
            yaw_gyro_pure = normalize_angle_deg(yaw_gyro_pure + gz_dps * dt)

            # 4. Filter Komplementer Roll & Pitch
            roll_cf = (COMPLEMENTARY_FILTER_ALPHA * (roll_cf + gx_dps * dt)) + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * roll_accel)
            pitch_cf = (COMPLEMENTARY_FILTER_ALPHA * (pitch_cf + gy_dps * dt)) + ((1.0 - COMPLEMENTARY_FILTER_ALPHA) * pitch_accel)

            # 5. Dapatkan Arah Hadap Kompas (Magnetometer)
            shm_heading = get_compass_heading(compass_shm)
            if shm_heading is not None:
                heading_raw = shm_heading
            else:
                # Simulasi pembacaan kompas HMC5883L (arah hadap aktual + derau acak kompas)
                actual_yaw_raw = 257.40 + 10.0 * math.sin(now * 0.5)
                noise = random.gauss(0, 1.5)
                heading_raw = (actual_yaw_raw + noise) % 360

            # Konversi dan inversi sesuai metode di sensor_readers.py
            compass_heading_deg = invert_compass_heading_deg(heading_raw)

            # 6. Fusi Filter Komplementer Yaw dengan Kompas
            yaw_gyro_integrated = normalize_angle_deg(yaw_cf + gz_dps * dt)
            yaw_cf = blend_angle_deg(yaw_gyro_integrated, compass_heading_deg, 1.0 - COMPLEMENTARY_FILTER_ALPHA)

            # 7. Konversi Yaw ke Output Format NMEA [0, 360] derajat untuk penyimpanan log
            yaw_cf_360 = (-yaw_cf) % 360.0
            yaw_gyro_pure_360 = (-yaw_gyro_pure) % 360.0

            # Simpan data ke memori
            log_data.append([
                f"{now:.3f}",
                f"{ax_g:.4f}", f"{ay_g:.4f}", f"{az_g:.4f}",
                f"{gx_dps:.4f}", f"{gy_dps:.4f}", f"{gz_dps:.4f}",
                f"{roll_gyro_pure:.4f}", f"{pitch_gyro_pure:.4f}", f"{yaw_gyro_pure_360:.4f}",
                f"{roll_accel:.4f}", f"{pitch_accel:.4f}",
                f"{roll_cf:.4f}", f"{pitch_cf:.4f}", f"{yaw_cf_360:.4f}",
                f"{heading_raw:.2f}"
            ])
            
            # Tunggu hingga siklus 50Hz terpenuhi
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\nPengumpulan data dihentikan secara manual.")

    # 8. Tulis ke CSV
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
                "Roll Gyro Pure (deg)", "Pitch Gyro Pure (deg)", "Yaw Gyro Pure (deg)",
                "Roll Accel (deg)", "Pitch Accel (deg)",
                "Roll CF (deg)", "Pitch CF (deg)", "Yaw CF (deg)",
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
