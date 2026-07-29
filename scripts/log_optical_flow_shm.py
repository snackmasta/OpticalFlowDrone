#!/usr/bin/env python3
"""
Standalone Script to Log optical_flow_stream Shared Memory (SHM) to a CSV File.
Tracks write index changes to read and save every single incoming sample.
"""

import time
import struct
import csv
import os
import sys
import math
import random
from multiprocessing import shared_memory

# SHM Configuration
FLOW_SHM_NAME = "optical_flow_stream"
FLOW_SHM_MAGIC = b"FLOW"
FLOW_SHM_HEADER_FORMAT = "<4sII"
FLOW_SHM_RECORD_FORMAT = "<11d"
FLOW_SHM_HEADER_SIZE = struct.calcsize(FLOW_SHM_HEADER_FORMAT)
FLOW_SHM_RECORD_SIZE = struct.calcsize(FLOW_SHM_RECORD_FORMAT)
FLOW_MAX_SAMPLES = 120

# Output Path
OUTPUT_DIR = "hasil_dan_pembahasan"
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "optical_flow_shm_log.csv")

def main():
    """
    Main loop for logging optical flow shared memory data.
    Attempts to attach to the shared memory block and track sample index updates,
    writing all captured records to a CSV file. If shared memory is not available,
    runs in circular motion simulation mode.
    """
    print("=================================================================")
    print("          OPTICAL FLOW SHM DATA LOGGING SCRIPT (STANDALONE)      ")
    print("=================================================================")
    
    # Inisialisasi Berkas Output
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        
    print(f"Menyimpan log data ke: {OUTPUT_CSV}")
    print("Tekan Ctrl+C untuk menghentikan perekaman.")
    
    # Hubungkan ke Shared Memory
    shm = None
    try:
        shm = shared_memory.SharedMemory(name=FLOW_SHM_NAME)
        print("Berhasil terhubung ke Shared Memory 'optical_flow_stream'!")
    except FileNotFoundError:
        print("Peringatan: Shared Memory tidak aktif. Mengaktifkan mode simulasi data.")
        shm = None
        
    # Buka file CSV dan tulis header jika belum ada
    file_exists = os.path.exists(OUTPUT_CSV)
    try:
        csv_file = open(OUTPUT_CSV, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "Timestamp (s)",
            "X Position (cm)", "Y Position (cm)",
            "Raw X Position (cm)", "Raw Y Position (cm)",
            "VX (m/s)", "VY (m/s)",
            "Raw VX (m/s)", "Raw VY (m/s)",
            "Altitude (m)", "Heading (deg)"
        ])
    except Exception as e:
        print(f"Gagal membuka berkas CSV untuk penulisan: {e}")
        if shm:
            shm.close()
        sys.exit(1)
        
    last_read_idx = None
    sample_counter = 0
    start_time = time.time()
    
    try:
        while True:
            if shm is not None:
                try:
                    # Baca header SHM
                    magic, write_index, sample_count = struct.unpack_from(FLOW_SHM_HEADER_FORMAT, shm.buf, 0)
                    if magic != FLOW_SHM_MAGIC or sample_count == 0:
                        time.sleep(0.01)
                        continue
                        
                    if last_read_idx is None:
                        # Inisialisasi awal index baca
                        last_read_idx = (write_index - 1) % FLOW_MAX_SAMPLES
                        
                    if write_index != last_read_idx:
                        # Baca data baru (bisa lebih dari satu sampel jika rate logger lambat)
                        current_idx = last_read_idx
                        while current_idx != write_index:
                            offset = FLOW_SHM_HEADER_SIZE + (current_idx * FLOW_SHM_RECORD_SIZE)
                            # Record: timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading
                            values = struct.unpack_from(FLOW_SHM_RECORD_FORMAT, shm.buf, offset)
                            
                            writer.writerow([
                                f"{values[0]:.4f}", 
                                f"{values[1]:.4f}", f"{values[2]:.4f}",
                                f"{values[3]:.4f}", f"{values[4]:.4f}",
                                f"{values[5]:.4f}", f"{values[6]:.4f}",
                                f"{values[7]:.4f}", f"{values[8]:.4f}",
                                f"{values[9]:.4f}", f"{values[10]:.4f}"
                            ])
                            csv_file.flush()
                            sample_counter += 1
                            
                            current_idx = (current_idx + 1) % FLOW_MAX_SAMPLES
                            
                        last_read_idx = write_index
                        if sample_counter % 20 == 0:
                            print(f"Berhasil merekam {sample_counter} data sampel...")
                            
                except Exception as e:
                    print(f"Error membaca Shared Memory: {e}")
                    break
            else:
                # Mode Simulasi Gerak Melingkar (Untuk PC Penguji)
                now = time.time()
                t_sec = now - start_time
                
                # Simulasi koordinat gerak lingkaran
                radius_cm = 50.0
                freq_hz = 0.1
                theta = 2.0 * math.pi * freq_hz * t_sec
                
                x_pos = radius_cm * math.cos(theta)
                y_pos = radius_cm * math.sin(theta)
                x_raw = x_pos + random.gauss(0, 1.5)
                y_raw = y_pos + random.gauss(0, 1.5)
                
                vx = -2.0 * math.pi * freq_hz * (radius_cm / 100.0) * math.sin(theta)
                vy = 2.0 * math.pi * freq_hz * (radius_cm / 100.0) * math.cos(theta)
                vx_raw = vx + random.gauss(0, 0.05)
                vy_raw = vy + random.gauss(0, 0.05)
                
                alt = 1.20 + 0.1 * math.sin(t_sec * 0.2)
                heading = (theta * 180.0 / math.pi) % 360.0
                
                writer.writerow([
                    f"{now:.4f}",
                    f"{x_pos:.4f}", f"{y_pos:.4f}",
                    f"{x_raw:.4f}", f"{y_raw:.4f}",
                    f"{vx:.4f}", f"{vy:.4f}",
                    f"{vx_raw:.4f}", f"{vy_raw:.4f}",
                    f"{alt:.4f}", f"{heading:.4f}"
                ])
                csv_file.flush()
                sample_counter += 1
                
                if sample_counter % 20 == 0:
                    print(f"Berhasil merekam {sample_counter} data sampel (simulasi)...")
                
                time.sleep(0.05)  # Frekuensi 20 Hz
                
    except KeyboardInterrupt:
        print("\nPerekaman data dihentikan secara manual.")
    finally:
        csv_file.close()
        if shm:
            shm.close()
        print(f"Selesai! Total {sample_counter} data telah dicatat di berkas log.")

if __name__ == "__main__":
    main()
