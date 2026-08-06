"""
Modul Logging CSV Telemetri Multi-Fase Penerbangan & Pengujian Sensor
======================================================================
Deskripsi:
    Modul ini menyediakan kelas `CSVLogger` untuk mencatat log telemetry penerbangan,
    kalkulasi Optical Flow, estimasi posisi/kecepatan, serta pembacaan sensor hardware
    dan fase pengujian (F0 hingga F6) ke dalam berkas CSV.
"""

import os
import csv
import time


class CSVLogger:
    """
    Handles CSV session logging for multi-phase flight telemetry, positions, and sensor readings.
    """

    def __init__(self, csv_arg=None, is_record_mode=False, static_alt_m=1.0):
        self.static_alt_m = static_alt_m
        self.csv_path = self._resolve_csv_path(csv_arg, is_record_mode)
        self.file = None
        self.writer = None
        self.row_count = 0
        self.start_timestamp = time.time()
        if self.csv_path:
            self._init_csv()

    def _resolve_csv_path(self, csv_arg, is_record_mode):
        if csv_arg is not None:
            if csv_arg == "auto":
                os.makedirs("recordings", exist_ok=True)
                return os.path.join("recordings", f"optical_flow_multiphase_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            return csv_arg
        elif is_record_mode:
            os.makedirs("recordings", exist_ok=True)
            return os.path.join("recordings", f"optical_flow_multiphase_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        return None

    def _init_csv(self):
        try:
            csv_dir = os.path.dirname(os.path.abspath(self.csv_path))
            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)
            self.file = open(self.csv_path, "w", newline="")
            self.writer = csv.writer(self.file)

            # Metadata header comments
            self.writer.writerow(["# SESSION METADATA"])
            self.writer.writerow(["# Start Time", time.strftime('%Y-%m-%d %H:%M:%S')])
            self.writer.writerow(["# Static Reference Altitude (m)", f"{self.static_alt_m:.2f}"])
            self.writer.writerow(["# Test Phases", "F0 (Diam Awal), F1 (Statis Lanjutan), F2 (Gerak Lurus), F3 (Belokan), F4 (Geofence), F5 (Getaran), F6 (Selesai)"])
            self.writer.writerow([])

            # Column Headers
            self.writer.writerow([
                "Timestamp (s)",
                "Phase",
                "Event_Marker",
                "Sensor_Conn_Status",
                "FPS_Actual",
                "Frame_Index",
                "Raw_Accel_X (g)",
                "Raw_Accel_Y (g)",
                "Raw_Accel_Z (g)",
                "Raw_Gyro_X (dps)",
                "Raw_Gyro_Y (dps)",
                "Raw_Gyro_Z (dps)",
                "Raw_Mag_X (uT)",
                "Raw_Mag_Y (uT)",
                "Raw_Mag_Z (uT)",
                "Sensor_Temp_C",
                "CF_Roll (deg)",
                "CF_Pitch (deg)",
                "CF_Yaw (deg)",
                "Fused_Heading (deg)",
                "OF_Inliers",
                "OF_Tx_px",
                "OF_Ty_px",
                "OF_VX (m/s)",
                "OF_VY (m/s)",
                "Raw_VX (m/s)",
                "Raw_VY (m/s)",
                "Speed (m/s)",
                "Fused_X (cm)",
                "Fused_Y (cm)",
                "Raw_X (cm)",
                "Raw_Y (cm)",
                "Static_Altitude (m)",
                "Geofence_Status",
                "Geofence_Breach_Event",
                "Buzzer_Signal",
                "Surface_Noise_Fallback_Flag",
                "Comms_UART_FC_Status",
                "Comms_UDP_Dash_Status",
                "Comms_RTSP_HUD_Status"
            ])
            self.file.flush()
            print(f"Logging session sensor readings and trajectory to CSV: {self.csv_path}")
        except Exception as e:
            print(f"Failed to open CSV file for logging: {e}")
            self.file = None
            self.writer = None

    def log_frame(
        self,
        frame_ts,
        phase="F0",
        event_marker="",
        sensor_conn_status="IMU:OK|MAG:OK|CAM:OK",
        fps_actual=60.0,
        frame_index=0,
        raw_accel_x=0.0,
        raw_accel_y=0.0,
        raw_accel_z=1.0,
        raw_gyro_x=0.0,
        raw_gyro_y=0.0,
        raw_gyro_z=0.0,
        raw_mag_x=0.0,
        raw_mag_y=0.0,
        raw_mag_z=0.0,
        sensor_temp_c=25.0,
        cf_roll=0.0,
        cf_pitch=0.0,
        cf_yaw=0.0,
        fused_heading=0.0,
        of_inliers=0,
        of_tx_px=0.0,
        of_ty_px=0.0,
        of_vx=0.0,
        of_vy=0.0,
        raw_vx=0.0,
        raw_vy=0.0,
        speed_mps=0.0,
        fused_x_cm=0.0,
        fused_y_cm=0.0,
        raw_x_cm=0.0,
        raw_y_cm=0.0,
        geofence_status="IN",
        geofence_breach_event=0,
        buzzer_signal="OFF",
        surface_noise_fallback=0,
        comms_uart="OK",
        comms_udp="OK",
        comms_rtsp="OK"
    ):
        if self.writer is None:
            return

        try:
            self.writer.writerow([
                f"{frame_ts:.4f}",
                phase,
                event_marker,
                sensor_conn_status,
                f"{fps_actual:.2f}",
                frame_index,
                f"{raw_accel_x:.4f}",
                f"{raw_accel_y:.4f}",
                f"{raw_accel_z:.4f}",
                f"{raw_gyro_x:.4f}",
                f"{raw_gyro_y:.4f}",
                f"{raw_gyro_z:.4f}",
                f"{raw_mag_x:.2f}",
                f"{raw_mag_y:.2f}",
                f"{raw_mag_z:.2f}",
                f"{sensor_temp_c:.1f}",
                f"{cf_roll:.4f}",
                f"{cf_pitch:.4f}",
                f"{cf_yaw:.4f}",
                f"{fused_heading:.4f}",
                of_inliers,
                f"{of_tx_px:.2f}",
                f"{of_ty_px:.2f}",
                f"{of_vx:.4f}",
                f"{of_vy:.4f}",
                f"{raw_vx:.4f}",
                f"{raw_vy:.4f}",
                f"{speed_mps:.4f}",
                f"{fused_x_cm:.4f}",
                f"{fused_y_cm:.4f}",
                f"{raw_x_cm:.4f}",
                f"{raw_y_cm:.4f}",
                f"{self.static_alt_m:.2f}",
                geofence_status,
                geofence_breach_event,
                buzzer_signal,
                surface_noise_fallback,
                comms_uart,
                comms_udp,
                comms_rtsp
            ])
            self.row_count += 1
            self.file.flush()
        except Exception as e:
            print(f"Error writing to CSV: {e}")

    def close(self, session_status="SUCCESS"):
        if self.file is not None:
            try:
                # Append session summary row
                self.writer.writerow([])
                self.writer.writerow(["# SESSION SUMMARY"])
                self.writer.writerow(["# Total Log Rows", self.row_count])
                self.writer.writerow(["# Duration (s)", f"{time.time() - self.start_timestamp:.2f}"])
                self.writer.writerow(["# Final Session Status", session_status])
                self.file.flush()
                file_size = os.path.getsize(self.csv_path) if os.path.exists(self.csv_path) else 0
                self.file.close()
                print(f"\n=======================================================")
                print(f"CSV Session Log Saved Successfully: {self.csv_path}")
                print(f"Total Rows: {self.row_count} | Size: {file_size / 1024:.2f} KB | Status: {session_status}")
                print(f"=======================================================\n")
            except Exception as e:
                print(f"\nError closing CSV file: {e}")
            finally:
                self.file = None
                self.writer = None
