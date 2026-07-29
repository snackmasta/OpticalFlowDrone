import os
import csv
import time


class CSVLogger:
    """
    Handles CSV session logging for optical flow telemetry, positions, and sensor readings.
    """

    def __init__(self, csv_arg=None, is_record_mode=False):
        self.csv_path = self._resolve_csv_path(csv_arg, is_record_mode)
        self.file = None
        self.writer = None
        if self.csv_path:
            self._init_csv()

    def _resolve_csv_path(self, csv_arg, is_record_mode):
        if csv_arg is not None:
            if csv_arg == "auto":
                os.makedirs("recordings", exist_ok=True)
                return os.path.join("recordings", f"optical_flow_session_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            return csv_arg
        elif is_record_mode:
            os.makedirs("recordings", exist_ok=True)
            return os.path.join("recordings", f"optical_flow_session_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        return None

    def _init_csv(self):
        try:
            csv_dir = os.path.dirname(os.path.abspath(self.csv_path))
            if csv_dir:
                os.makedirs(csv_dir, exist_ok=True)
            self.file = open(self.csv_path, "w", newline="")
            self.writer = csv.writer(self.file)
            self.writer.writerow([
                "Timestamp (s)",
                "X Position (cm)",
                "Y Position (cm)",
                "Raw X Position (cm)",
                "Raw Y Position (cm)",
                "VX (m/s)",
                "VY (m/s)",
                "VZ (m/s)",
                "Raw VX (m/s)",
                "Raw VY (m/s)",
                "Speed (m/s)",
                "Altitude (m)",
                "Roll (deg)",
                "Pitch (deg)",
                "Yaw (deg)",
                "Heading (deg)",
                "Gyro Z (dps)",
                "Accel X (g)",
                "Accel Y (g)",
                "Inliers"
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
        current_x_cm,
        current_y_cm,
        x_raw_cm,
        y_raw_cm,
        current_vx,
        current_vy,
        vz_mps,
        vx_raw_mps,
        vy_raw_mps,
        speed_mps,
        current_alt,
        roll_deg,
        pitch_deg,
        yaw_deg,
        zgyro_dps,
        xaccel_g,
        yaccel_g,
        tracked_count
    ):
        if self.writer is None:
            return

        try:
            heading_deg = (-yaw_deg) % 360.0
            self.writer.writerow([
                f"{frame_ts:.4f}",
                f"{current_x_cm:.4f}",
                f"{current_y_cm:.4f}",
                f"{x_raw_cm:.4f}",
                f"{y_raw_cm:.4f}",
                f"{current_vx:.4f}",
                f"{current_vy:.4f}",
                f"{vz_mps:.4f}",
                f"{vx_raw_mps:.4f}",
                f"{vy_raw_mps:.4f}",
                f"{speed_mps:.4f}",
                f"{current_alt:.4f}",
                f"{roll_deg:.4f}",
                f"{pitch_deg:.4f}",
                f"{yaw_deg:.4f}",
                f"{heading_deg:.4f}",
                f"{zgyro_dps:.4f}",
                f"{xaccel_g:.4f}",
                f"{yaccel_g:.4f}",
                tracked_count
            ])
            self.file.flush()
        except Exception as e:
            print(f"Error writing to CSV: {e}")

    def close(self):
        if self.file is not None:
            try:
                self.file.close()
                print(f"\nCSV session log saved successfully: {self.csv_path}")
            except Exception as e:
                print(f"\nError closing CSV file: {e}")
            finally:
                self.file = None
                self.writer = None
