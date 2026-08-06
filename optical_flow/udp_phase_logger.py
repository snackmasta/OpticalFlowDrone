"""
Modul Subscriber UDP Telemetri untuk Logging Multi-Fase (F0 - F6)
==================================================================
Modul ini mendengarkan broadcast telemetry UDP dari `send_attitude_udp.py`
pada port 5005 dan mencatat sampel telemetri tersebut secara real-time
ke dalam berkas CSV sesuai dengan fase pengujian aktif (F0-F6).
"""

import os
import sys
import time
import json
import socket
import threading
from optical_flow.csv_logger import CSVLogger
from optical_flow.phase_logger import PhaseTestManager


class UDPPhaseLogger:
    """
    Listens to UDP telemetry stream from send_attitude_udp.py and logs
    parsed records to CSV file aligned with active PhaseTestManager phase.
    """
    def __init__(self, csv_filename, udp_host="127.0.0.1", udp_port=5005):
        self.csv_filename = csv_filename
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.csv_logger = CSVLogger(csv_arg=csv_filename)
        self.phase_manager = PhaseTestManager(initial_phase="F0")

        self.running = False
        self.thread = None
        self.sock = None

        self.packet_count = 0
        self.bytes_received = 0
        self.last_packet_ts = 0.0
        self.last_telemetry = None

    def start(self):
        """Starts background UDP listening & CSV logging thread."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        print(f"[UDPPhaseLogger] Listener active on UDP {self.udp_host}:{self.udp_port} -> Logging to: {self.csv_filename}")

    def stop(self):
        """Stops background thread and flushes/closes CSV logger."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.csv_logger.close(session_status="SUCCESS")
        print(f"[UDPPhaseLogger] Listener stopped. Total logged rows: {self.csv_logger.row_count}")

    def get_stats(self):
        """Returns real-time status of UDP packet reception and CSV output."""
        file_size = os.path.getsize(self.csv_filename) if os.path.exists(self.csv_filename) else 0
        if file_size < 1024:
            size_str = f"{file_size} B"
        elif file_size < 1024 * 1024:
            size_str = f"{file_size / 1024:.2f} KB"
        else:
            size_str = f"{file_size / (1024 * 1024):.2f} MB"

        is_active = (time.time() - self.last_packet_ts) < 2.0 and self.packet_count > 0

        return {
            "udp_source": f"{self.udp_host}:{self.udp_port}",
            "packet_count": self.packet_count,
            "bytes_received": self.bytes_received,
            "csv_rows": self.csv_logger.row_count,
            "file_size_str": size_str,
            "is_active": is_active,
            "status": "ACTIVE RECEIVING UDP & WRITING CSV" if is_active else "WAITING FOR UDP BROADCAST",
            "last_telemetry": self.last_telemetry
        }

    def _run_loop(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except Exception:
                    pass
        except Exception:
            pass

        try:
            # Bind to 0.0.0.0 or 127.0.0.1 to listen for incoming UDP packets
            bind_host = "" if self.udp_host in ["0.0.0.0", "127.0.0.1"] else self.udp_host
            self.sock.bind((bind_host, self.udp_port))
            self.sock.settimeout(1.0)
        except Exception as e:
            print(f"[UDPPhaseLogger ERROR] Failed to bind UDP port {self.udp_port}: {e}")
            return

        frame_index = 0
        while self.running:
            try:
                data, addr = self.sock.recvfrom(65535)
                if not data:
                    continue

                self.bytes_received += len(data)
                self.packet_count += 1
                self.last_packet_ts = time.time()

                packet = json.loads(data.decode("utf-8", errors="ignore"))
                self.last_telemetry = packet

                # Extract phase & event marker synced from PhaseTestManager IPC
                phase_info = self.phase_manager.get_phase_info()
                current_phase = phase_info["code"]
                event_marker = self.phase_manager.pop_event_marker()

                # Extract fields from UDP telemetry packet sent by send_attitude_udp.py
                ts = float(packet.get("timestamp", time.time()))
                euler = packet.get("rotation", {}).get("euler", {})
                roll = float(euler.get("roll", 0.0))
                pitch = float(euler.get("pitch", 0.0))
                yaw = float(euler.get("yaw", 0.0))
                heading = float(packet.get("heading", yaw))

                pos = packet.get("translation", {}).get("position", {})
                fused_x_cm = float(pos.get("x", 0.0)) * 100.0
                fused_y_cm = float(pos.get("z", 0.0)) * 100.0  # Z axis in 3D web packet is the 2D Y translation
                static_alt = abs(float(pos.get("y", 1.5)))

                vel = packet.get("translation", {}).get("velocity", {})
                of_vx = float(vel.get("x", 0.0))
                of_vy = float(vel.get("z", 0.0))
                speed = float((of_vx**2 + of_vy**2)**0.5)

                flow_data = packet.get("optical_flow", {})
                of_inliers = int(flow_data.get("inliers", 0))

                gyro_data = packet.get("gyro", {})
                gyro_x = float(gyro_data.get("gx", 0.0))
                gyro_y = float(gyro_data.get("gy", 0.0))
                gyro_z = float(gyro_data.get("gz", 0.0))

                frame_index += 1

                self.csv_logger.log_frame(
                    frame_ts=ts,
                    phase=current_phase,
                    event_marker=event_marker,
                    sensor_conn_status="IMU:OK|MAG:OK|UDP:OK",
                    fps_actual=50.0,
                    frame_index=frame_index,
                    raw_gyro_x=gyro_x,
                    raw_gyro_y=gyro_y,
                    raw_gyro_z=gyro_z,
                    cf_roll=roll,
                    cf_pitch=pitch,
                    cf_yaw=yaw,
                    fused_heading=heading,
                    of_inliers=of_inliers,
                    of_vx=of_vx,
                    of_vy=of_vy,
                    speed_mps=speed,
                    fused_x_cm=fused_x_cm,
                    fused_y_cm=fused_y_cm,
                    geofence_status="IN",
                    comms_udp="OK"
                )

            except socket.timeout:
                continue
            except Exception:
                continue
