"""
Modul Pengelola Fase Pengujian & Logging Telemetri Multi-Fase (F0 - F6)
======================================================================
Deskripsi:
    Modul ini mengelola 7 fase pengujian kendaraan/drone tanpa GPS/LiDAR (ketinggian statis):
    - F0: Diam Awal (Orientasi magnetometer & status koneksi sensor)
    - F1: Statis Lanjutan (Perekaman bias sensor/derau complementary filter)
    - F2: Gerak Lurus Terukur (Perekaman marka fisik & translasi optical flow)
    - F3: Belokan Terukur (Sudut belokan 90° & flow terkompensasi vs mentah)
    - F4: Pelintasan Batas Geofence (Deteksi breach & event status buzzer)
    - F5: Getaran/Derau Permukaan (Low texture / getaran & flag fallback/outlier)
    - F6: Selesai/Diam Akhir (Verifikasi bias akhir, baris/ukuran CSV, status akhir)
"""

import os
import time
import json

PHASE_IPC_FILE = os.path.join("logs", "active_phase.json")


class PhaseTestManager:
    """
    State manager for F0-F6 test phase lifecycle and event marking.
    Supports inter-process sync via logs/active_phase.json.
    """
    PHASES = {
        "F0": {
            "name": "F0 - Diam Awal",
            "indicative_duration": "60-90s",
            "description": "Kendaraan diam datar; cek magnetometer per 4 orientasi"
        },
        "F1": {
            "name": "F1 - Statis Lanjutan",
            "indicative_duration": "2-3m",
            "description": "Kendaraan tetap diam; rekam bias/derau complementary filter & suhu"
        },
        "F2": {
            "name": "F2 - Gerak Lurus Terukur",
            "indicative_distance": "8-10m",
            "description": "Gerak lurus; catat penanda waktu per marka jarak fisik"
        },
        "F3": {
            "name": "F3 - Belokan Terukur",
            "indicative_turns": "1 kali",
            "description": "Belokan sudut acuan (mis. 90°); flow terkompensasi vs mentah"
        },
        "F4": {
            "name": "F4 - Pelintasan Batas Geofence",
            "indicative_events": "1x keluar + 1x kembali",
            "description": "Melintasi batas geofence; status breach & buzzer alert"
        },
        "F5": {
            "name": "F5 - Getaran/Derau Permukaan",
            "indicative_distance": "2-3m",
            "description": "Permukaan getar/low texture; inlier count & fallback flag"
        },
        "F6": {
            "name": "F6 - Selesai/Diam Akhir",
            "indicative_duration": "30s",
            "description": "Berhenti kembali; verifikasi bias akhir & status log berkas"
        }
    }

    def __init__(self, initial_phase="F0"):
        self.current_phase = initial_phase if initial_phase in self.PHASES else "F0"
        self.phase_start_ts = time.time()
        self.active_event_marker = ""
        self.mag_orientation_count = 0
        self.distance_mark_count = 0
        self._sync_to_file()

    def _sync_to_file(self):
        """Writes active phase & marker to IPC JSON file."""
        try:
            os.makedirs("logs", exist_ok=True)
            data = {
                "phase": self.current_phase,
                "marker": self.active_event_marker,
                "updated_at": time.time()
            }
            with open(PHASE_IPC_FILE, "w") as f:
                json.dump(data, f)
        except Exception:
            pass

    def _sync_from_file(self):
        """Reads active phase & marker from IPC JSON file if modified."""
        try:
            if os.path.exists(PHASE_IPC_FILE):
                with open(PHASE_IPC_FILE, "r") as f:
                    data = json.load(f)
                    if data.get("phase") in self.PHASES:
                        if data["phase"] != self.current_phase:
                            self.current_phase = data["phase"]
                            self.phase_start_ts = time.time()
                        if data.get("marker") is not None:
                            self.active_event_marker = data["marker"]
        except Exception:
            pass

    def set_phase(self, phase_code):
        """Sets active phase code (F0 - F6)."""
        if phase_code in self.PHASES:
            self.current_phase = phase_code
            self.phase_start_ts = time.time()
            self.active_event_marker = f"START_{phase_code}"
            self._sync_to_file()
            print(f"[PhaseTestManager] Switched to Phase {self.PHASES[phase_code]['name']}")
            return True
        return False

    def mark_event(self, event_text):
        """Sets temporary custom event marker for the upcoming frames."""
        self.active_event_marker = event_text
        self._sync_to_file()
        print(f"[PhaseTestManager] Event Marker set: {event_text}")

    def pop_event_marker(self):
        """Returns active event marker and clears transient state if set."""
        self._sync_from_file()
        marker = self.active_event_marker
        if marker:
            self.active_event_marker = ""
            self._sync_to_file()
        return marker

    def get_phase_info(self):
        """Returns detail dict of current phase."""
        self._sync_from_file()
        info = dict(self.PHASES[self.current_phase])
        info["code"] = self.current_phase
        info["elapsed_s"] = time.time() - self.phase_start_ts
        return info

