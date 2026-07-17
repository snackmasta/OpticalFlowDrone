#!/usr/bin/env python3
"""Read the live compass heading stream from shared memory and provide a dashboard."""

import argparse
import struct
import time
import threading
import math
import random
from multiprocessing import shared_memory
from multiprocessing import resource_tracker

# CLI constants
SHM_NAME = "compass_heading_stream"
SHM_MAGIC = b"CHDG"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<6d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
DEFAULT_FRESHNESS_THRESHOLD = 0.75

# Shared memory registry for multi-segment visualization
SHM_REGISTRY = {
    "compass_heading_stream": {
        "magic": b"CHDG",
        "header_format": "<4sII",  # magic, write_index, sample_count
        "record_format": "<6d",    # timestamp, raw_heading, heading, x, y, z
        "fields": ["timestamp", "raw_heading", "heading", "x", "y", "z"],
        "max_samples": 120,
    },
    "optical_flow_stream": {
        "magic": b"FLOW",
        "header_format": "<4sII",  # magic, write_index, sample_count
        "record_format": "<11d",   # timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading
        "fields": ["timestamp", "x_cm", "y_cm", "x_raw_cm", "y_raw_cm", "vx", "vy", "vx_raw", "vy_raw", "alt", "heading"],
        "max_samples": 120,
    },
    "future_sensor_stream": {
        "magic": b"FUTR",
        "header_format": "<4sII",  # magic, write_index, sample_count
        "record_format": "<4d",    # timestamp, temperature, pressure, altitude
        "fields": ["timestamp", "temperature", "pressure", "altitude"],
        "max_samples": 100,
    },
    "battery_status_stream": {
        "magic": b"BATT",
        "header_format": "<4sII",  # magic, write_index, sample_count
        "record_format": "<5d",    # timestamp, voltage, current, capacity, consumed_mah
        "fields": ["timestamp", "voltage", "current", "capacity", "consumed_mah"],
        "max_samples": 120,
    }
}

mock_threads = {}
mock_active = {}


def safe_unregister_shm(shm):
    """Prevents Python's resource_tracker from unlinking shared memory when a process exits."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass


def read_latest_sample(shm):
    """Returns the latest sample as a dict, or None if no valid sample is available."""
    magic, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
    if magic != SHM_MAGIC or sample_count == 0:
        return None

    latest_index = (write_index - 1) % MAX_SAMPLES
    record_offset = SHM_HEADER_SIZE + (latest_index * SHM_RECORD_SIZE)
    timestamp, raw_heading, heading, x, y, z = struct.unpack_from(SHM_RECORD_FORMAT, shm.buf, record_offset)
    return {
        "timestamp": timestamp,
        "raw_heading": raw_heading,
        "heading": heading,
        "x": x,
        "y": y,
        "z": z,
        "sample_count": sample_count,
        "latest_index": latest_index,
    }


def open_shared_memory(wait_interval=0.5):
    """Tries to open the shared memory segment, retrying until it becomes available."""
    while True:
        try:
            shm = shared_memory.SharedMemory(name=SHM_NAME)
            safe_unregister_shm(shm)
            return shm
        except FileNotFoundError:
            time.sleep(wait_interval)


def release_shared_memory(shm):
    """Releases the shared memory segment, ensuring it is properly closed and unlinked."""
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass

    try:
        shm.close()
    except Exception:
        pass


def sample_age_seconds(sample_timestamp):
    """Returns the age of the sample in seconds."""
    return time.time() - sample_timestamp


def sample_is_fresh(sample, freshness_threshold):
    """Determines if the sample is fresh."""
    return sample is not None and sample_age_seconds(sample["timestamp"]) <= freshness_threshold


def format_sample(sample):
    """Formats the sample for display."""
    return f"{sample['heading']:.2f}"


# Telemetry and hex-dump helpers for dashboard

def get_shm_samples(shm_name):
    """Read all available samples from the specified shared memory name."""
    cfg = SHM_REGISTRY.get(shm_name)
    if not cfg:
        return []

    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        safe_unregister_shm(shm)
    except FileNotFoundError:
        return []

    try:
        h_format = cfg["header_format"]
        r_format = cfg["record_format"]
        header_size = struct.calcsize(h_format)
        record_size = struct.calcsize(r_format)

        # Copy buffer to bytes to avoid memoryview export leakage
        buf = bytes(shm.buf)

        magic, write_index, sample_count = struct.unpack_from(h_format, buf, 0)
        if magic != cfg["magic"] or sample_count == 0:
            return []

        samples = []
        for i in range(sample_count):
            idx = (write_index - 1 - i) % cfg["max_samples"]
            offset = header_size + (idx * record_size)
            values = struct.unpack_from(r_format, buf, offset)
            sample = dict(zip(cfg["fields"], values))
            samples.append(sample)
        return samples
    except Exception:
        return []
    finally:
        try:
            shm.close()
        except Exception:
            pass


def get_shm_hex_dump(shm_name, max_bytes=256):
    """Get formatted hex dump lines of the shared memory buffer."""
    try:
        shm = shared_memory.SharedMemory(name=shm_name)
        safe_unregister_shm(shm)
    except FileNotFoundError:
        return []

    try:
        # Copy to bytes to avoid memoryview export leakage
        buf = bytes(shm.buf[:min(shm.size, max_bytes)])
        hex_rows = []
        for offset in range(0, len(buf), 16):
            chunk = buf[offset:offset + 16]
            hex_parts = [f"{b:02x}" for b in chunk]
            ascii_parts = [chr(b) if 32 <= b <= 126 else "." for b in chunk]

            hex_str = " ".join(hex_parts)
            if len(chunk) < 16:
                hex_str += " " * (3 * (16 - len(chunk)))

            ascii_str = "".join(ascii_parts)
            hex_rows.append({
                "address": f"0x{offset:04x}",
                "hex": hex_str,
                "ascii": ascii_str
            })
        return hex_rows
    except Exception:
        return []
    finally:
        try:
            shm.close()
        except Exception:
            pass


# Mock Generator Worker Thread

def mock_worker(shm_name, frequency, noise, mode):
    cfg = SHM_REGISTRY.get(shm_name)
    if not cfg:
        return

    h_format = cfg["header_format"]
    r_format = cfg["record_format"]
    header_size = struct.calcsize(h_format)
    record_size = struct.calcsize(r_format)
    shm_size = header_size + (cfg["max_samples"] * record_size)

    # Initialize shared memory
    shm = None
    try:
        shm = shared_memory.SharedMemory(name=shm_name, create=True, size=shm_size)
        safe_unregister_shm(shm)
        struct.pack_into(h_format, shm.buf, 0, cfg["magic"], 0, 0)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=shm_name, create=False)
        safe_unregister_shm(shm)

    step = 0
    last_values = {}

    try:
        while mock_active.get(shm_name, False):
            magic, write_index, sample_count = struct.unpack_from(h_format, shm.buf, 0)

            timestamp = time.time()
            record_values = [timestamp]

            for field in cfg["fields"]:
                if field == "timestamp":
                    continue

                val = 0.0
                if shm_name == "compass_heading_stream":
                    if field in ("heading", "raw_heading"):
                        if mode == "sine":
                            val = (180.0 + 170.0 * math.sin(step * 0.05)) % 360.0
                        else:
                            last_val = last_values.get(field, 180.0)
                            val = (last_val + random.uniform(-4.0, 4.0)) % 360.0
                    else:  # x, y, z
                        if mode == "sine":
                            multiplier = 80.0 if field == "x" else (60.0 if field == "y" else 30.0)
                            val = multiplier * math.sin(step * 0.1)
                        else:
                            last_val = last_values.get(field, 0.0)
                            val = last_val + random.uniform(-5.0, 5.0)
                elif shm_name == "optical_flow_stream":
                    if field in ("x_cm", "y_cm", "x_raw_cm", "y_raw_cm"):
                        if mode == "sine":
                            multiplier = 1000.0 if field in ("x_cm", "x_raw_cm") else 500.0
                            val = multiplier * math.sin(step * 0.05)
                        else:
                            last_val = last_values.get(field, 0.0)
                            val = last_val + random.uniform(-20.0, 20.0)
                    elif field in ("vx", "vy", "vx_raw", "vy_raw"):
                        if mode == "sine":
                            multiplier = 2.0 if field in ("vx", "vx_raw") else 1.0
                            val = multiplier * math.cos(step * 0.05)
                        else:
                            last_val = last_values.get(field, 0.0)
                            val = last_val + random.uniform(-0.1, 0.1)
                    elif field == "alt":
                        if mode == "sine":
                            val = 1.5 + 0.5 * math.sin(step * 0.02)
                        else:
                            last_val = last_values.get(field, 1.5)
                            val = max(0.5, min(5.0, last_val + random.uniform(-0.05, 0.05)))
                    elif field == "heading":
                        if mode == "sine":
                            val = (180.0 + 170.0 * math.sin(step * 0.05)) % 360.0
                        else:
                            last_val = last_values.get(field, 180.0)
                            val = (last_val + random.uniform(-4.0, 4.0)) % 360.0
                elif shm_name == "battery_status_stream":
                    if field == "voltage":
                        cap_val = last_values.get("capacity", 100.0)
                        val = 10.5 + (cap_val / 100.0) * 2.1
                    elif field == "current":
                        if mode == "sine":
                            val = 5.0 + 4.0 * math.sin(step * 0.05)
                        else:
                            last_val = last_values.get(field, 5.0)
                            val = max(0.5, min(15.0, last_val + random.uniform(-0.5, 0.5)))
                    elif field == "capacity":
                        if mode == "sine":
                            val = 50.0 + 48.0 * math.sin(step * 0.01)
                        else:
                            last_val = last_values.get(field, 100.0)
                            val = last_val - 0.1
                            if val <= 0:
                                val = 100.0
                    elif field == "consumed_mah":
                        if mode == "sine":
                            val = (step * 2.5) % 2200.0
                        else:
                            last_val = last_values.get(field, 0.0)
                            curr_val = last_values.get("current", 5.0)
                            val = last_val + (curr_val * 1000.0 * (1.0 / frequency) / 3600.0)
                            if last_values.get("capacity", 100.0) >= 99.9:
                                val = 0.0
                else:  # future_sensor_stream
                    if field == "temperature":
                        val = 24.0 + 6.0 * math.sin(step * 0.02) if mode == "sine" else last_values.get(field, 24.0) + random.uniform(-0.1, 0.1)
                    elif field == "pressure":
                        val = 1013.25 + 15.0 * math.sin(step * 0.01) if mode == "sine" else last_values.get(field, 1013.25) + random.uniform(-0.4, 0.4)
                    elif field == "altitude":
                        val = 80.0 + 40.0 * math.sin(step * 0.03) if mode == "sine" else last_values.get(field, 80.0) + random.uniform(-0.8, 0.8)

                if noise > 0:
                    val += random.normalvariate(0.0, noise)

                record_values.append(val)
                last_values[field] = val

            # Write record to buffer
            offset = header_size + (write_index * record_size)
            struct.pack_into(r_format, shm.buf, offset, *record_values)

            # Update header index/count
            write_index = (write_index + 1) % cfg["max_samples"]
            sample_count = min(sample_count + 1, cfg["max_samples"])
            struct.pack_into(h_format, shm.buf, 0, cfg["magic"], write_index, sample_count)

            step += 1
            time.sleep(1.0 / frequency)

    except Exception as exc:
        print(f"Mock worker {shm_name} encountered error: {exc}")
    finally:
        if shm is not None:
            try:
                shm.close()
            except Exception:
                pass


def run_dashboard(port):
    """Run Flask Web server dashboard."""
    from flask import Flask, render_template, jsonify, request

    app = Flask(__name__, template_folder="templates")

    @app.route("/")
    def index():
        """
        Renders the main dashboard index page.
        """
        return render_template("shm_dashboard.html")

    @app.route("/api/shm/list")
    def shm_list():
        """
        Lists all defined shared memory segments, status, sizes, and mock states.
        """
        segments = []
        for name, cfg in SHM_REGISTRY.items():
            active = False
            size = 0
            try:
                shm = shared_memory.SharedMemory(name=name)
                safe_unregister_shm(shm)
                active = True
                size = shm.size
                shm.close()
            except FileNotFoundError:
                pass

            segments.append({
                "name": name,
                "active": active,
                "size_bytes": size,
                "magic": cfg["magic"].decode("utf-8", errors="ignore"),
                "header_format": cfg["header_format"],
                "record_format": cfg["record_format"],
                "fields": cfg["fields"],
                "max_samples": cfg["max_samples"],
                "mocking": mock_active.get(name, False)
            })
        return jsonify({"segments": segments})

    @app.route("/api/shm/data/<name>")
    def shm_data(name):
        """
        Retrieves samples and hex dumps for a specific shared memory segment.
        """
        cfg = SHM_REGISTRY.get(name)
        if not cfg:
            return jsonify({"error": "Unknown shared memory name"}), 404

        active = False
        try:
            shm = shared_memory.SharedMemory(name=name)
            safe_unregister_shm(shm)
            active = True
            shm.close()
        except FileNotFoundError:
            pass

        samples = get_shm_samples(name) if active else []
        hex_dump = get_shm_hex_dump(name) if active else []

        return jsonify({
            "name": name,
            "active": active,
            "fields": cfg["fields"],
            "samples": samples,
            "raw_hex": hex_dump
        })

    @app.route("/api/shm/unlink/<name>", methods=["POST"])
    def shm_unlink(name):
        """
        Closes and unlinks a shared memory segment from the operating system.
        """
        # Stop mocking first
        if name in mock_active:
            mock_active[name] = False
            if name in mock_threads:
                mock_threads[name].join(timeout=1.0)

        try:
            shm = shared_memory.SharedMemory(name=name)
            safe_unregister_shm(shm)
            shm.close()
            shm.unlink()
            return jsonify({"success": True, "message": f"Successfully unlinked {name}"})
        except FileNotFoundError:
            return jsonify({"error": "Shared memory segment not found"}), 404
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route("/api/shm/create/<name>", methods=["POST"])
    def shm_create(name):
        """
        Creates and initializes a new shared memory segment using the schema registry.
        """
        cfg = SHM_REGISTRY.get(name)
        if not cfg:
            return jsonify({"error": "Unknown schema"}), 404

        header_size = struct.calcsize(cfg["header_format"])
        record_size = struct.calcsize(cfg["record_format"])
        shm_size = header_size + (cfg["max_samples"] * record_size)

        try:
            shm = shared_memory.SharedMemory(name=name, create=True, size=shm_size)
            safe_unregister_shm(shm)
            struct.pack_into(cfg["header_format"], shm.buf, 0, cfg["magic"], 0, 0)
            shm.close()
            return jsonify({"success": True, "message": f"Successfully initialized {name}"})
        except FileExistsError:
            return jsonify({"message": f"{name} already exists"}), 200
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route("/api/shm/mock/<name>", methods=["POST"])
    def shm_mock(name):
        """
        Starts or stops a background mock generator thread for a shared memory segment.
        """
        cfg = SHM_REGISTRY.get(name)
        if not cfg:
            return jsonify({"error": "Unknown schema"}), 404

        req_data = request.get_json() or {}
        action = req_data.get("action", "start")

        if action == "start":
            frequency = float(req_data.get("frequency", 10.0))
            noise = float(req_data.get("noise", 0.05))
            mode = req_data.get("mode", "sine")

            # Stop existing mock if running
            if mock_active.get(name, False):
                mock_active[name] = False
                if name in mock_threads:
                    mock_threads[name].join()

            mock_active[name] = True
            t = threading.Thread(
                target=mock_worker,
                args=(name, frequency, noise, mode),
                daemon=True
            )
            mock_threads[name] = t
            t.start()
            return jsonify({"success": True, "mocking": True})

        else:
            mock_active[name] = False
            if name in mock_threads:
                mock_threads[name].join(timeout=1.0)
            return jsonify({"success": True, "mocking": False})

    @app.route("/api/opticalflow/reset", methods=["POST"])
    def reset_opticalflow():
        """
        Sends a reset command via loopback UDP to the optical flow stream process.
        """
        import socket
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(b"reset", ("127.0.0.1", 5009))
            return jsonify({"success": True, "message": "Reset command sent to optical flow service"})
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    @app.route("/api/opticalflow/offset", methods=["GET", "POST"])
    def opticalflow_offset():
        """
        Gets or sets the camera tilt offset settings, updating both the config file
        and sending updates via UDP.
        """
        import json
        import os
        config_file = "tilt_calibration.json"
        if request.method == "POST":
            try:
                data = request.get_json() or {}
                ox = float(data.get("offset_x", 0.0))
                oy = float(data.get("offset_y", 0.0))
                
                # Send command via loopback UDP to optical_flow_stream
                import socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(f"offset {ox} {oy}".encode("utf-8"), ("127.0.0.1", 5009))
                
                return jsonify({"success": True, "offset_x": ox, "offset_y": oy})
            except Exception as e:
                return jsonify({"error": str(e)}), 500
        else:
            ox, oy = 0.0, 0.0
            if os.path.exists(config_file):
                try:
                    with open(config_file, "r") as f:
                        cal = json.load(f)
                        ox = cal.get("camera_offset_x", 0.0)
                        oy = cal.get("camera_offset_y", 0.0)
                except Exception:
                    pass
            return jsonify({"offset_x": ox, "offset_y": oy})

    @app.route("/api/shm/config", methods=["GET", "POST"])


    def shm_config():
        """
        Gets or updates the web dashboard configuration settings JSON file.
        """
        import json
        import os
        config_path = "dashboard_config.json"
        if request.method == "POST":
            try:
                data = request.get_json() or {}
                with open(config_path, "w") as f:
                    json.dump(data, f, indent=4)
                return jsonify({"success": True})
            except Exception as e:
                return jsonify({"error": str(e)}), 500
        else:
            if os.path.exists(config_path):
                try:
                    with open(config_path, "r") as f:
                        data = json.load(f)
                    return jsonify(data)
                except Exception as e:
                    return jsonify({})
            return jsonify({})

    print(f"\n" + "=" * 60)
    print(f"Shared Memory Management Dashboard")
    print(f"=" * 60)
    print(f"Starting dashboard on http://localhost:{port}")
    print(f"Open your browser to visualize/mock shared memories")
    print(f"=" * 60 + "\n")

    app.run(host="0.0.0.0", port=port, debug=False, threaded=True)


def main():
    """
    Main entry point for reading shared memory segments.
    Parses CLI arguments to support either continuous standard output stream reading,
    printing the latest sample once, or launching the Flask management dashboard.
    """
    parser = argparse.ArgumentParser(description="Read or manage compass heading / other shared memories.")
    parser.add_argument("--once", action="store_true", help="Print one sample and exit.")
    parser.add_argument("--interval", type=float, default=0.05, help="Polling interval in seconds.")
    parser.add_argument("--wait-interval", type=float, default=0.5, help="Retry interval while waiting for shared memory.")
    parser.add_argument("--freshness-threshold", type=float, default=DEFAULT_FRESHNESS_THRESHOLD, help="Maximum age in seconds for a sample to be considered live.")
    parser.add_argument("--dashboard", action="store_true", help="Start the Web GUI dashboard.")
    parser.add_argument("--port", type=int, default=5003, help="Web dashboard port (default 5003).")
    args = parser.parse_args()

    if args.dashboard:
        try:
            run_dashboard(args.port)
        except KeyboardInterrupt:
            print("\nDashboard stopped.")
        finally:
            # Clean up all mocks
            for k in list(mock_active.keys()):
                mock_active[k] = False
            for t in mock_threads.values():
                t.join(timeout=1.0)
        return

    # Default CLI consumer mode
    shm = None

    try:
        if args.once:
            while True:
                shm = open_shared_memory(args.wait_interval)
                try:
                    sample = read_latest_sample(shm)
                    if sample_is_fresh(sample, args.freshness_threshold):
                        print(format_sample(sample))
                        return
                finally:
                    release_shared_memory(shm)
                    shm = None

                time.sleep(args.interval)

        last_seen = None
        while True:
            if shm is None:
                shm = open_shared_memory(args.wait_interval)
                last_seen = None

            try:
                sample = read_latest_sample(shm)
                if not sample_is_fresh(sample, args.freshness_threshold):
                    last_seen = None
                    release_shared_memory(shm)
                    shm = None
                    time.sleep(args.wait_interval)
                    continue

                if sample is not None:
                    snapshot = (
                        sample["timestamp"],
                        sample["raw_heading"],
                        sample["heading"],
                        sample["x"],
                        sample["y"],
                        sample["z"],
                        sample["sample_count"],
                    )
                    if snapshot != last_seen:
                        print(format_sample(sample), flush=True)
                        last_seen = snapshot

                time.sleep(max(0.01, args.interval))
            except (FileNotFoundError, OSError):
                last_seen = None
                release_shared_memory(shm)
                shm = None
    except KeyboardInterrupt:
        pass
    finally:
        if shm is not None:
            release_shared_memory(shm)


if __name__ == "__main__":
    main()
