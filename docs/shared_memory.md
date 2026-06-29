# Shared Memory Specifications (IPC)

This document details the design, memory layouts, and programming interface of the Inter-Process Communication (IPC) system used to stream sensor values and state information between companion computer processes.

---

## 1. Design Overview

To exchange telemetry at high frequencies (up to 100 Hz) without the overhead of network sockets or the latency of OS pipes, the codebase uses POSIX Shared Memory segments mapped through Python's `multiprocessing.shared_memory` API.

### Key Architectural Characteristics
* **Zero-Copy Performance**: Processes read and write directly to the same raw virtual address space.
* **Lock-Free Reading**: Data is structured as a circular ring buffer with a single-writer, multiple-reader pattern. Because updates are sequential and writes happen atomically at discrete block alignments, readers can fetch the newest telemetry sample without acquiring mutexes.
* **Resource Tracker Isolation**: Python's built-in `multiprocessing.resource_tracker` normally tracks shared memory segments and deletes them from the system when the creating process exits. To allow persistent streams to survive process reboots, the codebase unregisters segments using:
  ```python
  from multiprocessing import resource_tracker
  resource_tracker.unregister(shm._name, "shared_memory")
  ```

---

## 2. Memory Struct Layouts

Every shared memory segment consists of a **Header** followed by a pre-allocated **Circular Buffer** of records. All values are packed using Python's `struct` library with little-endian (`<`) byte ordering.

```
+-------------------------------------------------------------------+
|                            HEADER                                 |
|  - Magic ID (char[4])                                             |
|  - Write Index (uint32)                                            |
|  - Sample Count (uint32)                                          |
+-------------------------------------------------------------------+
|                        CIRCULAR BUFFER                            |
|  +-------------------------------------------------------------+  |
|  | Record 0 (Double Array)                                     |  |
|  +-------------------------------------------------------------+  |
|  | Record 1 (Double Array)                                     |  |
|  +-------------------------------------------------------------+  |
|  | ...                                                         |  |
|  +-------------------------------------------------------------+  |
|  | Record N-1 (Double Array)                                   |  |
|  +-------------------------------------------------------------+  |
+-------------------------------------------------------------------+
```

---

## 3. Segment Registries

The system registers three memory segments, each configured with specific layouts and capacities:

### A. Compass Heading Segment (`compass_heading_stream`)
* **Segment Name:** `compass_heading_stream`
* **Identifier Magic:** `b"CHDG"` (4 bytes)
* **Header Format:** `<4sII` (12 bytes)
  * `magic`: `char[4]` (Offset: 0)
  * `write_index`: `uint32` (Offset: 4)
  * `sample_count`: `uint32` (Offset: 8)
* **Record Format:** `<6d` (48 bytes)
* **Record Offset Mapping:**
  | Variable | Index | Format | Size (Bytes) | Relative Byte Offset | Description |
  | :--- | :--- | :--- | :--- | :--- | :--- |
  | `timestamp` | 0 | `d` (float64) | 8 | 0 | System epoch timestamp |
  | `raw_heading`| 1 | `d` (float64) | 8 | 8 | Unfiltered heading in degrees |
  | `heading` | 2 | `d` (float64) | 8 | 16 | Smoothed heading in degrees |
  | `x` | 3 | `d` (float64) | 8 | 24 | Magnetometer X axis reading |
  | `y` | 4 | `d` (float64) | 8 | 32 | Magnetometer Y axis reading |
  | `z` | 5 | `d` (float64) | 8 | 40 | Magnetometer Z axis reading |
* **Buffer Capacity:** 120 samples
* **Total Segment Size:** $12 + (120 \times 48) = 5,772 \text{ bytes}$

### B. Optical Flow Segment (`optical_flow_stream`)
* **Segment Name:** `optical_flow_stream`
* **Identifier Magic:** `b"FLOW"` (4 bytes)
* **Header Format:** `<4sII` (12 bytes)
* **Record Format:** `<11d` (88 bytes)
* **Record Offset Mapping:**
  | Variable | Index | Format | Size (Bytes) | Relative Byte Offset | Description |
  | :--- | :--- | :--- | :--- | :--- | :--- |
  | `timestamp` | 0 | `d` (float64) | 8 | 0 | System performance counter timestamp |
  | `x_cm` | 1 | `d` (float64) | 8 | 8 | Compensated X position displacement in cm |
  | `y_cm` | 2 | `d` (float64) | 8 | 16 | Compensated Y position displacement in cm |
  | `x_raw_cm` | 3 | `d` (float64) | 8 | 24 | Raw X position displacement in cm |
  | `y_raw_cm` | 4 | `d` (float64) | 8 | 32 | Raw Y position displacement in cm |
  | `vx` | 5 | `d` (float64) | 8 | 40 | Fused body-frame X velocity in m/s |
  | `vy` | 6 | `d` (float64) | 8 | 48 | Fused body-frame Y velocity in m/s |
  | `vx_raw` | 7 | `d` (float64) | 8 | 56 | Raw body-frame X velocity in m/s |
  | `vy_raw` | 8 | `d` (float64) | 8 | 64 | Raw body-frame Y velocity in m/s |
  | `alt` | 9 | `d` (float64) | 8 | 72 | Rangefinder altitude in meters |
  | `heading` | 10| `d` (float64) | 8 | 80 | Fused yaw heading in degrees |
* **Buffer Capacity:** 120 samples
* **Total Segment Size:** $12 + (120 \times 88) = 10,572 \text{ bytes}$

### C. Future Sensor Segment (`future_sensor_stream`)
* **Segment Name:** `future_sensor_stream`
* **Identifier Magic:** `b"FUTR"` (4 bytes)
* **Header Format:** `<4sII` (12 bytes)
* **Record Format:** `<4d` (32 bytes)
  * Fields: `timestamp` (float64), `temperature` (float64), `pressure` (float64), `altitude` (float64).
* **Buffer Capacity:** 100 samples
* **Total Segment Size:** $12 + (100 \times 32) = 3,212 \text{ bytes}$

---

## 4. Code Implementation Examples

### Writing to Shared Memory (`optical_flow_stream.py`)
Below is the exact writing implementation used by the optical flow pipeline. It attaches to the shared memory block, retrieves the write indexes, updates the circular array slot, and increments the header indexes atomically:

```python
import struct
from multiprocessing import shared_memory

SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<11d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120

def write_flow_stream_sample(timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading):
    try:
        # Create or attach to shared memory
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
        
        # Read header
        _, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
        
        # Write record at write_index offset
        record_offset = SHM_HEADER_SIZE + (write_index * SHM_RECORD_SIZE)
        struct.pack_into(
            SHM_RECORD_FORMAT,
            shm.buf,
            record_offset,
            float(timestamp), float(x_cm), float(y_cm),
            float(x_raw_cm), float(y_raw_cm),
            float(vx), float(vy), float(vx_raw), float(vy_raw),
            float(alt), float(heading),
        )
        
        # Increment index
        write_index = (write_index + 1) % MAX_SAMPLES
        sample_count = min(sample_count + 1, MAX_SAMPLES)
        
        # Update header
        struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)
    except Exception as exc:
        print(f"Failed to write to shared memory: {exc}")
```

### Reading from Shared Memory (`static_gps_injector.py`)
Below is the implementation logic used by the GPS injector process to consume the latest telemetry values without thread locking:

```python
import struct
from multiprocessing import shared_memory

def get_latest_flow_data():
    try:
        shm = shared_memory.SharedMemory(name="optical_flow_stream")
        
        # Unpack header values
        magic, write_index, sample_count = struct.unpack_from("<4sII", shm.buf, 0)
        if magic != b"FLOW" or sample_count == 0:
            return None
        
        # Calculate index offset of the last written sample
        latest_index = (write_index - 1) % 120
        offset = 12 + (latest_index * 88)
        
        # Unpack the 11 doubles
        values = struct.unpack_from("<11d", shm.buf, offset)
        x_cm, y_cm = values[1], values[2]
        alt = values[9]
        
        return x_cm / 100.0, y_cm / 100.0, alt # returns metric coordinates and altitude
    except Exception:
        return None
```
