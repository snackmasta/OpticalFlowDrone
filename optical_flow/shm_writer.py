import struct
import threading
from multiprocessing import shared_memory

SHM_NAME = "optical_flow_stream"
SHM_MAGIC = b"FLOW"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<11d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
SHM_SIZE = SHM_HEADER_SIZE + (MAX_SAMPLES * SHM_RECORD_SIZE)

flow_shm = None
flow_shm_lock = threading.Lock()


def attach_flow_stream_shm():
    """
    Attaches to or creates the shared memory block for optical flow stream data.
    Ensures that the shared memory size is correct and initializes the header.
    """
    global flow_shm
    with flow_shm_lock:
        if flow_shm is not None:
            return flow_shm
        try:
            flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
        except FileExistsError:
            flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=False)
            if flow_shm.size < SHM_SIZE:
                flow_shm.close()
                try:
                    flow_shm.unlink()
                except FileNotFoundError:
                    pass
                flow_shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)

        try:
            from multiprocessing import resource_tracker
            resource_tracker.unregister(flow_shm._name, "shared_memory")
        except Exception:
            pass

        struct.pack_into(SHM_HEADER_FORMAT, flow_shm.buf, 0, SHM_MAGIC, 0, 0)
        return flow_shm


def write_flow_stream_sample(timestamp, x_cm, y_cm, x_raw_cm, y_raw_cm, vx, vy, vx_raw, vy_raw, alt, heading):
    """
    Writes a single optical flow state sample into the circular buffer of the shared memory block.
    """
    try:
        shm = attach_flow_stream_shm()
        with flow_shm_lock:
            _, write_index, sample_count = struct.unpack_from(SHM_HEADER_FORMAT, shm.buf, 0)
            record_offset = SHM_HEADER_SIZE + (write_index * SHM_RECORD_SIZE)
            struct.pack_into(
                SHM_RECORD_FORMAT,
                shm.buf,
                record_offset,
                float(timestamp),
                float(x_cm),
                float(y_cm),
                float(x_raw_cm),
                float(y_raw_cm),
                float(vx),
                float(vy),
                float(vx_raw),
                float(vy_raw),
                float(alt),
                float(heading),
            )
            write_index = (write_index + 1) % MAX_SAMPLES
            sample_count = min(sample_count + 1, MAX_SAMPLES)
            struct.pack_into(SHM_HEADER_FORMAT, shm.buf, 0, SHM_MAGIC, write_index, sample_count)
    except Exception as exc:
        print(f"Failed to write to shared memory: {exc}")


def close_flow_stream_shm():
    """
    Closes and unlinks the shared memory resource when stopping the application.
    """
    global flow_shm
    with flow_shm_lock:
        if flow_shm is None:
            return
        try:
            flow_shm.close()
        finally:
            try:
                flow_shm.unlink()
            except FileNotFoundError:
                pass
            flow_shm = None
