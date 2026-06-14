#!/usr/bin/env python3
"""Read the live compass heading stream from shared memory."""

import argparse
import struct
import time
from multiprocessing import shared_memory


SHM_NAME = "compass_heading_stream"
SHM_MAGIC = b"CHDG"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<6d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120


def read_latest_sample(shm):
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
    while True:
        try:
            return shared_memory.SharedMemory(name=SHM_NAME)
        except FileNotFoundError:
            time.sleep(wait_interval)


def format_sample(sample):
    return f"{sample['heading']:.2f}"


def main():
    parser = argparse.ArgumentParser(description="Read compass samples from shared memory.")
    parser.add_argument("--once", action="store_true", help="Print one sample and exit.")
    parser.add_argument("--interval", type=float, default=0.05, help="Polling interval in seconds.")
    parser.add_argument("--wait-interval", type=float, default=0.5, help="Retry interval while waiting for shared memory.")
    args = parser.parse_args()

    shm = None

    try:
        if args.once:
            shm = open_shared_memory(args.wait_interval)
            while True:
                sample = read_latest_sample(shm)
                if sample is not None:
                    print(format_sample(sample))
                    return
                time.sleep(args.interval)

        last_seen = None
        while True:
            if shm is None:
                shm = open_shared_memory(args.wait_interval)
                last_seen = None

            try:
                sample = read_latest_sample(shm)
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
                try:
                    shm.close()
                except Exception:
                    pass
                shm = None
    except KeyboardInterrupt:
        pass
    finally:
        if shm is not None:
            shm.close()


if __name__ == "__main__":
    main()
