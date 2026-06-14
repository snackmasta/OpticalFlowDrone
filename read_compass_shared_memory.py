#!/usr/bin/env python3
"""Read the live compass heading stream from shared memory."""

import argparse
import struct
import time
from multiprocessing import shared_memory
from multiprocessing import resource_tracker


SHM_NAME = "compass_heading_stream"
SHM_MAGIC = b"CHDG"
SHM_HEADER_FORMAT = "<4sII"
SHM_RECORD_FORMAT = "<6d"
SHM_HEADER_SIZE = struct.calcsize(SHM_HEADER_FORMAT)
SHM_RECORD_SIZE = struct.calcsize(SHM_RECORD_FORMAT)
MAX_SAMPLES = 120
DEFAULT_FRESHNESS_THRESHOLD = 0.75


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


def release_shared_memory(shm):
    try:
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception:
        pass

    try:
        shm.close()
    except Exception:
        pass


def sample_age_seconds(sample_timestamp):
    return time.time() - sample_timestamp


def sample_is_fresh(sample, freshness_threshold):
    return sample is not None and sample_age_seconds(sample["timestamp"]) <= freshness_threshold


def format_sample(sample):
    return f"{sample['heading']:.2f}"


def main():
    parser = argparse.ArgumentParser(description="Read compass samples from shared memory.")
    parser.add_argument("--once", action="store_true", help="Print one sample and exit.")
    parser.add_argument("--interval", type=float, default=0.05, help="Polling interval in seconds.")
    parser.add_argument("--wait-interval", type=float, default=0.5, help="Retry interval while waiting for shared memory.")
    parser.add_argument("--freshness-threshold", type=float, default=DEFAULT_FRESHNESS_THRESHOLD, help="Maximum age in seconds for a sample to be considered live.")
    args = parser.parse_args()

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
