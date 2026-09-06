"""Benchmark the checked-out decoder using fixed, synthetic protocol fixtures.

Run with the SDK dependencies installed, from any working directory:
    python /path/to/python/tools/benchmark_decoder.py --frames 10000
    python /path/to/python/tools/benchmark_decoder.py --frames 10000 --tracemalloc

Frames cycle through the shared golden fixtures, including the explicit
partial HI83 case. Input preparation and imports are excluded from timing.
Optional tracemalloc reports peak traced Python allocations during decoding,
excluding the prebuilt input bytes; it also slows execution. No port is opened
and no hardware throughput or platform-independent speed threshold is implied.
"""

from __future__ import annotations

import argparse
import binascii
import json
from pathlib import Path
import platform
import sys
import time
import tracemalloc


def positive_count(value: str) -> int:
    count = int(value)
    if count < 1:
        raise argparse.ArgumentTypeError("frames must be a positive integer")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--frames",
        type=positive_count,
        default=10000,
        help="total frames, cycling through the fixed fixtures (default: 10000)",
    )
    parser.add_argument(
        "--tracemalloc",
        action="store_true",
        help="measure peak Python allocations; this slows the benchmark",
    )
    args = parser.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    # Always measure this checkout, even if another SDK version is installed.
    sys.path.insert(0, str(package_root / "src"))
    from hipnuc import Decoder

    fixture_path = package_root / "tests" / "fixtures" / "hipnuc_protocol.json"
    fixtures = json.loads(fixture_path.read_text(encoding="utf-8"))["frames"]
    frames = [bytes.fromhex(item["raw_hex"]) for item in fixtures]
    for item, raw in zip(fixtures, frames):
        if len(raw) != item["payload_length"] + 6 or binascii.crc_hqx(
            raw[:4] + raw[6:], 0
        ) != int.from_bytes(raw[4:6], "little"):
            raise ValueError(f"Invalid golden fixture: {item['name']}")

    # Reuse bounded input batches; --frames does not allocate a full recording.
    cycle = b"".join(frames)
    batch = cycle * 32
    cycles, extra_frames = divmod(args.frames, len(frames))
    batches, extra_cycles = divmod(cycles, 32)
    tail = cycle * extra_cycles + b"".join(frames[:extra_frames])
    input_bytes = batches * len(batch) + len(tail)

    if args.tracemalloc:
        tracemalloc.start()
    decoder = Decoder()
    samples = 0
    started = time.perf_counter()
    for _ in range(batches):
        samples += len(decoder.feed(batch))
    if tail:
        samples += len(decoder.feed(tail))
    elapsed = time.perf_counter() - started
    peak_memory = tracemalloc.get_traced_memory()[1] if args.tracemalloc else None
    if args.tracemalloc:
        tracemalloc.stop()
    if samples != args.frames or decoder.buffered_bytes:
        raise RuntimeError("Decoder lost fixture samples or retained an incomplete frame")

    print(
        json.dumps(
            {
                "python": platform.python_version(),
                "python_implementation": platform.python_implementation(),
                "os": platform.platform(),
                "architecture": platform.machine(),
                "fixture_names": [item["name"] for item in fixtures],
                "frames": args.frames,
                "input_bytes": input_bytes,
                "samples": samples,
                "elapsed_s": elapsed,
                "samples_per_s": samples / elapsed,
                "bytes_per_s": input_bytes / elapsed,
                "tracemalloc_enabled": args.tracemalloc,
                "peak_memory_bytes": peak_memory,
                "buffered_bytes": decoder.buffered_bytes,
            },
            indent=2,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
