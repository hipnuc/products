"""Record every decoded sample and optionally the original received bytes."""

import logging
import signal

from hipnuc import Recorder, ResponseTimeout, SerialDevice

PORT = None  # Set both values to connect directly, e.g. "COM3" and 115200.
BAUDRATE = None
JSONL_PATH = "samples.jsonl"  # Existing files are never overwritten.
RAW_PATH = None  # Set to "capture.bin" to also record original received bytes.


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    stopped = False

    def stop_recording(_signum, _frame):
        nonlocal stopped
        stopped = True

    with Recorder(JSONL_PATH, raw_path=RAW_PATH) as recording:
        with SerialDevice(
            PORT,
            BAUDRATE,
            sample_sink=recording.write,
            raw_sink=recording.write_raw if RAW_PATH else None,
        ) as device:
            print(f"Recording {device.port} at {device.baudrate} baud to {JSONL_PATH or RAW_PATH}")
            # During discovery Ctrl-C still cancels immediately. Once connected,
            # finish the current batch of callbacks before closing the files.
            previous_handler = signal.signal(signal.SIGINT, stop_recording)
            try:
                while not stopped:
                    try:
                        device.read()
                    except ResponseTimeout:
                        if not stopped:
                            raise
            finally:
                signal.signal(signal.SIGINT, previous_handler)
    if stopped:
        raise KeyboardInterrupt


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
