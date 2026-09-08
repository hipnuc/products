"""Send product-supported ASCII commands in order; stop if a command fails."""

import logging
import sys

from hipnuc import HipnucError, SerialDevice

PORT = None  # Set both values to connect directly, e.g. "COM3" and 115200.
BAUDRATE = None
TIMEOUT = 2.0  # Seconds to wait for each command response.
COMMANDS = [
    "LOG VERSION",
    "LOG COMCONFIG",
]


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with SerialDevice(PORT, BAUDRATE, timeout=TIMEOUT) as device:
        print(f"Connected to {device.port} at {device.baudrate} baud.")
        for command in COMMANDS:
            print(f"> {command}")
            result = device.command(command)
            print(result.text)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except (HipnucError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
