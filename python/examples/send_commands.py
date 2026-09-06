"""Send product-supported ASCII commands in order; stop if a command fails."""

import logging

from hipnuc import SerialDevice

PORT = None  # Set both values to connect directly, e.g. "COM3" and 115200.
BAUDRATE = None
COMMANDS = [
    "LOG VERSION",
    "LOG COMCONFIG",
]


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with SerialDevice(PORT, BAUDRATE) as device:
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
