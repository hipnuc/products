"""Print IMU or INS samples using the installed SDK."""

import logging
import sys

from hipnuc import HipnucError, SerialDevice

PORT = None  # Set both values to connect directly, e.g. "COM3" and 115200.
BAUDRATE = None
TIMEOUT = 2.0  # Seconds; increase for devices that output less often.


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    with SerialDevice(PORT, BAUDRATE, timeout=TIMEOUT) as device:
        print(f"Connected to {device.port} at {device.baudrate} baud.")
        for sample in device.iter_samples():
            print(
                sample.type,
                "acceleration (m/s^2):",
                sample.acceleration_m_s2,
                "angular velocity (rad/s):",
                sample.angular_velocity_rad_s,
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except (HipnucError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
