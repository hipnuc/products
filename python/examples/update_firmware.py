"""Update one device over serial, or over CAN, and confirm the new firmware."""

import sys

from hipnuc import HipnucError, TransportError, update_can, update_serial

IMAGE = "firmware.hex"  # The Intel HEX application image made for this product.
PORT = "COM3"  # Serial port, e.g. "/dev/ttyUSB0"; set to None to update over CAN.
CAN_BUS = {"interface": "socketcan", "channel": "can0"}  # Used when PORT is None.
NODE_ID = 8
shown = -1


def progress(written: int, total: int) -> None:
    global shown
    percent = written * 100 // total
    if percent == shown:
        return  # CAN reports every 7 bytes; print each percent once.
    shown = percent
    print(f"\rWriting {percent:3d}%", end="", flush=True)
    if written == total:
        print("\nWaiting for the new firmware to start...")


def main() -> None:
    if PORT:
        print("Keep the device still until the update finishes.")
        result = update_serial(IMAGE, port=PORT, progress=progress)
    else:
        import can  # The optional extra: python -m pip install ".[can]"

        try:
            bus = can.Bus(ignore_config=True, **CAN_BUS)
        except can.CanError as exc:
            raise TransportError(f"Cannot open the CAN bus: {exc}") from exc
        with bus:
            result = update_can(bus, NODE_ID, IMAGE, progress=progress)
    was = f" (was {result.previous_version})" if result.previous_version else ""
    print(f"Running firmware {result.verified_version}{was}.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except (HipnucError, ValueError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
