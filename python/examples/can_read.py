"""Print a node's identity and CAN measurements through any python-can bus."""

import sys

from hipnuc import HipnucError, TransportError
from hipnuc.can import decode_message, read_info

# Linux SocketCAN: set the interface bitrate with ip link first.
BUS = {"interface": "socketcan", "channel": "can0"}
# Windows PCAN: BUS = {"interface": "pcan", "channel": "PCAN_USBBUS1", "bitrate": 500000}
NODE_ID = 8


def main() -> None:
    import can  # The optional extra: python -m pip install ".[can]"

    try:
        bus = can.Bus(ignore_config=True, **BUS)
    except can.CanError as exc:
        raise TransportError(f"Cannot open the CAN bus: {exc}") from exc
    with bus:
        print(read_info(bus, NODE_ID))
        for message in bus:
            try:
                sample = decode_message(message)
            except ValueError:
                continue  # A malformed frame with a HiPNUC PGN; keep reading.
            if sample is not None and sample.values.get("node_id") == NODE_ID:
                print(sample.type, sample.values)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except (HipnucError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
