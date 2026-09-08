"""Poll existing, uniquely addressed nodes on one physical RS-485 bus."""

import json
import sys
import time

from hipnuc import HipnucError, ModbusBus

PORT = "COM3"  # Linux example: "/dev/ttyUSB0".
BAUDRATE = 115200
TIMEOUT = 2.0  # Seconds to wait for a response, plus the wire transfer time.
NODE_IDS = [80, 81]  # Configure a unique address on each device before connecting the bus.
INTERVAL_S = 0.1


def main() -> None:
    with ModbusBus(PORT, BAUDRATE, timeout=TIMEOUT) as bus:
        devices = [bus.device(node_id) for node_id in NODE_IDS]
        while True:
            for device in devices:
                sample = device.read_sample()
                print(json.dumps(sample.to_dict(), allow_nan=False))
            time.sleep(INTERVAL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130) from None
    except (HipnucError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
