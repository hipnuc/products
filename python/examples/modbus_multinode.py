"""Poll existing, uniquely addressed nodes on one physical RS-485 bus."""

import json
import time

from hipnuc import ModbusBus

PORT = "COM3"  # Linux example: "/dev/ttyUSB0".
BAUDRATE = 115200
NODE_IDS = [80, 81]  # Configure a unique address on each device before connecting the bus.
INTERVAL_S = 0.1


def main() -> None:
    with ModbusBus(PORT, BAUDRATE) as bus:
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
