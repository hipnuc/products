# Modbus RTU

[English](modbus.md) | [中文](modbus_zh.md)

Defaults: `115200 / 8N1`, node ID `80`, unicast IDs `1–247`, function codes
FC03 (read) and FC06 (write) only. One `ModbusBus` owns one physical bus; every
node on it shares the port and a transaction lock. Writes are never retried and
no save request is appended automatically.

## Prepare a single device

1. Connect one RS-485/Modbus-capable device and check power, wiring (A/B) and
   its current baudrate and node ID.
2. Check in the product manual which COM port provides Modbus RTU.
3. If that port is streaming serial frames, stop them from a single-device ASCII
   session (`hipnuc command "LOG DISABLE" -p COM3 -b 115200`; make it persistent
   by turning off the timed messages and sending `SAVECONFIG`).
4. Close the ASCII session, then run `hipnuc modbus info -p COM3 --id 80`.
5. Before joining several devices on one bus, give each a unique ID and the same
   baudrate. Never run `scan` or `command` on a multi-node bus.

## Read

```sh
hipnuc modbus info -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80
hipnuc modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
hipnuc modbus read -p COM3 --id 80 --count 10 --interval 0.1 --jsonl
```

`-b` defaults to 115200, `--id` to 80 and `--timeout` to 2 s. `read` polls until
Ctrl-C, `--duration` or `--count`; `--interval` is the pause between polls.

```python
from hipnuc import ModbusBus

with ModbusBus("/dev/ttyUSB0") as bus:
    device = bus.device(80)
    info = device.read_info()
    status = device.read_status()  # main_status, status_flags, calibration
    sample = device.read_sample(include_status=True, include_mru=False)
    words = device.read_registers(0x34, 26)
```

| Block | Registers |
| --- | --- |
| Identity | `0x70–0x82` (19 registers) |
| Main status, calibration status and progress | `0x09–0x0B` |
| IMU/attitude | `0x34–0x4D` (26 registers) |
| With MRU heave/surge/sway | `0x34–0x53` (32 registers) |

Registers are big-endian and zero based; 32-bit values are high word first.
Acceleration uses `9.8 / 2048 m/s²` per count, magnetic field 32.768 counts/µT.
`sample.metadata["register_snapshot"]` stays `not_guaranteed`: the fields of one
block are not promised to come from the same firmware cycle.

## Write registers

```sh
hipnuc modbus registers 0x06 1 -p COM3 --id 80
hipnuc modbus write-register 0x06 1 -p COM3 --id 80 --save --reboot
```

Use the register addresses and values from the product manual. `write_register`
reads the value back by default (`--no-verify` / `verify=False` for write-only
controls). Illegal writes may still be echoed by the device, so only a matching
readback sets `verified=True`. `--save` saves once after a successful write and
`--reboot` restarts and waits for the device.

## Save, change ID, change baudrate

```python
with ModbusBus("COM3") as bus:
    device = bus.device(80)
    device.write_register(0x06, 1)  # heading mode, see the manual
    device.write_register(0xA6, 24)  # installation orientation
    device.save_config()  # once after the batch
    device.reboot()
```

- `set_id(81, save=False)` writes the new ID, rebinds the object and reads the ID
  register back at the new address. The target ID must be free on the bus.
- `set_baudrate(921600, reboot=False, save=False)` writes the baudrate code; the
  device applies it after a reboot. With `reboot=True` the SDK reboots, switches
  the host port to the new speed and waits for the identity block.
- `reboot(timeout=5, save=False, baudrate=None)` sends one reset and waits until
  `read_info()` answers. Pass `baudrate` when a new device speed takes effect.
- `bus.reconfigure(baudrate)` changes only the host speed.

```sh
hipnuc modbus set-id 81 -p COM3 --id 80 --save
hipnuc modbus baudrate 921600 -p COM3 --id 81 --save --reboot
hipnuc modbus info -p COM3 -b 921600 --id 81
```

All nodes on one bus share the baudrate; changing one node does not reconfigure
the others.

## Record from Python

```python
from hipnuc import ModbusBus, Recorder

with Recorder("samples.jsonl") as recording, ModbusBus("COM3") as bus:
    device = bus.device(80)
    for _ in range(100):
        recording.write(device.read_sample())
```

Each record keeps `metadata.device_id`. Polling several nodes is shown in
[modbus_multinode.py](../examples/modbus_multinode.py); edit `PORT`, `BAUDRATE`,
`NODE_IDS` and `INTERVAL_S` at the top of the script and run it from `python/`.

Register addresses and values are documented in the product's command and
programming manual.
