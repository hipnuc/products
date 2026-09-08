# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

Read, configure and record HiPNUC IMU/AHRS/MRU and INS devices from a terminal
or your Python application. Supports HI91/HI81/HI83, NMEA GGA/RMC and Modbus RTU
with **Python 3.10–3.14** on Windows, Linux (including Ubuntu and Raspberry Pi OS),
and macOS.

Supported devices: firmware 1.6.9 or later (HI01–HI06, HI12–HI18, HI32,
HI70/HI71, CH0X0). Legacy HI2xx/CH1xx products use the archived C examples.

## Install

Extract the repository and open a terminal in its `python/` directory.

**Windows PowerShell:**

```powershell
py -3 --version
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install .
python -m hipnuc --help
```

**Linux / Raspberry Pi / macOS:**

On Ubuntu/Debian/Pi OS, install missing venv support with
`sudo apt update && sudo apt install -y python3-venv`.

```sh
python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
python -m hipnuc --help
```

Activate the environment again in each new terminal. For an existing application,
activate its environment and run `python -m pip install "/path/to/products/python"`;
select that same environment in your IDE. Reinstall after updating SDK source.

## Read, record and send commands

Connect the device and close other programs using its port. In a VM, attach the
USB adapter to the guest OS.

```sh
python -m hipnuc list
python -m hipnuc read
python -m hipnuc read --duration 60 --record samples.jsonl
```

Press Ctrl-C to stop a continuous read before trying the next command.
`list` shows USB serial ports and an other-port count; `list --all` expands all
ports and `list --json` always returns all ports. Automatic discovery searches
USB serial ports, showing progress for up to 30 seconds. For multiple devices,
built-in/GPIO UARTs or other ports, select `-p PORT`; add `-b BAUD` if known:

```sh
python -m hipnuc read -p COM3 -b 115200
python -m hipnuc info -p COM3 -b 115200
python -m hipnuc command "LOG VERSION" -p COM3 -b 115200
python -m hipnuc command --file commands.txt -p COM3 -b 115200 --save
```

Replace `COM3` with your port, for example `/dev/ttyUSB0` on Linux.
Connection options follow the **final command**, including `modbus read`.
`-b` sets host connection speed only; `baudrate NEW_BAUD` changes the device.
Use `scan -p PORT` for an unknown baudrate, `reboot` to restart, and append
`--help` for options. Configuration commands require an explicit port.

Command files contain one product command per line; `#` and `;` start comments.
Execution stops on failure. `--save` saves once after all commands succeed;
add `--reboot` only when the settings require it. Use the managed `baudrate`
and `reboot` commands for connection changes. Command names and applicability
come from the product's command and programming manual.

Recording keeps all decoded samples independently of the display's five
readings/second limit per message type. Add `--record-raw capture.bin` for
received bytes, `--quiet` to hide readings or `--jsonl` for machine output.
Files use the current directory; existing files are protected unless
`--overwrite` is explicit. Finite reads finish the current receive batch.
Diagnostics go to stderr. Exit codes: 0 success, 1 failure, 2 usage error, 130 Ctrl-C.

## Python API and examples

```python
from hipnuc import SerialDevice

with SerialDevice() as device:
    for sample in device.iter_samples():
        print(sample.acceleration_m_s2, sample.angular_velocity_rad_s)
```

Importing and constructing objects perform no I/O. Calls are synchronous;
use `with` to open and close resources. `SerialDevice("COM3", baudrate=115200)`
selects a known connection; omitted connection parameters use discovery.

| Interface | Purpose |
| --- | --- |
| `SerialDevice(..., timeout=2.0)` | Serial connection; timeouts are in seconds |
| `device.read(timeout=None)`, `device.iter_samples(idle_timeout=None)` | Read new samples; idle expiry raises `ResponseTimeout`, defaulting to device timeout |
| `device.read_info()` | Product, firmware and serial number |
| `device.command("LOG VERSION").text` | Send ASCII and receive the reply; an unsupported command may time out |
| `device.save_config()`, `device.set_baudrate(...)`, `device.reboot()` | Explicit save and managed connection changes |
| `Decoder().feed(data)` | Incremental byte decoding without a device; reuse one decoder, call `finish()` at end of input |
| `sample.values`, `sample.to_dict()` | Protocol-specific fields and JSON-compatible output |

Samples represent individual messages, not merged history. The API/JSON use
m/s², rad/s, rad, tesla and Pa; latitude/longitude are degrees and temperature is
°C. The human display uses degrees and degrees/s. Missing values remain `None`.
`quaternion_wxyz` is WXYZ, body-to-navigation; `euler_rad` follows the device's
configured convention. Heading is clockwise from north, distinct from Euler yaw.
The SDK does not change the device's coordinate configuration.
`received_time_ns` is host reception time; device time and UTC are separate fields.
`complete`, `issues` and `metadata` retain parsing and source information.

Communication errors derive from `HipnucError`: `TransportError`,
`ResponseTimeout`, `DeviceError` and `VerificationError` (readback mismatch).
Invalid arguments raise `ValueError`; recording I/O failures raise `OSError`.

For recording, connect before creating files, then attach callbacks before reading:

```python
from hipnuc import Recorder, SerialDevice

with (
    SerialDevice("COM3", baudrate=115200) as device,
    Recorder("samples.jsonl", raw_path="capture.bin") as recording,
):
    device.sample_sink = recording.write
    device.raw_sink = recording.write_raw
    for sample in device.iter_samples():
        pass
```

`Recorder` writes JSONL and optional exact received bytes; use `write_raw` on
received chunks, not `sample.raw`. Callbacks run synchronously; keep them short
and do not call device I/O from them. Files flush every second while writing
and on close; `flush()`, `samples_written` and `raw_bytes_written` are available.
Discovery traffic precedes recording. The recording example also finishes the
current receive batch on Ctrl-C.

Edit constants at the top of these scripts, then run `python examples/read_samples.py`
(or the chosen filename). They do not take CLI arguments.

| Example | Use |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | Read measurements; `PORT = None`, `BAUDRATE = None` discover a USB connection |
| [record_samples.py](examples/record_samples.py) | JSONL recording; set `RAW_PATH` for original bytes |
| [send_commands.py](examples/send_commands.py) | Edit `COMMANDS`; defaults query identity/configuration without saving |
| [modbus_multinode.py](examples/modbus_multinode.py) | One bus, several node IDs, sequential polling |

## Modbus RTU

Use an explicit port and node ID. Defaults are 115200 baud, 8N1 and ID 80;
unicast IDs are 1–247. Prepare each device's RTU port, output mode, baudrate and
unique ID before joining a bus. Never run ASCII `scan` or `command` on a multi-node
Modbus bus. If streaming must be disabled, send `LOG DISABLE` in a single-device
ASCII session first.
`LOG DISABLE` is temporary; to keep RTU operation after reboot, disable the
timed output messages and save that configuration explicitly.

```sh
python -m hipnuc modbus info -p COM3 --id 80
python -m hipnuc modbus read -p COM3 --id 80
python -m hipnuc modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
```

`read` continues until Ctrl-C, `--duration` or `--count`; `--interval` controls
the pause between polls. Register addresses, values and availability depend on
the model's command and programming manual. Use `registers` / `write-register`
for raw access, and `set-id`, `baudrate`, `reboot` for managed changes;
append `--help` to the final command.

```python
from hipnuc import ModbusBus

with ModbusBus("COM3") as bus:
    device = bus.device(80)
    print(device.read_info())
    sample = device.read_sample()
    print(sample.acceleration_m_s2)
```

One `ModbusBus` owns a physical port; nodes share its transaction lock.
FC03 reads 1–125 registers; FC06 writes one 16-bit register. Wide values are
big-endian, high word first. Writes are not retried and are read back by default;
saving is explicit (`--save` or `save_config()`). The SDK does not promise that
all registers in a measurement block come from the same firmware cycle.
JSONL records keep `metadata.device_id`; Modbus raw-bus recording is not provided.
For Python recording, open the bus before `Recorder` and write each returned sample.
See [modbus_multinode.py](examples/modbus_multinode.py) for multi-device polling.

## Common issues

| Symptom | Next step |
| --- | --- |
| Missing Python / venv / pip | Use Python 3.10+. Ubuntu 22.04's default 3.10 works; 20.04's default 3.8 does not. On Ubuntu/Debian/Pi OS, install `python3-venv`. |
| APT waits for a lock | Wait for the OS updater; do not delete its lock or kill the updater. |
| `UNKNOWN-0.0.0` / `No module named hipnuc` | Activate the correct environment, upgrade pip, then reinstall from this `python/` directory. |
| PowerShell blocks activation | Use `.\.venv\Scripts\python.exe` instead of `python`; no global execution-policy change is needed. |
| No port / busy port | Check USB passthrough in a VM, cable/driver and other serial applications. Use `list --all` for non-USB ports. |
| Port opens, no valid samples | Check actual baudrate, wiring, output mode and rate. Give slow output enough `--timeout`. |
| Download / certificate error | Check network, clock and the required proxy/certificate settings. Do not disable TLS verification. |

For HGFS or another shared folder that cannot create venv symlinks, keep the
environment on local storage; the source may remain shared if complete and writable:

```sh
python3 -m venv "$HOME/.venvs/hipnuc"
source "$HOME/.venvs/hipnuc/bin/activate"
python -m pip install --upgrade pip
python -m pip install .
```

Activate this external environment with the same `source` command in new terminals.
For Linux serial permissions, check `ls -l /dev/ttyUSB0` and `id -nG`. If the
device group is `dialout`, run `sudo usermod -a -G dialout "$(id -un)"`, then log
out and back in or reboot. A virtual environment does not grant serial permissions.
