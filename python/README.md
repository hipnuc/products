# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

Read, configure and record HiPNUC IMU/AHRS/MRU and INS devices from a terminal
or your Python application. Supports serial binary/NMEA and Modbus RTU with
Python 3.10–3.14 on Windows, Linux (including Ubuntu and Raspberry Pi OS), and macOS.

## Install

Download the repository and open a terminal in its `python/` directory
(`cd python` from the repository root). If downloaded as a ZIP, extract it first.
Connect the device, then create a virtual environment:

**Windows PowerShell:**

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install .
.\.venv\Scripts\python.exe -m hipnuc list
.\.venv\Scripts\python.exe -m hipnuc read
```

**Linux / Raspberry Pi / macOS:**

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install .
python -m hipnuc list
python -m hipnuc read
```

`list` shows available serial ports. `read` finds the connected HiPNUC device and
its baudrate, shows the selected connection, then displays measurements. Discovery
can take up to 30 seconds and shows each port/baudrate attempt and its result.
If several devices match, select one with `-p`.
Press Ctrl-C to stop.

Windows commands use the environment's Python directly, so no activation or PATH
change is needed. On Linux/macOS, activate the environment again in a new terminal.

## Read and record

The following examples use `python`. In Windows PowerShell, replace it with
`.\.venv\Scripts\python.exe` as above.

```sh
# Show device identity.
python -m hipnuc info

# Use a known connection. Replace COM3 with your port on Linux/macOS.
python -m hipnuc read -p COM3 -b 115200

# Record 60 seconds of decoded samples.
python -m hipnuc read --duration 60 --record samples.jsonl

# Also keep the original received bytes.
python -m hipnuc read --record samples.jsonl --record-raw capture.bin
```

The display shows up to five readings per second for each message type. Recording
keeps every decoded sample, independently of the display rate. `--quiet` hides
readings; `--jsonl` prints complete JSON measurements instead of the human display.
Existing files are protected unless you add `--overwrite`.

JSONL and the Python API use SI units: acceleration m/s², angular velocity rad/s,
and attitude rad. The human display uses ° and °/s for easier reading. Missing
measurements remain unavailable instead of becoming zeros.

Append `--help` to a `hipnuc` command for options. See the [API and CLI reference](docs/api.md)
for configuration and the [Modbus guide](docs/modbus.md) for addressed RTU devices.

## Send commands

```sh
python -m hipnuc command "LOG VERSION" -p COM3 -b 115200
python -m hipnuc command --file commands.txt -p COM3 -b 115200 --save
```

Put one product command per line in `commands.txt`; replies appear as each command
finishes. `--save` saves once after all commands succeed. Add `--reboot` when the
settings require a restart. Execution stops on the first failure.

## Use in your application

```python
from hipnuc import Recorder, SerialDevice

with SerialDevice() as device, Recorder("samples.jsonl") as recording:
    for sample in device.iter_samples():
        recording.write(sample)
        print(sample.acceleration_m_s2)
```

Use `SerialDevice("COM3", baudrate=115200)` for an explicit connection.
`sample.to_dict()` produces JSON-compatible data. For bytes you already have,
use `Decoder.feed(data)` without opening a device.

The examples are short scripts to edit and run. Change the constants at the top
of a script, then run it from the SDK's `python/` directory, for example
`python examples/read_samples.py`. They do not take command-line arguments.
On Windows, use `.\.venv\Scripts\python.exe` as above.

| Example | Use |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | Read IMU/INS measurements; `PORT = None` and `BAUDRATE = None` discover the connection |
| [record_samples.py](examples/record_samples.py) | Record to `JSONL_PATH = "samples.jsonl"`; set `RAW_PATH` to also keep received bytes |
| [send_commands.py](examples/send_commands.py) | Edit `COMMANDS`; the defaults query version and output configuration, without saving |
| [modbus_multinode.py](examples/modbus_multinode.py) | Poll nodes 80 and 81; set `PORT`, `BAUDRATE`, `NODE_IDS` and `INTERVAL_S` for your bus |

Reading, recording and Modbus polling continue until Ctrl-C. Recording protects
existing files. Use a new output filename for another recording.

## Troubleshooting

- **Command not found:** use the same Python for `-m pip install` and `-m hipnuc`.
  The Windows commands above do not depend on the Scripts directory being on PATH.
- **No serial ports:** check power, the USB data cable, and the USB-to-serial
  driver. On Windows, check Device Manager for a COM port.
- **Connection or read fails:** close CHCenter or other programs using the port.
  If you know the port and baudrate, specify both with `-p` and `-b`. A valid
  measurement stream can be read even when identity replies are unavailable.
- **Output is disabled:** with a single device connected, use
  `python -m hipnuc command "LOG ENABLE" -p COM3 -b 115200` at its actual baudrate.
- **Gaps or checksum errors:** ensure the output rate fits the serial bandwidth;
  reduce the rate or increase the device baudrate when needed. `-b` only sets the
  host connection speed; `baudrate NEW_BAUD` changes the device speed.
- **Unknown baudrate:** use `python -m hipnuc scan -p COM3`. Do not run ASCII
  discovery on a multi-node Modbus bus; use the [Modbus guide](docs/modbus.md).
- **Linux permission denied:** grant your user access to the serial device,
  commonly through the `dialout` group, then log in again. Prefer
  `/dev/serial/by-id/...` when available.
- **Python/venv unavailable:** install Python 3.10 or later. Ubuntu 22.04's
  default Python 3.10 is supported. Ubuntu, Debian and Raspberry Pi OS may need
  `sudo apt install python3-venv`. Keep SDK installation inside the virtual
  environment; do not use `sudo pip`.
- **Pi GPIO UART:** enable UART and disable the serial login console in the OS
  configuration. USB-to-serial adapters do not require this GPIO setting.

[IMU command and programming manual](https://download.hipnuc.com/en/products/imu/cum.html)
· [INS command and programming manual](https://download.hipnuc.com/en/products/ins/cum.html)
