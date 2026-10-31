# HiPNUC Python SDK

[English](README.md) | [中文](README_zh.md)

`hipnuc` is the Python SDK for HiPNUC IMU, AHRS, MRU and INS modules, and
`hihost` is its command-line tool. Read and record data, send commands, change
settings and update firmware over serial, Modbus RTU or CAN.

- Python 3.10–3.14 on Windows, Linux (including Raspberry Pi OS and other
  embedded boards) and macOS
- Serial messages HI91, HI81, HI83, NMEA GGA and RMC
- Optional interfaces: Modbus RTU (`[modbus]`, PyModbus) and CAN J1939 / CANFD83
  (`[can]`, python-can)
- Devices: HI01–HI06, HI12–HI18, HI70/HI71 and CH0X0; `-I2C` models have no CAN

## Install

Open a terminal in the `python/` directory of this repository.

Windows (PowerShell):

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install .
hihost --version
```

Linux and macOS:

```sh
python3 -m venv .venv            # Ubuntu/Debian/Pi OS: sudo apt install python3-venv first
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
hihost --version
```

Add the optional interfaces you need with `python -m pip install ".[modbus,can]"`.
Activate the environment again in each new terminal. To use the SDK in an
existing project, activate that project's environment and run
`python -m pip install /path/to/products/python`; install again after you
update the SDK source. If the shell cannot find `hihost`, run `python -m hipnuc`
with the same arguments.

For a board without network access, or with a Python older than 3.10, see
[Embedded Linux boards](#embedded-linux-boards).

## First steps

Connect the device and close any other program that uses its serial port.

```sh
hihost list                                       # USB serial ports
hihost read                                       # find the device, print readings; Ctrl-C stops
hihost read --duration 60 --record samples.jsonl  # record 60 s
hihost info                                       # product, firmware, serial number
```

Without `-p`, `hihost` sends an identity query to each USB serial port to find
the device, which can take up to 30 s. With several devices, or for a port that
is not USB, give the port; add `-b` when you know the baud rate:

```sh
hihost read -p COM3 -b 115200
hihost read -p /dev/ttyUSB0 -b 115200
hihost read -p /dev/serial/by-id/usb-...   # stable name when a board has several USB serial devices
```

`hihost list --all` also shows non-USB ports. On Linux, some on-board UARTs such
as `/dev/ttyS3` do not appear in the list; pass them with `-p`. Options follow
the last command word, for example `hihost modbus read -p COM3`, and
`hihost <command> --help` lists them.

### Commands and settings

```sh
hihost command "LOG VERSION" -p COM3
hihost command --file commands.txt -p COM3 --save   # one command per line
hihost baudrate 921600 -p COM3                      # change the device's baud rate
hihost scan -p COM3                                 # find the baud rate on this port
hihost reboot -p COM3
```

Command names and their effect are in the product's command and programming
manual. In a command file, `#` and `;` start comments, execution stops at the
first failure, and `--save` saves once after all commands succeed. `-b` only sets
the computer's baud rate; `hihost baudrate` changes the device's and reconnects.
`LOG <message> ONMARK ONCE` gets no reply, so send it alone with `--no-reply`
(Python: `response="none"`).

### Recording

`--record` writes every decoded sample as JSON Lines in SI units (angles in
radians). The screen shows angles in degrees and at most five readings per second
per message type; the recording keeps all of them.

- `--record-raw capture.bin` also saves the received bytes; `hihost decode capture.bin`
  decodes them later (`--jsonl` or `--record out.jsonl`).
- `--quiet` hides readings; `--jsonl` prints one JSON object per sample.
- Existing files are kept unless you add `--overwrite`.

On stop, `hihost` prints the measured rate of each message type. Exit codes are
0 for success, 1 for failure, 2 for a usage error and 130 for Ctrl-C.
`hihost -v <command>` prints SDK debug logs to stderr.

The environment variables `HIHOST_PORT`, `HIHOST_BAUDRATE`, `HIHOST_MODBUS_ID`,
`HIHOST_CAN_INTERFACE`, `HIHOST_CAN_ID` and `HIHOST_CAN_BITRATE` set defaults
for the matching options. For shell completion, run
`eval "$(_HIHOST_COMPLETE=bash_source hihost)"` (or `zsh_source`, `fish_source`).

## Python

```python
from hipnuc import SerialDevice

with SerialDevice() as device:  # or SerialDevice("COM3", baudrate=115200)
    for sample in device.iter_samples():
        print(sample.acceleration_m_s2, sample.angular_velocity_rad_s)
```

Creating an object does no I/O; `with` opens and closes the port, and every call
is synchronous. A port or baud rate left as `None` is found by discovery.

| API | Purpose |
| --- | --- |
| `SerialDevice(port=None, baudrate=None, timeout=2.0)` | Serial connection; times in seconds |
| `device.read()`, `device.iter_samples()` | Next sample, or a stream of samples; `ResponseTimeout` when the link stays idle |
| `device.read_info()` | Product, firmware and serial number |
| `device.command("LOG VERSION")` | Send an ASCII command; returns `CommandResult` with `text` and `acknowledged` |
| `device.save_config()`, `device.set_baudrate(rate)`, `device.reboot()` | Save, change the baud rate, restart |
| `Decoder().feed(data)` | Decode bytes from any source; call `finish()` at the end |
| `sample.values`, `sample.to_dict()` | Message fields, JSON-ready dictionary |

**Units and conventions.** Each sample is one message. Values are in SI units (m/s², rad/s, rad,
T, Pa); latitude and longitude are in degrees and temperature in °C. Missing
values are `None`. `quaternion_wxyz` rotates body to navigation frame, and
`euler_rad` follows the device's configured convention. Heading is measured
clockwise from north and differs from Euler yaw. `received_time_ns` is the host
time of the received chunk; use the device time fields for sample timing.

**Errors.** Communication errors derive from `HipnucError`: `TransportError`,
`ResponseTimeout`, `DeviceError` (request rejected) and `VerificationError`
(readback mismatch). Invalid arguments raise `ValueError`. Commands are never
retried. When a sent command times out, `CommandTimeout.sent` is true and the
connection closes, but the device may still be executing it; wait before you
reconnect.

To record from Python, open the device first, then create the files and attach
the callbacks:

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

`write_raw` takes the received chunks, not `sample.raw`. Callbacks run in the
reading thread, so keep them short and do not call device I/O from them. Files
are flushed every second and on close.

### Examples

Edit the constants at the top of a script, then run it, for example
`python examples/read_samples.py`.

| Example | What it does |
| --- | --- |
| [read_samples.py](examples/read_samples.py) | Print measurements; finds a USB device when `PORT` is `None` |
| [record_samples.py](examples/record_samples.py) | Record JSON Lines, and raw bytes when `RAW_PATH` is set |
| [send_commands.py](examples/send_commands.py) | Send the commands in `COMMANDS` |
| [modbus_multinode.py](examples/modbus_multinode.py) | Poll several Modbus stations on one bus |
| [can_read.py](examples/can_read.py) | Read CAN measurements and identity through any python-can bus |
| [update_firmware.py](examples/update_firmware.py) | Update firmware over serial or CAN |

## Modbus RTU

```sh
python -m pip install ".[modbus]"
hihost modbus info -p COM3 --id 80
hihost modbus read -p COM3 --id 80 --duration 60 --record samples.jsonl
```

The factory settings are 115200 baud, 8N1 and station ID 80; unicast IDs are
1–247. Prepare each device before it joins a bus:

- Turn off periodic output on the Modbus port, or data frames mix with the
  replies. Connect the device alone, switch off each timed message over ASCII
  (for example `LOG HI91 ONTIME 0`) and save. `LOG DISABLE` lasts only until the
  next restart.
- Give each device a unique station ID. Do not run ASCII `scan` or `command` on a
  shared Modbus bus.

`read` polls until Ctrl-C, `--duration` or `--count`; `--interval` sets the pause
between polls. `registers` and `write-register` give raw FC03/FC06 access,
`set-id` and `baudrate` change communication settings, and `reboot` restarts the
device. Register addresses are in the product manual.

```python
from hipnuc import ModbusBus

with ModbusBus("COM3") as bus:
    device = bus.device(80)
    print(device.read_info())
    print(device.read_sample().acceleration_m_s2)
```

One `ModbusBus` owns the port, and its devices take turns on it. Values wider
than 16 bits are big-endian, high word first. Writes are read back by default
and saved only on request (`--save` or `save_config()`).

## CAN

### Linux (SocketCAN)

```sh
python -m pip install ".[can]"
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
# CAN FD products: ... bitrate 500000 dbitrate 4000000 fd on restart-ms 100
sudo ip link set can0 up
hihost can scan -i can0 --identify
hihost can read -i can0 --id 8 --record samples.jsonl
```

The factory settings are 500 kbit/s and node ID 8, and `restart-ms` lets the
controller recover from bus-off by itself. PEAK adapters work with the kernel's
`peak_usb` driver; PEAK's own Linux driver replaces it, and its default build
provides no SocketCAN. When a command fails, `hihost` names the likely cause and
prints the `ip link` commands to fix it. To keep the settings across reboots, add
them to the system network configuration, for example
`/etc/systemd/network/80-can0.network`:

```ini
[Match]
Name=can0

[CAN]
BitRate=500000
RestartSec=100ms
```

### Windows (PCAN)

Install the PEAK driver, which includes PCAN-Basic. A PCAN channel also needs
the node's bitrate, so in the examples below replace `-i can0` with
`-i PCAN_USBBUS1 --bitrate 500000`, or set `HIHOST_CAN_INTERFACE` and
`HIHOST_CAN_BITRATE` once. PCAN channels open in Classic CAN mode.

```sh
hihost can scan -i PCAN_USBBUS1 --bitrate 500000 --identify
```

### Reading and configuration

`scan` lists the nodes on the bus and the rate of each PGN; `--identify` adds
their identity. `read --id` keeps one node, and CAN FD frames need `--fd`.
Recordings use the same JSON Lines format as serial and keep each frame's node
ID and CAN identifier.

Configuration commands address one node (`--id`, 1–127; 85 is reserved for the
host), and the device stores each setting when it is written. A new node ID or
bitrate takes effect after a restart:

```sh
hihost can info -i can0 --id 8
hihost can period acc 10 -i can0 --id 8          # period in ms, off, or sync (SYNC_IN)
hihost can output off -i can0 --id 8             # all periodic PGNs
hihost can trigger quat -i can0 --all            # one frame from every node, no reply
hihost can set-id 9 -i can0 --id 8 --reboot
hihost can bitrate 1000000 -i can0 --id 8 --reboot   # then set can0 to 1000000
hihost can reboot -i can0 --id 8
```

PGN names are acc, gyr, mag, roll_pitch, yaw, quat, temp, inclination, time,
position, altitude, gnss_status, velocity and canfd83; numbers work too. After
`can reboot --factory-reset` the node may return at 500 kbit/s as node 8.

Raw registers: `hihost can reg read ADDRESS -i can0 --id 8` and
`hihost can reg write ADDRESS VALUE -i can0 --id 8` (decimal or `0x`). A write
only checks the echo. For CAN FD output, register `0x180` holds the CANFD83
field bitmap (for example `0x12B`) and `0x9B` the data-bitrate code (`11` for
4 Mbit/s); write one at a time, then restart.

### Python

Use a python-can bus with the SDK functions:

```python
import can
from hipnuc.can import decode_message

with can.Bus(interface="socketcan", channel="can0", ignore_config=True) as bus:
    for message in bus:
        try:
            sample = decode_message(message)
        except ValueError:  # Another device's frame on a shared PGN.
            continue
        if sample is not None:
            print(sample.values)
```

Other adapters work the same way once the bus is created with their settings,
for example `can.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=500000,
ignore_config=True)`. CANFD83 frames arrive only on an FD bus: configure the
interface for CAN FD and create the bus with `fd=True`.

`decode_message()` returns `None` for unrelated frames and raises `ValueError`
for a malformed HiPNUC frame. It keeps the adapter timestamp in
`metadata["can_timestamp_s"]` and leaves `received_time_ns` for the caller to
set. `read_register` and `write_register` give raw access; `read_info`,
`reboot`, `request_reboot`, `wait_for_node`, `factory_reset`, `save_config`,
`set_node_id`, `set_bitrate`, `set_pgn_period`, `set_output_enabled` and
`send_trigger` cover the managed operations. They take the caller's bus and a
node ID; `scan(bus, duration)` finds the nodes itself.

Use one receiver per bus: register functions consume the frames that arrive
while they wait. Receive filters must pass the node's replies, extended IDs
`0x0CEF55xx` for registers and standard ID `0x580` + node ID for updates.
`make_trigger(node_id, pgn)` builds a trigger frame for `bus.send()` or
`bus.send_periodic()`. `pgn` is a number, for example `PGN_NAMES["QUAT"]` from
`hipnuc.can`; node 255 addresses every node, and no reply is expected.

## Firmware update

```sh
hihost update firmware.hex -p /dev/ttyUSB0
hihost can update firmware.hex -i can0 --id 8
hihost can update firmware.hex -i PCAN_USBBUS1 --bitrate 500000 --id 8   # Windows
```

Use the Intel HEX application image that HiPNUC provides for your exact product;
the bootloader cannot tell products apart. `hihost` checks the image before it
touches the device, detects the serial baud rate when `-b` is omitted, and
afterwards confirms that the new firmware runs (`--no-verify` skips this). Any
version can be installed, including an older one. Close other programs that use
the port or bus, and keep the device still during a serial update. A 190 KB
image takes about 25 s over serial and 40 s over CAN at 500 kbit/s. CHCenter,
HiPNUC's desktop tool, offers the same update.

If an update stops part-way, run the same command again. If the device does not
answer at all, for example after an image for another product was installed, the
command asks you to power-cycle the device and catches its bootloader as it
starts:

- Serial: unplug the device's whole connector, power and signal lines together,
  then plug it back. A USB serial adapter stays in the computer.
- CAN: the bootloader starts at 500 kbit/s as node 8, so set the interface to
  500 kbit/s and use `--id 8`. Only one device may use that node ID while it
  updates.

In Python, `update_serial(path, port=..., baudrate=None)` opens and closes its
own connection, and `update_can(bus, node_id, path)` uses your bus at the node's
bitrate. Both return `UpdateResult(bytes_written, image_version,
previous_version, verified_version)`, where an unknown version is `None`, and
accept `verify=False` and an optional `progress(written, total)` callback;
raising from the callback cancels the update. When nothing answers, the
power-cycle request is logged as a WARNING on the `hipnuc.update` logger and the
call waits up to `power_cycle_wait` seconds (default 60, `0` fails at once). An
invalid image file raises `ValueError`, an unreadable file `OSError`, and an image
the bootloader refuses `DeviceError`. A serial update does not need the CAN extra.

## Embedded Linux boards

The SDK and its dependencies install from wheels, so x86, 64-bit ARM (for
example RK3588) and 32-bit ARM boards need no compiler.

**Without network access.** On a connected computer, build the SDK wheel and
download the dependencies for the board's Python version. Use the platform
`manylinux2014_aarch64` for 64-bit ARM, `linux_armv7l` for 32-bit ARM or
`manylinux2014_x86_64` for x86. Copy `wheels/` to the board and install it in a
virtual environment there (the board needs `python3-venv` on Ubuntu/Debian).
`--force-reinstall` also replaces an installed SDK of the same version.

```sh
python -m pip wheel --no-deps -w wheels .
python -m pip download -d wheels --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.10 "pyserial>=3.5,<4" "click>=8.1,<9" "python-can>=4.6,<5" "pymodbus>=3.15,<3.16"
# on the board, inside the virtual environment:
python -m pip install --no-index --find-links wheels --force-reinstall "hipnuc-sdk[can,modbus]"
```

**Python older than 3.10** (Ubuntu 20.04 ships 3.8, Debian 11 ships 3.9). Use a
standalone CPython from the python-build-standalone releases on GitHub; it needs
no root access and runs beside the system Python on glibc 2.17 or newer.
Download `cpython-3.10.<x>+<date>-aarch64-unknown-linux-gnu-install_only.tar.gz`
(32-bit ARM: `armv7-unknown-linux-gnueabihf`; in mainland China the npmmirror
binary mirror has the same files), fetch the wheels with `--python-version 3.10`,
copy both to the board and run:

```sh
tar -xzf cpython-3.10.*-install_only.tar.gz -C "$HOME"
"$HOME/python/bin/python3" -m venv "$HOME/.venvs/hipnuc"
. "$HOME/.venvs/hipnuc/bin/activate"
python -m pip install --no-index --find-links wheels --force-reinstall "hipnuc-sdk[can,modbus]"
```

## Troubleshooting

| Symptom | What to do |
| --- | --- |
| `python3 -m venv` fails | Install `python3-venv` (Ubuntu/Debian/Pi OS). |
| Python older than 3.10 | Use a standalone Python, see [Embedded Linux boards](#embedded-linux-boards). |
| PowerShell blocks `Activate.ps1` | Run `.\.venv\Scripts\python.exe -m pip ...` and `.\.venv\Scripts\hihost.exe` directly. |
| `No module named hipnuc` or `UNKNOWN-0.0.0` | Activate the right environment, upgrade pip and reinstall from this directory. |
| `Modbus support requires` / `CAN support requires` | Install the extra in the same environment: `python -m pip install ".[modbus]"` or `".[can]"`. |
| Linux: permission denied on the port | Add yourself to the port's group, usually `dialout` (`ls -l /dev/ttyUSB0` shows it): `sudo usermod -a -G dialout "$(id -un)"`, then log out and back in. |
| No port, or port busy | Check the cable and driver, USB passthrough in a virtual machine, and other programs using the port. On Linux, ModemManager may hold a new port for a few seconds. |
| Port opens but no samples | Check the baud rate, wiring, and that output is enabled; give slow output a longer `--timeout`. |
| CAN: no node answers, or bus-off | Check the bitrate, wiring and termination (120 Ω at both ends). If no frames arrive even at the right bitrate, replug the USB CAN adapter. |
| Update stopped part-way | Run the same command again; see [Firmware update](#firmware-update). |
