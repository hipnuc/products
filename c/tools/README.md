[English](README.md) | [中文](README_zh.md)

# Firmware update and CAN tools

For serial discovery, configuration and recording, use the [Python SDK](../../python/README.md).
These tools provide serial firmware update on Windows/Linux and SocketCAN access on Linux.

## Build

From this directory, with a C compiler and CMake installed:

```sh
cmake -S . -B build
cmake --build build --config Release
```

On Linux the executables are `build/serial_update/hipnuc-update` and
`build/canhost/canhost`. Visual Studio places the Windows updater at
`build\serial_update\Release\hipnuc-update.exe`.

## Serial firmware update

Use the Intel HEX image for the exact device model. Specify its current
connection speed and port:

```powershell
.\build\serial_update\Release\hipnuc-update.exe firmware.hex -p COM3 -b 115200
```

On Linux:

```sh
./build/serial_update/hipnuc-update firmware.hex -p /dev/ttyUSB0 -b 115200
```

A failure stops the operation without an automatic reset. A successful transfer
and reset acknowledgement do not verify the new application's startup.

## CAN (Linux)

First bring up your interface at the device's bitrate, for example:

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

From this directory:

```sh
./build/canhost/canhost list
./build/canhost/canhost scan -i can0 --duration 2
./build/canhost/canhost read -i can0 -n 8 --duration 10
./build/canhost/canhost read -i can0 -n 8 --record samples.jsonl
```

`scan` observes valid measurement traffic; it cannot discover a silent device.
`read` outputs JSONL to stdout or to `--record`, with diagnostics on stderr.
Omit `-n` to receive all source addresses. Ctrl-C stops after the current batch;
`--count` and `--duration` also finish the current batch. Existing recording
files are protected unless `--overwrite` is given.

Register addresses, values and applicable models are defined in the product
manual. Pass raw numeric values (decimal or `0x` hexadecimal):

```text
./build/canhost/canhost reg read ADDRESS -i can0 -n 8
./build/canhost/canhost reg write ADDRESS VALUE -i can0 -n 8
./build/canhost/canhost sync PGN -i can0 -n 8 --interval 0.01 --count 10
./build/canhost/canhost update firmware.hex -i can0 -n 8
```

Use only addresses and trigger PGNs supported by your product. Save and reboot
are explicit register writes; ordinary writes do not automatically save.
Register requests use host address `0x55`, as required by the device reply
protocol. CAN update accepts device IDs 1–127 and uses CANopen SDO only for the
bootloader. `--bin` selects a raw binary update image.

Run the final command with `--help` for its parameters. Commands never read an
INI file or change the interface bitrate. Exit codes: 0 success, 1 runtime
failure, 2 invalid arguments, 130 Ctrl-C.
