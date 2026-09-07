# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
`AGENTS.md` points here so other agents share the same instructions.

## Project Overview

This is the **HiPNUC SDKs and Examples** repository — a multi-platform SDK and example collection for HiPNUC IMU/INS (Inertial Measurement Unit / Inertial Navigation System) products. The repository contains:

- Cross-platform driver libraries (C)
- CLI tools for Linux (serial and CAN)
- A small installable Python SDK with CLI, documentation, examples, and tests
- Example code for STM32, Arduino, ROS, MATLAB, and EtherCAT

Supported products for the current SDK: firmware 1.6.9 or later on the current
platform (HI01–HI06, HI12–HI18, HI32, HI70/HI71, CH0X0). Legacy HI2xx/CH1xx
products are not a target of new work.

## Repository Structure

```
drivers/          # Shared C decoder libraries
  hipnuc_dec.{c,h}          # HiPNUC binary protocol decoder (0x91/0x81/0x83 packets)
  nmea_dec.{c,h}            # NMEA parser
  hipnuc_can_common.{c,h}   # CAN common types and JSON output
  hipnuc_j1939_parser.{c,h} # J1939 CAN frame parser
  canopen_parser.{c,h}      # CANopen TPDO frame parser
  hipnuc_can_update.{c,h}   # CANopen SDO firmware update client
  example_data.{c,h}        # Static example data for testing

python/           # Python SDK: package, CLI, docs, examples, and tests
  src/hipnuc/     # Runtime package
  examples/       # Short scripts using the installed SDK
  docs/           # api.md quick reference, modbus.md (+ modbus_zh.md)

tests/c/          # Host-side C decoder regression tests (CMake + ctest)

examples/
  C/              # Linux serial CLI tool: hihost
  CAN/linux/      # Linux CAN CLI tool: canhost
  CAN/stm32/      # STM32F103 CAN example (Keil MDK project)
  CAN/dbc/        # DBC files for CAN analysis tools
  stm32_serial/   # STM32 USART example (Keil MDK project)
  ROS_Melodic/    # ROS 1 example
  ROS2/           # ROS 2 example
  arduino/        # Arduino example
  matlab/         # MATLAB data reading + Allan variance analysis
  ecat/           # EtherCAT example (HI15)
```

## Building

`hihost` and `canhost` are POSIX/Linux-only (termios, SocketCAN). They do not
build on Windows; use the Python SDK there.

### `hihost` (Linux serial CLI)
```sh
cd examples/C
mkdir -p build && cd build
cmake ..
make
# Output: build/hihost
```

### `canhost` (Linux CAN CLI)
```sh
cd examples/CAN/linux
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# Output: build/canhost
```

### C decoder tests
```sh
cmake -S tests/c -B build/c-tests
cmake --build build/c-tests
ctest --test-dir build/c-tests --output-on-failure
```

### STM32 Examples
Open the Keil MDK project files; both reference `drivers/` by relative path:
- Serial: `examples/stm32_serial/` (Keil MDK V5.38)
- CAN: `examples/CAN/stm32/USER/hipnuc_can_decode.uvprojx`

### Python
```sh
cd python
# Create and activate a virtual environment as described in README.md first.
python -m pip install .
python -m hipnuc --help
```

The Python SDK supports Python 3.10+. It is deliberately small: `SerialDevice`
(with automatic port/baudrate discovery), `Decoder`, `Recorder`, `ModbusBus`
and a click CLI. Put CLI connection options after the final subcommand, for
example `python -m hipnuc read -p COM3 -b 115200`. The four scripts under
`python/examples/` use editable constants and a `main()` entry point; they have
no argument parsers and perform no I/O when imported. Developer checks are in
`python/tests/README.md`. Do not add release engineering (PyPI, changelog) or
legacy-firmware heuristics; keep first-connection ergonomics (discovery, clear
error hints).

Documentation policy: English primary with a `_zh` Chinese mirror; avoid
linking to manual URLs (they change), name the manual in words instead.

## Key Architectural Concepts

### Driver Layer (`drivers/`)
The core decoder libraries are designed to be embedded in any project. The primary interface for serial decoding is:
- `hipnuc_input(raw, byte)` — feed one byte at a time; returns 1 for a complete frame, 0 for incomplete input, -1 for an invalid frame. Always test `> 0`.
- Three packet types: `hi91_t` (IMU float), `hi81_t` (INS raw), `hi83_t` (INS float with bitmap)
- HI91 acceleration is encoded as `acc / 9.8` by the firmware; decode with 9.8, not 9.80665.

### CLI Architecture (both `hihost` and `canhost`)
Both tools share the same modular command pattern:
- `main.c` — parses global options, calls `execute_command()`
- `commands.c` — command registry and dispatch table
- `command_handlers.h` — declares all `cmd_<name>()` functions
- `commands/cmd_<name>.c` — individual command implementations

### Adding a New Command to `hihost`
1. Create `examples/C/commands/cmd_<name>.c` implementing `int cmd_<name>(GlobalOptions *opts, int argc, char **argv)`
2. Declare it in `examples/C/command_handlers.h`
3. Register it in the `examples/C/commands.c` command table: `{"<name>", cmd_<name>}`
4. Add the source file to `examples/C/CMakeLists.txt` `SOURCES` list

### CAN Protocol Support
- **J1939**: 29-bit extended frames → `hipnuc_j1939_parser`
- **CANopen**: 11-bit standard frames → `canopen_parser`
- DBC files for analysis tools: `examples/CAN/dbc/J1939.dbc` and `CANopen.dbc` (default node ID `0x08`, little-endian)

### `canhost` Configuration (`canhost.ini`)
Loaded in order: `$CANHOST_CONF` → `<source dir>/canhost.ini` (compiled-in path) → `./canhost.ini` → `~/.canhost.ini` → `/etc/canhost.ini`

### `hihost` Configuration (`hihost.ini`)
Compiled-in absolute path `examples/C/hihost.ini`. Stores `port=` and `baud=`. Updated automatically after `probe`.

## Device Communication

- ASCII command protocol: send commands as plain text (e.g., `LOG VERSION`, `SAVECONFIG`); the device replies `OK` or `ERR` and prints nothing for unknown commands
- Python reads measurements and command responses through one serial session without changing output settings
- Linux serial access requires device permissions, commonly membership in `dialout`
- CAN interface setup: `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`
