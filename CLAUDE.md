# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **HiPNUC Product Software Examples Package** — a multi-platform SDK and example collection for HiPNUC IMU/INS (Inertial Measurement Unit / Inertial Navigation System) products. The repository contains:

- Cross-platform driver libraries (C)
- CLI tools for Linux (serial and CAN)
- Example code for Python, STM32, Arduino, ROS, MATLAB, and EtherCAT

## Repository Structure

```
drivers/          # Shared decoder libraries used by all examples
  hipnuc_dec.{c,h}          # HiPNUC binary protocol decoder (0x91/0x81/0x83 packets)
  nmea_dec.{c,h}            # NMEA parser (GGA/RMC/SXT)
  hipnuc_can_common.{c,h}   # CAN common types and JSON output
  hipnuc_j1939_parser.{c,h} # J1939 CAN frame parser
  canopen_parser.{c,h}      # CANopen TPDO frame parser
  example_data.{c,h}        # Static example data for testing

examples/
  C/              # Linux serial CLI tool: hihost
  CAN/linux/      # Linux CAN CLI tool: canhost
  CAN/stm32/      # STM32F103 CAN example (Keil MDK project)
  CAN/dbc/        # DBC files for CAN analysis tools
  python/         # Python CLI for serial devices
  stm32_serial/   # STM32 USART example (Keil MDK project)
  ROS_Melodic/    # ROS Melodic example
  ROS2/           # ROS2 example
  arduino/        # Arduino example
  matlab/         # MATLAB data reading + Allan variance analysis
  ecat/           # EtherCAT example
```

## Building

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

### Windows (either tool, using MinGW + Ninja)
```powershell
mkdir build && cd build
cmake .. -G "Ninja"
ninja
```

### STM32 Examples
Open the Keil MDK project files:
- Serial: `examples/stm32_serial/` (Keil MDK V5.38)
- CAN: `examples/CAN/stm32/USER/hipnuc_can_decode.uvprojx`

### Python
```sh
cd examples/python
pip install -r requirements.txt
python main.py --help
```

## Key Architectural Concepts

### Driver Layer (`drivers/`)
The core decoder libraries are designed to be embedded in any project. The primary interface for serial decoding is:
- `hipnuc_input(raw, byte)` — feed one byte at a time; returns 1 when a full packet is decoded
- Three packet types: `hi91_t` (IMU float), `hi81_t` (INS raw), `hi83_t` (INS float with bitmap)

### CLI Architecture (both `hihost` and `canhost`)
Both tools share the same modular command pattern:
- `main.c` — parses global options, calls `execute_command()`
- `commands.c` — command registry and dispatch table
- `command_handlers.h` — declares all `cmd_<name>()` functions
- `commands/cmd_<name>.c` — individual command implementations

### Adding a New Command to `hihost`
1. Create `examples/C/commands/cmd_<name>.c` implementing `int cmd_<name>(GlobalOptions *opts, int argc, char **argv)`
2. Declare it in `examples/C/command_handlers.h`
3. Register it in `examples/C/commands.c` command table: `{"<name>", cmd_<name>, "<desc>"}`
4. Add the source file to `examples/C/CMakeLists.txt` `SOURCES` list

### CAN Protocol Support
- **J1939**: 29-bit extended frames → `hipnuc_j1939_parser`
- **CANopen**: 11-bit standard frames → `canopen_parser`
- DBC files for analysis tools: `examples/CAN/dbc/J1939.dbc` and `CANopen.dbc` (default node ID `0x08`, little-endian)

### `canhost` Configuration (`canhost.ini`)
Loaded in order: `$CANHOST_CONF` → `./canhost.ini` → `~/.canhost.ini` → `/etc/canhost.ini`

### `hihost` Configuration (`hihost.ini`)
Fixed path at `examples/C/hihost.ini`. Stores `port=` and `baud=`. Updated automatically after `probe`.

## Device Communication

- AT command protocol: send commands as plain text (e.g., `LOG VERSION`, `SAVECONFIG`)
- Tools automatically stop output before sending commands and restore it after
- Linux serial port access typically requires `sudo`
- CAN interface setup: `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`
