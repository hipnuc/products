# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
`AGENTS.md` points here so other agents share the same instructions.

## Project Overview

**HiPNUC SDKs and Examples** — the public SDK and example collection for HiPNUC
IMU/AHRS/MRU/INS products. Supported devices: current-platform firmware 1.6.9 or
later (HI01–HI06, HI12–HI18, HI32, HI70/HI71, CH0X0). Legacy HI2xx/CH1xx products
and the 4-byte HI83 timestamp layout of early 1.7.1 builds are not targets.

The repository is a set of small, independent deliverables; keep each one
simple, professional and quick to start with. No release engineering (PyPI,
changelog, ROS index).

## Layout

```
c/hipnuc/      C core: hipnuc_dec (0x91/0x81/0x83), nmea_dec (GGA/RMC), hipnuc_sample (SI + valid bits),
               hipnuc_json, hipnuc_j1939 (J1939 + CANFD83), hipnuc_kboot / hipnuc_can_update (firmware update)
c/tools/       Linux CLI tools hihost (serial) and canhost (SocketCAN) + common/ (log, hexfile, ini)
c/tests/       ctest suite for the core, plus copied-files and C++ consumption checks
python/        Python SDK (src/hipnuc), click CLI, docs, examples, tests
ros/ros1/      ROS 1 Noetic package (catkin workspace; COLCON_IGNORE)
ros/ros2/      ROS 2 Humble/Jazzy packages (CATKIN_IGNORE)
stm32/serial/  Keil MDK project (StdPeriph): USART2 reception via board module, main.c is the whole app
stm32/can/     Keil MDK project: J1939 reception
ethercat/      HI15 IgH EtherCAT example
matlab/        CSV reading and Allan variance (not part of the SDK; leave as is)
protocol/dbc/  DBC files for CAN analysis tools
```

## Building and testing

```sh
# C core tests (Windows MinGW: add -G "MinGW Makefiles")
cmake -S c/tests -B build/c-tests && cmake --build build/c-tests && ctest --test-dir build/c-tests --output-on-failure

# Linux tools (POSIX only: termios, SocketCAN)
cmake -S c/tools -B build/tools && cmake --build build/tools

# Keil projects (command line; the IDE works too)
"C:/Keil_v5/UV4/UV4.exe" -b stm32/serial/USER/hipnuc_serial_decode.uvprojx -o build.log

# Python
cd python && python -m pip install -e ".[dev]" && python -m pytest tests -q && python -m ruff check . && python -m ruff format --check .

# ROS (Linux with ROS installed; CI uses ros:humble / ros:noetic containers)
cd ros/ros2 && colcon build          # cd ros/ros1 && catkin_make
```

## Rules that are easy to get wrong

- HI91 acceleration is encoded as `acc / 9.8` by the firmware: decode with 9.8, never 9.80665.
- `MAIN_STATUS` bits `WB_CONV` / `ATT_CONV` *set* mean NOT converged; the sample types expose
  `attitude_converged` / `gyro_bias_converged` booleans and Python `status_flags` are warnings.
- HI83: decode bits 0–19, 30, 31 only; bits 25–29 are internal and their wire order is not ascending
  (…27, 30, 31, 28, 29). Reject frames with unknown bits instead of guessing offsets.
- CAN: J1939 only (plus CANFD83, PGN 0xFF5B). CANopen exists solely inside the firmware-update client.
  Frames use `hipnuc_can_frame_t` with `len` as a byte count; reject remote and error frames; filter by the
  full 8-bit source address.
- The C core is copy-friendly: C99, no malloc, no stdio (except `hipnuc_json`), no global mutable state,
  no pointer casts into wire buffers, per-target CMake options only. Keep `c/hipnuc/README.md` copy lists true.
- Units: wire structs keep wire units (documented in headers); `hipnuc_sample_t`, JSON, Python and ROS are SI
  with Python key names (`acceleration_m_s2`, ...).
- Python: keep automatic discovery and actionable connection errors (port in use, no driver / dialout,
  no bytes, wrong baudrate); no legacy-firmware heuristics; no Modbus in C.
- Documentation: English primary with a `_zh` mirror; never link manual URLs (name the manual instead);
  do not repeat what headers or `--help` already say; tool features (e.g. firmware update) document usage only.
- Windows shell caveat for agents: the Bash tool collapses `\\n`-style escapes in command text; edit source
  files with the Edit/Write tools, not heredocs, when backslashes matter.

## Device communication

- ASCII commands (`LOG VERSION`, `SAVECONFIG`, `LOG HI91 ONTIME 0.01`): reply `OK`/`ERR`; unknown commands
  print nothing; `SERIALCONFIG` answers `OK` and switches immediately; `LOG <MSG> ONMARK ONCE` prints nothing.
- Linux serial access needs the `dialout` group; CAN: `sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up`.
