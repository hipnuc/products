[English](README.md) | [中文](README_zh.md)

# C and C++

Small C99 decoders for HiPNUC binary, NMEA and J1939/CANFD83 data. The core
needs no heap, operating system or global mutable state. Windows/Linux
applications can also use the synchronous serial interface; C++ calls the same C API.

## MCU or an existing receive loop

Copy the files you need from [hipnuc/](hipnuc). Each row is independent and lists
all required files. Add its `.c` files and include directory to your project.

| Input or output | Files |
| --- | --- |
| Serial binary HI91/HI81/HI83 | `hipnuc_dec.c/.h`, `hipnuc_sample.c/.h` |
| NMEA GGA/RMC | `nmea_dec.c/.h`, `hipnuc_sample.c/.h` |
| CAN J1939/CANFD83 | `hipnuc_j1939.c/.h`, `hipnuc_can_frame.h`, `hipnuc_sample.c/.h` |
| Optional JSON formatting | `hipnuc_json.c/.h`, `hipnuc_sample.c/.h` |

Feed the bytes received from your UART:

```c
#include "hipnuc_dec.h"

static hipnuc_raw_t decoder; /* zero-initialized; one per input stream */

void on_byte(uint8_t byte)
{
    hipnuc_sample_t sample;
    if (hipnuc_input(&decoder, byte) > 0) {
        hipnuc_sample_from_raw(&decoder, &sample);
        if (sample.valid & HIPNUC_VALID_ACC) {
            /* Use sample.acc[0..2], in m/s^2, in your application. */
        }
    }
}
```

`hipnuc_input()` returns `1` for a complete supported frame, `0` while waiting,
and `-1` for an invalid frame. Keep feeding after an error. Receive bytes into a
buffer in the interrupt and call the decoder from the main loop; the
[STM32 examples](../stm32/README.md) include wiring and complete Keil projects.

NMEA uses `nmea_input()` and `hipnuc_sample_from_nmea()`. For CAN, copy your
driver's frame into `hipnuc_can_frame_t` and call `hipnuc_j1939_parse()`; a
positive result means a new sample. Check `node_id` on a multi-device bus.

## Read from Windows or Linux

Install CMake 3.16 or later and a C compiler: Visual Studio / Build Tools with
the C++ build tools on Windows, or GCC/Clang on Linux.

Edit the settings at the top of one example:

| Example | Use |
| --- | --- |
| [read.c](examples/read.c) | C serial input |
| [read.cpp](examples/read.cpp) | C++ using the same C serial API |
| [read_can.c](examples/read_can.c) | Linux SocketCAN, Classic CAN and CAN FD |

From the repository root:

```sh
cmake -S c -B build/c
cmake --build build/c --config Release
```

Linux: run `./build/c/examples/read_c`, `./build/c/examples/read_cpp` or
`./build/c/examples/read_can`.
Windows with Visual Studio: run `.\build\c\examples\Release\read_c.exe` or
`.\build\c\examples\Release\read_cpp.exe`.
The C++ example is built when a C++ compiler is available.
Ctrl-C stops and closes the connection. These are integration examples;
printing every sample is unsuitable for high-rate recording.

On Ubuntu, serial access normally requires `sudo usermod -aG dialout "$USER"`,
then log out and back in. Set the correct port and baudrate. Increase
`TIMEOUT_MS` for slow device output. For SocketCAN, bring up the adapter at the
device's bitrate first; for example, Classic CAN at 500 kbit/s:

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

CAN FD additionally requires a capable adapter and the matching data bitrate.

## CMake integration

```cmake
add_subdirectory(path/to/c hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)
```

Choose `hipnuc_core` for binary/NMEA, `hipnuc_j1939` for CAN, or `hipnuc_json`
for optional JSON formatting. They share the independent `hipnuc_sample`
target. A parent project builds only what it links; examples are off by default.
No C++ compiler is needed for a C application.
The targets supply their include paths and C99 requirement to your application.

For Windows/Linux serial input, set `HIPNUC_BUILD_SERIAL` to `ON` before
`add_subdirectory()` and link `hipnuc_serial`. Start with a zero-initialized
`hipnuc_serial_t`; open the explicit port and baudrate, read, then close.
`hipnuc_serial_read_sample()` returns `1` for a new sample, `0` on timeout,
or `-1` on failure. See [hipnuc_serial.h](serial/hipnuc_serial.h)
for ownership and timeout contracts. Existing projects may also include
`c/hipnuc` or `c/serial` directly.

## Measurement conventions

Read a field only when its `HIPNUC_VALID_*` bit is set. These bits mean
**available**, not a valid navigation fix or converged attitude. Each sample
describes one packet; different CAN PGNs are never combined with old values.
Roll/pitch, yaw and heading are independent; INS and raw GNSS positions stay separate.

Samples use SI units: acceleration m/s² (gravity not removed), angular rate
rad/s, angles rad, magnetic field T, pressure Pa; positions use degrees and
temperature °C. The device's coordinate configuration is preserved. Units,
status and quality fields are documented in [hipnuc_sample.h](hipnuc/hipnuc_sample.h).
Use the derived convergence fields: a set device `ATT_CONV`/`WB_CONV` bit is a warning.

Supported products use current-platform firmware 1.6.9 or later, excluding
the early 1.7.1 HI83 layout with a 4-byte timestamp. The core
requires 8-bit bytes and IEEE 754 floats; position fields require 8-byte
`double`. Unsupported binary layouts and HI83 bitmap fields are rejected.
Only the optional JSON formatter uses `stdio`.

For desktop configuration and firmware updates, use CHCenter from
[official downloads](https://download.hipnuc.com). For discovery, commands,
recording, CAN operations and headless firmware updates, use the
[Python SDK](../python/README.md).
