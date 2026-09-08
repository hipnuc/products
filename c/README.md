[English](README.md) | [中文](README_zh.md)

# C and C++

Decode HiPNUC serial binary, NMEA and J1939/CANFD83 data, or read a device
directly from a Windows/Linux application. C++ uses the same C API.

## Read a device

Edit `PORT` and `BAUDRATE` in [read.c](examples/read.c) or
[read.cpp](examples/read.cpp), then build from this directory:

```sh
cmake -S examples -B build/examples
cmake --build build/examples --config Release
```

Run `./build/examples/read_c` or `./build/examples/read_cpp` on Linux. With Visual Studio on
Windows, run `.\build\examples\Release\read_c.exe` or `.\build\examples\Release\read_cpp.exe`.
Ctrl-C stops and closes the connection. On Ubuntu, your account needs serial
port access: `sudo usermod -aG dialout "$USER"`, then log out and back in.

## Integrate into your project

- **Microcontroller or your own transport:** copy the files listed in the
  [core guide](hipnuc/README.md). The C99 core has no heap or OS dependencies.
- **CMake:** `add_subdirectory(path/to/c/hipnuc hipnuc)` and link `hipnuc_core`
  for binary/NMEA, or `hipnuc_j1939` for CAN. JSON formatting is optional.
- **Desktop serial:** add `path/to/c/serial` instead and link `hipnuc_serial`.
  Use one zero-initialized `hipnuc_serial_t` per connection; see the
  [public header](serial/hipnuc_serial.h) for return values and ownership.

```c
hipnuc_sample_t sample;
int result = hipnuc_serial_read_sample(&device, &sample, 200);
if (result == 1 && (sample.valid & HIPNUC_VALID_ACC)) {
    /* sample.acc[] is specific force in m/s^2, with gravity not removed. */
}
```

A sample describes one received packet. Read a field only when its validity
bit is set; absence is not zero. The device's coordinate configuration is
preserved. INS and raw GNSS data remain separate.

For discovery, configuration and serial recording use the [Python SDK](../python/README.md).
For firmware update and SocketCAN use the [specialized tools](tools/README.md).
