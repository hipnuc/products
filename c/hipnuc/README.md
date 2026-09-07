# HiPNUC C core

[English](README.md) | [中文](README_zh.md)

Portable C99 decoders and firmware-update clients for HiPNUC devices
(firmware 1.6.9 or later). No dynamic memory, no OS dependency, no global
state, no `stdio` in the decoders. Builds with GCC, Clang, MinGW, MSVC and
ARM Compiler 5/6. Field layouts, units and status bits are documented in the
headers.

## Copy the files you need

| Purpose | Files |
| --- | --- |
| Serial binary stream (HI91/HI81/HI83) | `hipnuc_dec.c/.h` |
| + SI units and validity flags | `hipnuc_sample.c/.h`, `nmea_dec.h` |
| + NMEA `$GPGGA` / `$GPRMC` | `nmea_dec.c` |
| + JSON output (uses `stdio`) | `hipnuc_json.c/.h` |
| CAN: J1939 and CANFD83 | `hipnuc_j1939.c/.h`, `hipnuc_can_frame.h`, `hipnuc_sample.c/.h`, `hipnuc_dec.h`, `nmea_dec.h` |
| Firmware update over serial | `hipnuc_kboot.c/.h`, `hipnuc_dec.c/.h` |
| Firmware update over CAN | `hipnuc_can_update.c/.h`, `hipnuc_can_frame.h` |

Or, with CMake:

```cmake
add_subdirectory(path/to/c/hipnuc hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)   # hipnuc_json, hipnuc_j1939, hipnuc_update
```

## Serial

```c
static hipnuc_raw_t raw;                 /* one per port, zero-initialized */

void on_byte(uint8_t byte)               /* your UART receive path */
{
    hipnuc_sample_t s;
    if (hipnuc_input(&raw, byte) > 0 && hipnuc_sample_from_raw(&raw, &s)) {
        if (s.valid & HIPNUC_VALID_EULER) use(s.roll, s.pitch, s.yaw);   /* rad */
        if (!s.attitude_converged) keep_still();
    }
}
```

`hipnuc_input()` returns 1 for a complete frame, 0 while more bytes are
needed, -1 for a damaged frame; test `> 0`. `hipnuc_input_buffer()` feeds a
block and stops after the first frame. `hipnuc_sample_t` is the SI view with
one validity bit per field; the raw packets in `hipnuc_raw_t` keep wire
units. NMEA uses `nmea_input()` and `hipnuc_sample_from_nmea()` the same way.

Things easy to get wrong:

- A set `WB_CONV` / `ATT_CONV` status bit means *not* converged. Use
  `s.attitude_converged` and `s.gyro_bias_converged` instead of the raw bits.
- HI91 acceleration is in G on the wire; the SDK converts with 1 G = 9.8 m/s²
  (the firmware constant, not 9.80665).
- HI83 frames carrying the internal bits 25–29 are rejected, not partially
  decoded.

## CAN

```c
hipnuc_can_frame_t frame;   /* id, is_extended, len, data from your driver */
hipnuc_sample_t part, merged = {0};
if (hipnuc_j1939_parse(&frame, &part, NULL) > 0 && part.node_id == 8)
    hipnuc_j1939_merge(&merged, &part);
```

Each PGN fills only the fields it carries; merge the parts of one source
address yourself. CANFD83 (PGN 0xFF5B) is decoded from its bitmap header.

## Firmware update

Use the ready-made tools: `hihost update` (serial) or `canhost firmware update`
(CAN) in `c/tools`. To embed an updater in your own program, see the API in
`hipnuc_kboot.h` and `hipnuc_can_update.h`.

## Constraints

8-bit bytes, IEEE 754 floats, 8-byte `double` (a 4-byte `double` rejects HI83
frames with positions). Host byte order does not matter. Headers are C++
safe. One context must own each decoder; feeding from an interrupt is fine.

Tests: `c/tests` (CMake + ctest), including a copied-files project and a C++
consumer.
