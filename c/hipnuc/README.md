# HiPNUC C core

[English](README.md) | [中文](README_zh.md)

Small C99 decoders for current HiPNUC products (firmware 1.6.9 or later).
No heap, OS or global mutable state. Decoders do not use `stdio`.
For desktop serial connections and C/C++ examples, start at [C SDK](../README.md).

## Copy the files you need

Each row lists all required files; binary, NMEA and CAN decoding are independent.

| Purpose | Files |
| --- | --- |
| Serial binary HI91/HI81/HI83, including SI samples | `hipnuc_dec.c/.h`, `hipnuc_sample.c/.h` |
| NMEA GGA/RMC, including SI samples | `nmea_dec.c/.h`, `hipnuc_sample.c/.h` |
| CAN J1939/CANFD83, including SI samples | `hipnuc_j1939.c/.h`, `hipnuc_can_frame.h`, `hipnuc_sample.c/.h` |
| JSON formatting only (uses `stdio`) | `hipnuc_json.c/.h`, `hipnuc_sample.c/.h` |
| Serial firmware update | `hipnuc_kboot.c/.h`, `hipnuc_dec.c/.h`, `hipnuc_sample.c/.h` |
| CAN firmware update | `hipnuc_can_update.c/.h`, `hipnuc_can_frame.h` |

Or link a CMake target:

```cmake
add_subdirectory(path/to/c/hipnuc hipnuc)
target_link_libraries(my_app PRIVATE hipnuc_core)
# Other targets: hipnuc_json, hipnuc_j1939, hipnuc_update
```

## Receive bytes

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

`hipnuc_input()` returns `1` for a complete supported frame, `0` while
waiting, and `-1` for an invalid frame. Continue feeding after an error.
Each outer binary frame must contain exactly one supported sub-packet.
NMEA uses `nmea_input()` and `hipnuc_sample_from_nmea()` in the same way.
For MCU applications, receive into a buffer in the interrupt and decode in
the main loop; the [STM32 examples](../../stm32/README.md) show this arrangement.

For CAN, pass one `hipnuc_can_frame_t` from your CAN driver to
`hipnuc_j1939_parse()`. A positive result is a new sample. Check `node_id`
when sharing a bus. Each sample contains only the current frame's fields;
there is no automatic combination of different PGNs.

## Use the measurement fields

`hipnuc_sample_t.valid` is a 64-bit **availability** mask. A missing field's
stored zero is not a measurement. Roll/pitch and yaw, horizontal position
and altitude, and heave displacement and frequency have separate flags.
INS position and raw GNSS position also have separate fields. Check GNSS
quality or RMC status/mode independently before treating a position as a fix.

Units and coordinate conventions are in `hipnuc_sample.h`. The SDK preserves
the device coordinate configuration. HI91 acceleration uses the firmware's
9.8 m/s² per G. HI81 reports heading, not Euler yaw. A set `WB_CONV` or
`ATT_CONV` status bit means **not converged**; use the derived convergence
fields when `HIPNUC_VALID_STATUS` is present.

The decoders require 8-bit bytes and IEEE 754 floats. Binary64 position
fields require 8-byte `double`; unsupported layouts and HI83 bitmap bits
are rejected. Headers can be included from C++.

Firmware update tools are documented under [tools](../tools/README.md).
To embed an updater, implement the callbacks in `hipnuc_kboot.h` or
`hipnuc_can_update.h`. CAN update is the only use of CANopen SDO.
