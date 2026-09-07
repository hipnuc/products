# Python API and CLI quick reference

Import from `hipnuc`. Importing opens no ports and prints nothing. All I/O is
synchronous; use `with` or call `open()` / `close()`. Library failures raise
exceptions; only the CLI decides exit status.

Supported devices: HiPNUC products running firmware 1.6.9 or later
(HI01–HI06, HI12–HI18, HI32, HI70/HI71 and the CH0X0 series). Serial messages
HI91, HI81 and HI83 plus the `$GPGGA` / `$GPRMC` sentences are decoded.

## Connect and read

```python
from hipnuc import SerialDevice

with SerialDevice() as device:  # find the device automatically
    print(device.port, device.baudrate)
    for sample in device.iter_samples():
        print(sample.type, sample.acceleration_m_s2, sample.euler_rad)
```

`SerialDevice("COM3", baudrate=115200)` opens a known connection directly.
When either parameter is omitted the SDK scans the serial ports and baudrates
and needs exactly one HiPNUC device; the error message names the candidates
or the reason nothing was found.

| `SerialDevice` | Contract |
| --- | --- |
| `SerialDevice(port=None, baudrate=None, timeout=2.0, *, scan_timeout=30.0, raw_sink=None, sample_sink=None, queue_size=4096)` | Construct without I/O |
| `read(timeout=None)` | Next sample, or `ResponseTimeout` when the link stays idle |
| `iter_samples(idle_timeout=None)` | Samples until closed |
| `read_info()` | `LOG VERSION` as `DeviceInfo` (product, firmware, serial number) |
| `command(text, timeout=None, response="auto")` | One ASCII command; waits for `OK` (`response="none"` only sends) |
| `save_config()` | `SAVECONFIG` |
| `set_baudrate(baud, device_port=None, recovery_timeout=5, save=False)` | `SERIALCONFIG`, switch host speed, query identity |
| `reboot(recovery_timeout=5)` | `REBOOT`, then wait for `LOG VERSION` to answer |
| `dropped_samples` | Samples discarded because the read queue overflowed |

`sample_sink` receives every decoded sample (also during commands) and `raw_sink`
every received byte chunk; both run inline in the reading thread, so keep them
fast and never call device I/O from them. The device prints nothing for an unknown
command, so a typo surfaces as `ResponseTimeout`.

Send the product's own commands and save once after a batch:

```python
with SerialDevice("COM3", baudrate=115200) as device:
    device.command("LOG HI91 ONTIME 0.01")
    print(device.command("LOG COMCONFIG").text)
    device.save_config()
```

## Sample

`Sample(type, values, raw, received_time_ns, complete, issues, metadata)` is one
decoded message. Common fields are properties returning `None` when absent:

| Property | Unit / convention |
| --- | --- |
| `acceleration_m_s2`, `angular_velocity_rad_s`, `magnetic_field_t` | Body axes; m/s², rad/s, T |
| `euler_rad` | Roll, pitch, yaw in rad (device Euler convention) |
| `quaternion_wxyz` | Body-to-navigation, WXYZ |
| `roll_rad`, `pitch_rad`, `heading_rad` | INS attitude; heading is clockwise from north |
| `latitude_deg`, `longitude_deg`, `altitude_msl_m`, `geoid_separation_m` | WGS84 |
| `velocity_enu_m_s` | East, north, up |
| `heave_surge_sway_m`, `heave_surge_sway_hz` | MRU |
| `pressure_pa`, `temperature_c`, `inclination_rad` | |

`values` holds every decoded field, including `main_status`, `status_flags`
(names of the set MAIN_STATUS bits; every listed flag is a warning: `ATT_CONV`
and `WB_CONV` mean the attitude or gyro bias has *not* converged yet,
`MAG_DIST` magnetic disturbance, `UTC_UNSYNC` device time not synchronized,
`STATIC` device detected as static), `ins_status` / `ins_status_name`
(`aligning`, `navigating`, `dead_reckoning`), `device_time_s`, `utc` and GNSS
quality fields.
`sample.to_dict()` is JSON-compatible (`allow_nan=False` safe). `received_time_ns`
is host wall-clock time at the end of the serial read.

HI91 acceleration arrives in G and is converted with the product convention
1 G = 9.8 m/s². Angles are device-configured (ENU/312 by default).

## Decoder

```python
from hipnuc import Decoder

decoder = Decoder()
samples = decoder.feed(chunk)  # any byte chunk; partial frames are retained
samples += decoder.finish()  # at end of a file
```

The decoder handles binary, NMEA and ASCII replies in one stream, verifies
CRC-16 and checksums, and exposes `statistics` (frames, CRC errors, noise).
Malformed frames are counted and skipped; `complete=False` marks a sample whose
HI83 bitmap contains undecoded bits.

## Recorder

```python
from hipnuc import Recorder, SerialDevice

with (
    Recorder("samples.jsonl", raw_path="capture.bin") as recording,
    SerialDevice(
        "COM3", baudrate=115200, sample_sink=recording.write, raw_sink=recording.write_raw
    ) as device,
):
    for sample in device.iter_samples():
        pass
```

`Recorder(jsonl_path=None, *, raw_path=None, overwrite=False)` writes one
`sample.to_dict()` JSON object per line and/or the exact received bytes. Existing
files are protected unless `overwrite=True`; files are flushed every second.

## Errors

`HipnucError` is the base class. `TransportError` (cannot open/read/write the
port, with a hint such as closing CHCenter or joining `dialout`),
`ResponseTimeout` (no data or no reply), `DeviceError` (`ERR` reply or Modbus
exception) and `VerificationError` (readback mismatch). Invalid arguments raise
`ValueError`.

## CLI

`hipnuc` and `python -m hipnuc` are the same program. Connection options go after
the final command.

| Command | Purpose |
| --- | --- |
| `list` | Serial ports on this computer |
| `scan [-p PORT] [-b BAUD]` | Find HiPNUC devices |
| `info` | Product, firmware and serial number |
| `read [--duration S] [--record FILE] [--record-raw FILE] [--jsonl] [--quiet]` | Read continuously |
| `command "TEXT" \| --file FILE [--save] [--reboot] [--no-reply]` | Send ASCII commands |
| `baudrate NEW_BAUD [--save]` | Change the device baudrate and reconnect |
| `reboot [--save]` | Restart and reconnect |
| `modbus ...` | RTU operations, see [modbus.md](modbus.md) |

Options: `-p/--port`, `-b/--baudrate` (host side only), `--timeout` (default 2 s),
`--scan-timeout` (default 30 s), `--json` for machine-readable output. `info` and
`read` discover omitted connection parameters; `command`, `baudrate` and `reboot`
require `-p`. Exit status: 0 success, 1 runtime failure, 2 usage error, 130 Ctrl-C.

Command files contain one command per line; `#` and `;` start comments.
`--save` sends `SAVECONFIG` after all commands succeed and `--reboot` restarts and
waits for the device. Do not put raw `SERIALCONFIG` or `REBOOT` in a file together
with these options; use the `baudrate` and `reboot` commands instead.

Command names and parameters are documented in the product's command and
programming manual.
