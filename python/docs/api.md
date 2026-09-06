# Python API and CLI reference

Import from `hipnuc`. Importing opens no ports, starts no threads, prints nothing,
and does not configure global logging. All I/O is synchronous. Use a context
manager or call `open()` / `close()` explicitly. Library failures raise exceptions;
only the CLI decides exit status and presentation.

## Start with a device

```python
from hipnuc import SerialDevice

with SerialDevice() as device:
    print(device.port, device.baudrate)
    sample = device.read()
    print(sample.acceleration_m_s2)
```

Omit the connection parameters to find a unique HiPNUC device, or use
`SerialDevice("COM3", baudrate=115200)` to open a known connection directly.
Constructing the object does not open a port. `open()` or entering `with`
establishes the connection; leaving `with` closes it, including on an exception.
Call `device.read_info()` when you need the device identity; reading samples
does not depend on that query succeeding.

`read()` returns a sample or raises `ResponseTimeout`; a disconnected or
inaccessible port raises `TransportError`. `iter_samples()` provides the same
behavior in a loop. Set a longer `timeout` when the device intentionally outputs
less frequently than once every two seconds.

## Data models

`Sample(type, values, raw, received_time_ns=None, complete=True, issues=(), metadata={})`
represents one decoded message. `values` contains named protocol measurements;
fields absent from that message are absent, and invalid/non-finite values become
`None` with issues where identifiable. Measurements and their provenance are
accessible through the same dataclass for serial and Modbus. The common fields
below are explicit, read-only properties: use `sample.acceleration_m_s2` rather
than dictionary lookups. Scalars and vectors can be `None` when absent; invalid
vector components can also be `None`. `values` remains the underlying storage
and provides access to protocol-specific fields, such as status flags.

| Common field | Unit / convention |
| --- | --- |
| `acceleration_m_s2` | Three device-configured body axes, m/s² |
| `angular_velocity_rad_s` | Three device-configured body axes, rad/s |
| `magnetic_field_t` | Tesla |
| `euler_rad` | Roll, pitch, yaw; sequence depends on device coordinate configuration |
| `quaternion_wxyz` | WXYZ, body to navigation rotation |
| `roll_rad`, `pitch_rad`, `heading_rad` | INS attitude; heading is separate from IMU Euler yaw |
| `latitude_deg`, `longitude_deg` | Degrees; inspect position status before use |
| `altitude_msl_m`, `geoid_separation_m` | Metres; separate quantities |
| `velocity_enu_m_s` | East, north, up in m/s |
| `heave_surge_sway_m`, `heave_surge_sway_hz` | MRU vector ordered heave, surge, sway |
| `pressure_pa`, `temperature_c` | Pa, °C |
| `inclination_rad` | Two independent inclination angles; not necessarily Euler roll/pitch |

HI91 and Modbus encode acceleration in the product's G unit. The SDK uses
1 G = 9.8 m/s² to recover the same physical quantity that HI83 sends directly
in m/s².

The SDK preserves the device axis configuration. ENU uses the device's 312 Euler
convention and NWU uses 321; a decoder receiving bytes alone cannot discover the
configuration. Read public configuration and retain it alongside recordings.
An INS heading, course over ground, and Euler yaw are distinct measurements.

`complete` concerns field coverage, not the sensor's operating state. Check
`main_status`/`status_ext`, GNSS quality and `issues` as relevant. GGA/RMC expose
`fix_valid`; unavailable coordinates are not replaced with a zero position.
SXT's GNSS quality must be interpreted with its separate `ins_status`: loss of
GNSS does not by itself prove an INS dead-reckoned position is invalid.

`sample.to_dict(include_raw=False)` returns a flat JSON-compatible copy. Date,
time and datetime become ISO strings; UTC has `Z`; NaN/Infinity become `null`.
`json.dumps(sample.to_dict(), allow_nan=False)` is supported. Use
`include_raw=True` to include `raw_hex`. `raw` is the entire binary frame/NMEA
sentence, or big-endian register bytes for Modbus (see `metadata.raw_format`).
Several binary subpackets can share the same outer `raw` frame; `payload_offset`
identifies each subpacket. Modbus `raw` does not include the RTU envelope.

Time is deliberately separated:

- `received_time_ns`: host wall-clock time at completion of the serial read or
  Modbus measurement response. It can jump with OS clock adjustment; it is not
  a precise per-sample hardware timestamp. One input batch may share this time.
- `device_time_*`: native or converted device counter, with reference and layout
  in metadata. HI83 historic milliseconds and current microseconds remain
  identifiable. Modbus CPUTIME wraps as a 32-bit millisecond counter.
- `utc` / `utc_time` / `utc_date`: device-reported UTC only. GGA has time of day,
  not a date. The decoder never uses today's date to invent a full timestamp.
  Invalid or unsynchronized binary UTC remains unavailable.

`DeviceInfo` holds optional product, firmware, bootloader, serial number, build,
and `raw_response`. Three-digit firmware codes are normalized to dotted form
(`172` → `1.7.2`), while unknown strings remain usable. Identity/version does not
uniquely identify all historic firmware behavior.

## Decoder

```python
decoder = Decoder(max_payload_size=506, max_line_size=1024, max_pending_lines=128)
samples = decoder.feed(chunk)
samples_at_eof = decoder.finish()
```

`feed` accepts bytes, bytearray or memoryview. It handles partial frames, combined
frames and binary/NMEA/ASCII mixing. HI91/HI81 are fixed length; HI83 uses bitmap
and exact remaining payload length to select its historic timestamp layout.
HI83 must be the last subpacket in its outer frame. Unknown bitmap fields preserve
a reliable prefix and mark `complete=False`; offsets beyond them are not guessed.

The default 506-byte payload bound matches the C SDK's 512-byte total buffer.
It is a configurable resource bound, not the wire format's 16-bit maximum.
Text, pending ASCII lines and incomplete input are bounded. A plausible incomplete
binary frame waits for its announced length; `finish()` at EOF can recover a
following valid frame after truncation. A live session should use a finite read
timeout and may explicitly reset/reopen after a stalled corrupt stream.

`statistics` exposes bytes, successful frames/samples, CRC/length/NMEA errors,
malformed packets, noise and dropped ASCII lines. `buffered_bytes` reports
undecoded bytes. `reset()` drops pending input but preserves statistics.
`drain_lines()` returns complete non-NMEA ASCII lines; `discard_partial=True`
also discards a partial ASCII response, preserving partial binary/NMEA input.

After binary corruption, ASCII is quarantined until a valid checksummed binary
or NMEA frame establishes a boundary, or the caller resets the decoder. This
prevents binary payload text from becoming a false command ACK. If only ASCII
follows such corruption, a command may time out. Reset the decoder or reopen the
serial session to resume ASCII-only communication.

## SerialDevice

`SerialDevice(port=None, baudrate=None, timeout=2.0, *, scan_timeout=30.0,
decoder=None, raw_sink=None, sample_sink=None, queue_size=4096)` owns a single
serial port and serializes reads/commands with a lock. Opening does not alter
output settings. Explicit port and baudrate values are never replaced; only
missing connection parameters are discovered. The library prints no discovery
messages. After opening, `port` and `baudrate` hold the selected connection.

Read and command timeouts are enforced by the SDK without changing UART settings
during reception. A command's `timeout` override limits sending and waiting for
its response; the
transport write timeout is set to the constructor's `timeout` when opening.
Before the first command after opening, an existing stream is read to a complete
boundary while preserving its samples. Silent devices proceed after a bounded
observation window: the greater of 50 ms or one maximum binary frame's wire time,
capped by the command timeout. The command deadline starts after this observation
and includes the send time; blocking driver writes also obey the transport limit.

| Method / property | Contract |
| --- | --- |
| `read(timeout=None)` | Next pending sample; raise `ResponseTimeout` when no sample arrives within the timeout |
| `iter_samples(idle_timeout=None)` | Yield samples until closed; raise on idle timeout or transport failure |
| `read_info()` | Query `LOG VERSION`; require recognizable identity |
| `command(text, timeout=None, response="auto")` | One ASCII line; `CommandResult` |
| `save_config()` | Explicit SAVECONFIG, once at the end of a batch |
| `set_baudrate(baudrate, device_port=None, recovery_timeout=5, save=False)` | Change the connected port, verify identity at new speed; optional save |
| `reboot(recovery_timeout=5)` | Send once, wait, recover same identity; no automatic save |
| `dropped_samples` | Count of oldest pending samples discarded on queue overflow |

Omitting the read or idle timeout uses the device's `timeout`. Send the commands
documented for your product directly, then save once after the batch:

```python
with SerialDevice("COM3", baudrate=115200) as device:
    print(device.command("LOG VERSION").text)
    device.command("LOG HI91 ONTIME 0.01")
    print(device.command("LOG COMCONFIG").text)
    device.save_config()
```

Use messages and output periods supported by your model and serial bandwidth.
Use the product's query commands to read back settings. A failed command raises
an exception and stops this sequence; previously applied changes are not undone.
If the settings require a restart, call `reboot()` after `save_config()`.

Sinks execute inline and must be fast. `raw_sink` sees every received byte chunk;
`sample_sink` sees every decoded sample including those received during commands.
The pending read queue is separate and bounded; an application that records via
`sample_sink` still receives samples when its read queue overflows. Slow storage
or callbacks can still overrun OS/hardware buffers; Python cannot promise lossless
physical acquisition under arbitrary load. Never call device I/O from a sink.

`response="auto"` accepts terminal `OK` or matching `LOG [COMn] <message> ONMARK
ONCE` data. `"ack"` requires `OK`; `"text"` accepts nonempty text after an idle
gap; `"none"` only reports sending, with `acknowledged=False`. Use the latter two
only when the device command genuinely has that response contract.

Error lines are terminal `ERR`/`ERROR`/`FAIL`, not substring searches. Binary bytes
cannot acknowledge a command. A short, bounded ASCII tail drain absorbs the
known legacy duplicate ACK. ASCII has no transaction ID: an arbitrarily delayed
old response cannot always be attributed. Following a timeout, inspect/reopen
and read identity/configuration before relying on a subsequent write.
`CommandResult.acknowledged` and `verified` are distinct; raw `OK` does not prove
flash persistence or a physical effect. Interpret query replies using your
product's documented fields and compare settings as needed by your application.

Managed reboot/baud recovery retries the original port name, including reopening
a USB handle. A renamed USB port requires reopening with its new path. Identity
recovery proves communication with that device, not every physical reset effect.
For another device COM port, use raw `SERIALCONFIG COMn ...` and verify that port;
the managed helper is for the connected port.

### Discovery

`discover(ports=None, *, baudrates=BAUDRATES, timeout=2.0, scan_timeout=30.0)`
returns a `DiscoveryResult`. It first observes incoming data and sends
`LOG VERSION`. If a valid HiPNUC binary stream is present but identity times out,
it retries that read-only query once at the same baudrate, within `scan_timeout`.
This handles device input left by a previous incorrect baudrate. Other commands
are not automatically repeated. All discovery ports are closed before returning.
Exact POSIX path spelling is preserved.

Discovery reports each port/baudrate attempt, identity retry and outcome through
the standard `hipnuc.serial_device` logger at `INFO` level. The library installs
no handlers. The CLI shows these messages on stderr, including during automatic
connections; JSON output on stdout is unchanged.

- `devices`: `DiscoveredDevice` objects containing `port`, `baudrate`, optional
  `info`, recognized `protocol`, and optional `identity_error`.
- `errors`: a diagnostic for each unsuccessful port.
- `complete`: whether the search completed within its total time budget.

Valid HiPNUC binary data can identify a readable stream without a successful
identity response; a generic NMEA sentence alone does not identify the vendor.
When several baudrates deliver data, a completed identity exchange takes priority
over a data-only candidate. Buffered USB data can otherwise suggest a stale speed.
Automatic opening requires a complete search with exactly one matching device.
No match, multiple matches or an incomplete search produces an actionable error;
specify the port and baudrate when known. `device.discovery_result` retains the
report when automatic selection was used. Explicit port plus baudrate opens
directly, without requiring an identity query.

Do not use ASCII discovery on a multi-node RS-485 bus. Modbus uses an explicit
port and addressed requests instead.

## Recorder

`Recorder(jsonl_path=None, *, raw_path=None, overwrite=False)` writes decoded
samples and/or original received bytes. Supply at least one output path and use
it as a context manager, or call `open()` / `close()` explicitly. Construction
performs no I/O. A closed writer cannot be reopened; create a new instance for a
new recording.

| Method / property | Contract |
| --- | --- |
| `open()` | Open the configured outputs; called automatically by `with` |
| `write(sample)` | Write one complete `sample.to_dict()` JSONL record |
| `write_raw(data)` | Write the exact received bytes to the raw output |
| `flush()` | Flush the configured outputs |
| `close()` | Flush and close the outputs; safe to call again |
| `samples_written` | Number of JSONL records written |
| `raw_bytes_written` | Number of raw bytes written |

Existing files are protected unless `overwrite=True`; JSONL and raw outputs must
be different files. Output setup fails before opening a device in the example
below. Files are flushed at approximately one-second intervals during writes
and when closed. This is buffered file I/O, not a guarantee against power loss.
File errors propagate immediately so an application cannot silently continue
recording to a failed destination.

For simple application loops, call `recording.write(sample)` for each sample.
To record every received sample independently of how quickly the application
consumes its queue, attach the writer to the receive callbacks:

```python
from hipnuc import Recorder, SerialDevice

with Recorder("samples.jsonl", raw_path="capture.bin") as recording:
    with SerialDevice(
        "COM3",
        baudrate=115200,
        sample_sink=recording.write,
        raw_sink=recording.write_raw,
    ) as device:
        for sample in device.iter_samples():
            print(sample.acceleration_m_s2)
```

The callbacks write inline, without a background thread. Do not also call
`recording.write(sample)` in that loop: the callback already recorded it.
Raw recording contains noise, bad frames and command replies as received.
Concatenating `sample.raw` is not a substitute: several subpackets may share one
outer frame, and undecoded bytes would be missing.

For Modbus, call `recording.write(device.read_sample())`; each record retains
`metadata.device_id`. Modbus sample raw data consists of register bytes, not an
RTU bus capture. Raw Modbus recording is not provided.

## CLI

`hipnuc` and `python -m hipnuc` run the same CLI. The latter uses the selected
Python interpreter directly. Place connection options after the final command:
`hipnuc command "LOG VERSION" -p COM3 -b 115200`.

| Command | Purpose |
| --- | --- |
| `list` | Enumerate OS serial ports without opening them; `--json` is available |
| `scan` | Discover HiPNUC devices; optionally restrict with `-p` and/or `-b` |
| `info` | Query identity |
| `read` | Read continuously, or stop after `--duration SECONDS` |
| `command "TEXT"` | Send one raw ASCII command; alternatively use `--file PATH` |
| `baudrate NEW_BAUD` | Change and verify the connected device port's baudrate |
| `reboot` | Restart the connected device and recover communication |
| `modbus ...` | Addressed RTU operations; see the [Modbus guide](modbus.md) |

Serial commands share `-p/--port`, `-b/--baudrate`, `--timeout` (default 2 seconds)
and `--scan-timeout` (default 30 seconds). `-b` is always the host connection
baudrate; it never changes the device setting on its own.

- `info` and `read` discover omitted connection parameters.
- `baudrate`, `reboot` and `command` require `-p`, including
  raw query commands and command files. If `-b` is omitted, discovery checks only
  that port. Use `scan` to find an unknown port.
- Supplying both `-p` and `-b` connects directly. A failed connection does not
  trigger a search for another device. Use this form for repeated CLI calls.

Discovery prints progress and the selected connection to stderr. CLI invocations
do not remember previous connections. In application code, specify the port and
baudrate and reuse one `with SerialDevice(...)` session for multiple operations.
The Python API still permits automatic discovery when opening without these
parameters; individual device methods do not initiate another search.

```sh
hipnuc command "LOG VERSION" -p COM3 -b 115200
hipnuc command --file commands.txt -p COM3 -b 115200 --save --reboot
hipnuc baudrate 115200 -p COM3 -b 9600 --save
```

The last command connects at 9600 and changes the device to 115200. Use
`--help` on the corresponding command for save and recovery options.

Command files are UTF-8 with one command per line; blank lines and lines starting
with `#` or `;` are ignored. Raw commands use the same response contracts as
`SerialDevice.command()`. For example, `commands.txt` can contain:

```text
LOG HI91 ONTIME 0.01
LOG COMCONFIG
```

Human-readable replies appear after each command. `--json` emits one JSON value
after completion: a result object for a single command, or a list when using a
file, `--save` or `--reboot`. Execution stops at the first failure;
earlier changes remain applied, and no requested save or restart follows a failure.
On failure, stderr identifies the command's position and reason; stdout contains
no partial JSON value.

Saving and restarting are explicit: `--save` sends `SAVECONFIG` once after all
commands succeed; `--reboot` then restarts and waits for communication to recover.
Use both when settings need persistence and a restart. Neither an ACK nor restored
communication proves that every setting has the desired physical effect; include
the product's readback commands where needed. `command "SAVECONFIG"` can also
save a previously completed configuration session.

`--save` / `--reboot` require responses; they cannot be used with `--response none`.
Do not combine raw `SERIALCONFIG`, `REBOOT` or `FRESET` with `--save` / `--reboot`.
Those commands can change communication before the follow-up action. For the
connected port, use `baudrate NEW_BAUD --save` or `reboot --save` so the SDK handles
connection recovery. Raw commands without these options remain available.

For application code, edit `COMMANDS` in
[send_commands.py](../examples/send_commands.py) and run
`python examples/send_commands.py` from `python/`. Its defaults only query data;
it does not append a save command. The scripts use constants at the top of each
file instead of argument parsers.

Command names, model-specific parameters and effects are documented in the
official [IMU manual](https://download.hipnuc.com/en/products/imu/cum.html) and
[INS manual](https://download.hipnuc.com/en/products/ins/cum.html).

### Reading and recording options

- `--record PATH`: JSONL with every decoded sample, including its timestamps,
  parse state and protocol metadata.
- `--record-raw PATH`: every serial RX byte, independently of successful decoding.
  Can be combined with `--record`; unavailable for Modbus.
- `--overwrite`: allow replacement of existing recording files.
- `--jsonl`: complete JSONL on stdout; diagnostics remain on stderr.
- `--display-rate HZ`: set a positive human display limit per message type;
  default 5 Hz. It does not limit JSONL or file recording. Use `--jsonl` to
  print every sample.
- `--quiet`: hide measurements while retaining diagnostics and recording.
- `--duration SECONDS`: stop after a finite acquisition interval. Modbus also
  supports `--count` and `--interval`; the first reached stop condition wins.
  The current serial receive batch or Modbus poll finishes before stopping;
  Modbus does not start another poll after the deadline. Counts refer to
  completed samples, so an in-progress transaction can finish after the duration.

The human display converts angles to degrees and angular velocity to degrees/s,
with units shown. JSONL and API fields retain their documented SI units. Each
displayed line describes one sample, rather than combining fields of different
ages. Files close on normal completion, Ctrl-C and errors; stderr reports a short
acquisition summary.

CLI exit status: `0` for successful completion, `1` for runtime failures,
`2` for usage errors, and `130` for Ctrl-C. A finite acquisition with no samples
is a failure. Use `hipnuc`, `hipnuc help`, or a command's `--help` for navigation.

## Modbus and errors

See [Modbus API and operations](modbus.md). `ModbusBus`, `ModbusDevice` and
`WriteResult` are exported by `hipnuc`. PyModbus is contained in `modbus.py` and
constrained to the tested 3.15 minor version.

`HipnucError` is the library base exception. `TransportError` covers serial/RTU
transport failures, `ResponseTimeout` a missing response, `DeviceError` a device
rejection (`code` and raw `response` where available), and `VerificationError`
an inconsistent readback/identity. Invalid API arguments raise `ValueError`.
Context managers close resources on these failures. Applications decide retry
and logging policy; writes are not blindly retransmitted.
