"""CLI for the public SDK. Machine data goes to stdout, diagnostics to stderr."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import ExitStack, contextmanager
from dataclasses import asdict
from functools import partial, wraps
import json
import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

import click
from serial.tools import list_ports

from ._connection import discovery_error_summary, is_usb_port
from .errors import HipnucError, ResponseTimeout, TransportError
from .models import Sample
from .modbus import ModbusBus, ModbusDevice
from .recording import Recorder
from .serial_device import BAUDRATES, SerialDevice, discover


def _json(value) -> str:
    return json.dumps(
        value.to_dict() if hasattr(value, "to_dict") else value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )


def _show(value, as_json: bool) -> None:
    if as_json:
        click.echo(_json(value))
    elif hasattr(value, "text"):
        click.echo(value.text or "Sent (no response requested).")
        if value.verified:
            click.echo("Readback verified.")
    else:
        fields = value.to_dict() if hasattr(value, "to_dict") else value
        for key, item in fields.items():
            if item is not None and key != "raw_response":
                click.echo(f"{key}: {item}")


def _errors(func):
    @wraps(func)
    def wrapped(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except KeyboardInterrupt:
            click.echo("Interrupted.", err=True)
            raise click.exceptions.Exit(130) from None
        except ValueError as exc:
            raise click.UsageError(str(exc)) from exc
        except (HipnucError, OSError) as exc:
            raise click.ClickException(str(exc)) from exc

    return wrapped


def _finite(_ctx, param, value):
    if value is not None and not math.isfinite(value):
        raise click.BadParameter("must be finite", param=param)
    return value


def _timeout_option(func):
    return click.option(
        "--timeout",
        type=click.FloatRange(min=0, min_open=True),
        callback=_finite,
        default=2.0,
        show_default=True,
        help="Response/idle timeout in seconds.",
    )(func)


def _serial_options(func=None, *, require_port=False):
    if func is None:
        return partial(_serial_options, require_port=require_port)
    func = _timeout_option(func)
    func = click.option(
        "--scan-timeout",
        type=click.FloatRange(min=0, min_open=True),
        callback=_finite,
        default=30.0,
        show_default=True,
        help="Total automatic discovery time limit in seconds.",
    )(func)
    func = click.option(
        "-b",
        "--baudrate",
        type=click.IntRange(min=1),
        help="Host baudrate; auto-detect if omitted. Supply -p and -b to skip discovery.",
    )(func)
    return click.option(
        "-p",
        "--port",
        required=require_port,
        help=(
            "Target serial port, e.g. COM3 or /dev/ttyUSB0. Use 'hipnuc scan' to find it."
            if require_port
            else "COM3, /dev/ttyUSB0, etc.; auto-select a USB serial port if omitted."
        ),
    )(func)


def _modbus_options(func):
    func = _timeout_option(func)
    func = click.option(
        "--id", "device_id", type=click.IntRange(1, 247), default=80, show_default=True
    )(func)
    func = click.option(
        "-b", "--baudrate", type=click.IntRange(min=1), default=115200, show_default=True
    )(func)
    return click.option("-p", "--port", required=True, help="Serial port of the RTU bus.")(func)


def _record_options(func):
    func = click.option("--overwrite", is_flag=True, help="Replace existing recording files.")(func)
    func = click.option("--quiet", is_flag=True, help="Collect without printing samples.")(func)
    func = click.option("--jsonl", is_flag=True, help="Print every sample as SI JSONL.")(func)
    func = click.option(
        "--display-rate",
        type=click.FloatRange(min=0, min_open=True),
        callback=_finite,
        default=5.0,
        show_default=True,
        help="Human display limit per message type, in Hz.",
    )(func)
    func = click.option(
        "--duration",
        type=click.FloatRange(min=0, min_open=True),
        callback=_finite,
        help="Acquisition time in seconds; finish the current batch or Modbus poll before stopping.",
    )(func)
    return click.option(
        "--record",
        type=click.Path(path_type=Path, dir_okay=False),
        help="Write every sample as SI JSONL.",
    )(func)


def _integer(value: str) -> int:
    try:
        return int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as exc:
        raise click.BadParameter("expected a decimal integer or 0x-prefixed hexadecimal") from exc


@contextmanager
def _discovery_progress() -> Iterator[None]:
    """Show SDK discovery progress only while this CLI invocation owns it."""
    logger = logging.getLogger("hipnuc.serial_device")
    handler = logging.StreamHandler(sys.stderr)
    previous_level, previous_propagate = logger.level, logger.propagate
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False
    try:
        yield
    finally:
        logger.removeHandler(handler)
        handler.close()
        logger.setLevel(previous_level)
        logger.propagate = previous_propagate


@contextmanager
def _serial_connection(
    port: str | None,
    baudrate: int | None,
    timeout: float,
    scan_timeout: float,
) -> Iterator[SerialDevice]:
    """Own one CLI connection and route diagnostics to stderr."""
    if port is None or baudrate is None:
        message = (
            f"Detecting baudrate on {port}"
            if port is not None
            else "Searching USB serial ports for HiPNUC devices"
        )
        click.echo(f"{message} (Ctrl-C to cancel)...", err=True)
    with (
        _discovery_progress(),
        SerialDevice(
            port,
            baudrate,
            timeout,
            scan_timeout=scan_timeout,
        ) as device,
    ):
        identity = ""
        if device.info is not None:
            identity = f" — {device.info.product_name or 'HiPNUC'}"
            if device.info.firmware_version:
                identity += f" / firmware {device.info.firmware_version}"
        click.echo(f"Connected to {device.port} at {device.baudrate} baud{identity}.", err=True)
        if device.discovery_result is not None:
            selected = device.discovery_result.devices[0]
            if selected.identity_error:
                click.echo(
                    f"Using valid HiPNUC data; identity unavailable: {selected.identity_error}",
                    err=True,
                )
        yield device


def _with_serial(func):
    @wraps(func)
    def wrapped(*args, **kwargs):
        connection = {
            name: kwargs.pop(name) for name in ("port", "baudrate", "timeout", "scan_timeout")
        }
        with _serial_connection(**connection) as device:
            return func(device, *args, **kwargs)

    return wrapped


@contextmanager
def _modbus_connection(
    port: str, baudrate: int, timeout: float, device_id: int
) -> Iterator[ModbusDevice]:
    """Own the RTU bus for this command and select its addressed device."""
    with ModbusBus(port, baudrate, timeout) as bus:
        device = bus.device(device_id)
        click.echo(f"Connected to {port} at {baudrate} baud, Modbus ID {device_id}.", err=True)
        yield device


def _with_modbus(func):
    @wraps(func)
    def wrapped(*args, **kwargs):
        connection = {
            name: kwargs.pop(name) for name in ("port", "baudrate", "timeout", "device_id")
        }
        with _modbus_connection(**connection) as device:
            return func(device, *args, **kwargs)

    return wrapped


@contextmanager
def _stop_on_interrupt() -> Iterator[threading.Event]:
    """Finish recording the current input batch before honoring Ctrl-C."""
    stopped = threading.Event()
    installed = threading.current_thread() is threading.main_thread()
    previous = signal.getsignal(signal.SIGINT) if installed else None
    if installed:
        signal.signal(signal.SIGINT, lambda *_args: stopped.set())
    try:
        yield stopped
    finally:
        if installed:
            signal.signal(signal.SIGINT, previous)


def _human_sample(sample: Sample) -> str:
    """Render this sample only; human angles are degrees, API values stay SI."""
    parts = [sample.type]
    if "device_id" in sample.metadata:
        parts.append(f"id={sample.metadata['device_id']}")
    elif "node_id" in sample.values:
        parts.append(f"id={sample.values['node_id']}")
    flags = sample.values.get("status_flags")
    if flags:
        parts.append("[" + " ".join(flags) + "]")
    if sample.values.get("ins_status_name"):
        parts.append(f"ins={sample.values['ins_status_name']}")

    def number(value, scale=1.0):
        return "—" if value is None else f"{value * scale:.3f}"

    def vector(label, value, unit, scale=1.0):
        if value is not None:
            parts.append(f"{label}=[{', '.join(number(v, scale) for v in value)}] {unit}")

    def scalar(label, value, unit, scale=1.0, precision=3):
        if value is not None:
            parts.append(f"{label}={value * scale:.{precision}f} {unit}")

    vector("acc", sample.acceleration_m_s2, "m/s²")
    vector("gyr", sample.angular_velocity_rad_s, "°/s", 180 / math.pi)
    vector("mag", sample.magnetic_field_t, "µT", 1e6)
    vector("roll/pitch/yaw", sample.euler_rad, "°", 180 / math.pi)
    vector("inclination", sample.inclination_rad, "°", 180 / math.pi)
    vector("quaternion WXYZ", sample.quaternion_wxyz, "")
    scalar("roll", sample.roll_rad, "°", 180 / math.pi)
    scalar("pitch", sample.pitch_rad, "°", 180 / math.pi)
    scalar("heading", sample.heading_rad, "°", 180 / math.pi)
    scalar("latitude", sample.latitude_deg, "°", precision=7)
    scalar("longitude", sample.longitude_deg, "°", precision=7)
    scalar("altitude MSL", sample.altitude_msl_m, "m")
    scalar("geoid separation", sample.geoid_separation_m, "m")
    vector("velocity ENU", sample.velocity_enu_m_s, "m/s")
    vector("heave/surge/sway", sample.heave_surge_sway_m, "m")
    vector("heave/surge/sway frequency", sample.heave_surge_sway_hz, "Hz")
    scalar("temperature", sample.temperature_c, "°C")
    scalar("pressure", sample.pressure_pa, "Pa")
    values = sample.to_dict()
    scalar("course over ground", values.get("course_rad"), "°", 180 / math.pi)
    if len(parts) == 1:
        parts.append(_json(values))
    if not sample.complete or sample.issues:
        parts.append("partial/invalid: " + ", ".join(sample.issues or ("incomplete decode",)))
    return "  ".join(parts)


def _sample_consumer(
    recording: Recorder | None,
    stopped: threading.Event,
    *,
    jsonl: bool,
    quiet: bool,
    display_rate: float,
) -> Callable[[Sample], None]:
    """Record every sample; independently limit how often it is displayed."""
    last_display: dict[tuple[str, int | None], float] = {}

    def consume(sample: Sample) -> None:
        if recording is not None:
            recording.write(sample)
        now = time.monotonic()
        if quiet or (stopped.is_set() and not jsonl):
            return
        key = (sample.type, sample.metadata.get("device_id", sample.values.get("node_id")))
        if jsonl or now - last_display.get(key, float("-inf")) >= 1 / display_rate:
            try:
                click.echo(_json(sample) if jsonl else _human_sample(sample))
                last_display[key] = now
            except KeyboardInterrupt:
                stopped.set()

    return consume


@click.group(invoke_without_command=True)
@click.version_option(package_name="hipnuc-sdk")
@click.pass_context
def main(ctx):
    """HiPNUC IMU/INS tools. Start with list, info, or read.

    Connection options follow the final command: read -p COM3 -b 115200.
    """
    # Windows consoles support Unicode, but redirected Python streams can use
    # a legacy code page. CLI pipes/files consistently emit UTF-8, like Recorder.
    for stream in (sys.stdout, sys.stderr):
        if not stream.isatty() and hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@main.command("help")
@click.argument("commands", nargs=-1)
@click.pass_context
def help_command(ctx, commands):
    """Show help for a command, for example: help command."""
    context = ctx.parent
    for name in commands:
        group = context.command
        command = group.get_command(context, name) if isinstance(group, click.Group) else None
        if command is None:
            raise click.UsageError(f"Unknown command {name!r}")
        context = click.Context(command, info_name=name, parent=context)
    click.echo(context.get_help())


@main.command("list")
@click.option(
    "--all", "show_all", is_flag=True, help="Also show native and other non-USB serial ports."
)
@click.option("--json", "as_json", is_flag=True)
@_errors
def list_command(show_all, as_json):
    """List USB serial ports; --all or --json includes all system ports. Opens none."""
    ports = list(list_ports.comports())
    if as_json:
        click.echo(
            _json(
                [
                    {
                        "port": p.device,
                        "description": p.description,
                        "manufacturer": p.manufacturer,
                        "serial_number": p.serial_number,
                        "vid": p.vid,
                        "pid": p.pid,
                    }
                    for p in ports
                ]
            )
        )
        return
    usb = [p for p in ports if is_usb_port(p)]
    other = [p for p in ports if not is_usb_port(p)]
    for port in usb + other if show_all else usb:
        click.echo(f"{port.device}  {port.description or ''}")
    if not ports or (not usb and not show_all):
        message = "No serial ports found." if not ports else "No USB serial ports found."
        click.echo(
            message + " Check the USB connection and driver. In a virtual machine, "
            "connect the USB device to the guest system.",
            err=True,
        )
    if other and not show_all:
        click.echo(
            f"{len(other)} other serial ports hidden; use list --all to show them "
            "and -p PORT to use a native UART or mapped serial port.",
            err=True,
        )


@main.command("scan")
@_serial_options
@click.option("--json", "as_json", is_flag=True)
@_errors
def scan_command(port, baudrate, timeout, scan_timeout, as_json):
    """Find HiPNUC devices on USB serial ports, or on the port selected with -p."""
    scope = port if port is not None else "USB serial ports"
    click.echo(f"Searching {scope} for HiPNUC devices (Ctrl-C to cancel)...", err=True)
    with _discovery_progress():
        result = discover(
            [port] if port is not None else None,
            baudrates=(baudrate,) if baudrate is not None else BAUDRATES,
            timeout=timeout,
            scan_timeout=scan_timeout,
        )
    if as_json:
        click.echo(_json(result))
    else:
        for device in result.devices:
            name = device.info.product_name if device.info else "HiPNUC binary data"
            version = (
                f" / {device.info.firmware_version}"
                if device.info and device.info.firmware_version
                else ""
            )
            click.echo(f"{device.port}  {device.baudrate} baud  {name}{version}")
            if device.identity_error:
                click.echo(
                    f"{device.port}: identity unavailable: {device.identity_error}", err=True
                )
    if not result.complete:
        raise click.ClickException(
            "Search incomplete; increase --scan-timeout or specify -p and -b."
        )
    if not result.devices:
        raise click.ClickException(discovery_error_summary(result.errors))
    if result.errors:
        click.echo(
            f"{len(result.errors)} port(s) could not be matched; see the search results above.",
            err=True,
        )


@main.command("info")
@_serial_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_serial
def info_command(device, as_json):
    """Query product, firmware and serial number."""
    try:
        info = device.info or device.read_info()
    except ResponseTimeout as exc:
        raise ResponseTimeout(f"Identity query timed out. {exc}") from exc
    _show(info, as_json)


@main.command("read")
@_serial_options
@_record_options
@click.option(
    "--record-raw",
    type=click.Path(path_type=Path, dir_okay=False),
    help="Write exact received serial bytes.",
)
@_errors
def read_command(
    port,
    baudrate,
    timeout,
    scan_timeout,
    record,
    record_raw,
    duration,
    display_rate,
    quiet,
    jsonl,
    overwrite,
):
    """Read continuously; Ctrl-C stops. Recordings include every decoded sample."""
    with ExitStack() as stack:
        # A failed connection must not leave empty recording files behind.
        device = stack.enter_context(_serial_connection(port, baudrate, timeout, scan_timeout))
        recording = (
            stack.enter_context(Recorder(record, raw_path=record_raw, overwrite=overwrite))
            if record or record_raw
            else None
        )
        device.raw_sink = recording.write_raw if recording else None
        # Recording callbacks are ready before the first acquisition read.
        stopped = stack.enter_context(_stop_on_interrupt())
        device.sample_sink = _sample_consumer(
            recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=display_rate
        )
        deadline = time.monotonic() + duration if duration is not None else float("inf")
        try:
            while not stopped.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    device.read(min(timeout, remaining))
                except ResponseTimeout:
                    if stopped.is_set() or time.monotonic() >= deadline:
                        break
                    raise
        except KeyboardInterrupt:
            stopped.set()
        finally:
            stats = device.decoder.statistics
            click.echo(
                f"Stopped: {stats['samples']} samples, {stats['bytes_received']} bytes, "
                f"CRC errors {stats['crc_errors']}, length errors {stats['length_errors']}."
                + (
                    f" Recorded: {recording.samples_written} samples, {recording.raw_bytes_written} raw bytes."
                    if recording
                    else ""
                ),
                err=True,
            )
    # Flush and close normally before turning a requested stop into an exit status.
    if stopped.is_set():
        raise click.exceptions.Exit(130)
    if not device.decoder.statistics["samples"]:
        reason = (
            "No bytes received."
            if not device.decoder.statistics["bytes_received"]
            else "Received bytes but no valid measurement frames."
        )
        raise click.ClickException(
            f"No samples collected. {reason} Check the connection, host baudrate "
            "and supported output messages. For slow output, allow a longer --duration."
        )


@main.command("command")
@_serial_options(require_port=True)
@click.argument("command_text", required=False)
@click.option(
    "--file", "command_file", type=click.Path(exists=True, dir_okay=False, path_type=Path)
)
@click.option("--no-reply", is_flag=True, help="Send only; do not wait for OK.")
@click.option("--save", is_flag=True, help="Send SAVECONFIG once, after all commands succeed.")
@click.option(
    "--reboot", is_flag=True, help="Reboot after commands and optional save; wait for reconnection."
)
@click.option("--json", "as_json", is_flag=True)
@_errors
def command_command(
    port,
    baudrate,
    timeout,
    scan_timeout,
    command_text,
    command_file,
    no_reply,
    save,
    reboot,
    as_json,
):
    """Send one quoted ASCII command, or a UTF-8 command file."""
    response = "none" if no_reply else "auto"
    if (command_text is None) == (command_file is None):
        raise click.UsageError(
            'Supply one quoted command (e.g. "LOG VERSION") or --file, exclusively.'
        )
    lines = (
        command_file.read_text(encoding="utf-8-sig").splitlines()
        if command_file
        else [command_text]
    )
    commands = [
        line.strip() for line in lines if line.strip() and not line.lstrip().startswith(("#", ";"))
    ]
    if not commands:
        raise click.UsageError("No commands supplied.")
    if save or reboot:
        if no_reply:
            raise click.UsageError("--save/--reboot require command replies; remove --no-reply.")
        if any(
            line.split()[0].upper() in {"SERIALCONFIG", "REBOOT", "FRESET"} for line in commands
        ):
            raise click.UsageError(
                "Do not combine raw SERIALCONFIG/REBOOT/FRESET with --save/--reboot; "
                "use the baudrate or reboot command for managed reconnection."
            )
    with _serial_connection(port, baudrate, timeout, scan_timeout) as device:
        results = []
        for index, line in enumerate(commands, 1):
            if command_file:
                click.echo(f"Command {index}/{len(commands)}: {line}", err=True)
            try:
                result = device.command(line, response=response)
            except (HipnucError, OSError) as exc:
                raise click.ClickException(
                    f"Command {index}/{len(commands)} failed ({line}): {exc}"
                ) from exc
            results.append(result)
            if not as_json:
                _show(result, False)
        if save:
            click.echo("Saving configuration...", err=True)
            result = device.save_config()
            results.append(result)
            if not as_json:
                _show(result, False)
        if reboot:
            click.echo("Rebooting and waiting for reconnection...", err=True)
            result = device.reboot()
            results.append(result)
            if not as_json:
                _show(result, False)
        if as_json:
            click.echo(
                _json(
                    [result.to_dict() for result in results]
                    if command_file or save or reboot
                    else results[0]
                )
            )


@main.command("baudrate")
@click.argument("new_baud", type=click.IntRange(min=1))
@click.option("--device-port", type=click.Choice(["COM1", "COM2", "COM3", "COM4"]))
@click.option("--save", is_flag=True, help="Save after verifying the new connection.")
@_serial_options(require_port=True)
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_serial
def baudrate_command(device, new_baud, device_port, save, as_json):
    """Change device baudrate and verify communication at the new speed."""
    _show(device.set_baudrate(new_baud, device_port=device_port, save=save), as_json)


@main.command("reboot")
@click.option("--save", is_flag=True, help="Save current settings before rebooting.")
@_serial_options(require_port=True)
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_serial
def reboot_command(device, save, as_json):
    """Reboot and verify the returning device identity."""
    if save:
        device.save_config()
    _show(device.reboot(), as_json)


@main.group("modbus", invoke_without_command=True)
@click.pass_context
def modbus_group(ctx):
    """Addressed Modbus RTU operations; specify -p after the final command."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@modbus_group.command("info")
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_modbus
def modbus_info(device, as_json):
    """Read the device identity block."""
    _show(device.read_info(), as_json)


@modbus_group.command("read")
@_modbus_options
@_record_options
@click.option("--count", type=click.IntRange(min=1), help="Stop after this many samples.")
@click.option(
    "--interval",
    type=click.FloatRange(min=0),
    callback=_finite,
    default=0.1,
    show_default=True,
    help="Seconds between completed polls.",
)
@click.option("--mru", is_flag=True, help="Include the MRU measurement block.")
@_errors
def modbus_read(
    port,
    baudrate,
    timeout,
    device_id,
    record,
    duration,
    display_rate,
    quiet,
    jsonl,
    overwrite,
    count,
    interval,
    mru,
):
    """Poll continuously and optionally record SI JSONL with the station ID."""
    with ExitStack() as stack:
        device = stack.enter_context(_modbus_connection(port, baudrate, timeout, device_id))
        recording = stack.enter_context(Recorder(record, overwrite=overwrite)) if record else None
        stopped = stack.enter_context(_stop_on_interrupt())
        consume = _sample_consumer(
            recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=display_rate
        )
        deadline = time.monotonic() + duration if duration is not None else float("inf")
        samples = 0
        try:
            while not stopped.is_set() and time.monotonic() < deadline:
                consume(device.read_sample(include_mru=mru))
                samples += 1
                if count is not None and samples >= count:
                    break
                stopped.wait(min(interval, max(0, deadline - time.monotonic())))
        except KeyboardInterrupt:
            stopped.set()
        except HipnucError:
            if not stopped.is_set():
                raise
        finally:
            click.echo(
                f"Stopped: {samples} samples, Modbus ID {device_id}."
                + (f" Recorded: {recording.samples_written} samples." if recording else ""),
                err=True,
            )
    if stopped.is_set():
        raise click.exceptions.Exit(130)
    if not samples:
        raise click.ClickException("No samples collected from the Modbus device.")


@modbus_group.command("registers")
@click.argument("address", type=_integer)
@click.argument("count", type=click.IntRange(1, 125))
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_modbus
def modbus_registers(device, address, count, as_json):
    """Read raw FC03 words; accepts decimal or 0x-prefixed addresses."""
    _show({"address": address, "registers": device.read_registers(address, count)}, as_json)


@modbus_group.command("write-register")
@click.argument("address", type=_integer)
@click.argument("value", type=_integer)
@click.option(
    "--verify/--no-verify", default=True, help="Read back; disable for write-only controls."
)
@click.option("--save", is_flag=True, help="Save after the write succeeds.")
@click.option(
    "--reboot",
    is_flag=True,
    help="Reboot after the write and optional save; wait for reconnection.",
)
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
def modbus_write_register(
    port,
    baudrate,
    timeout,
    device_id,
    address,
    value,
    verify,
    save,
    reboot,
    as_json,
):
    """Issue raw FC06; an echo alone does not prove the setting took effect."""
    if (save or reboot) and address in {0x00, 0x04, 0x05}:
        raise click.UsageError(
            "Do not append --save/--reboot to raw control, baudrate or ID writes; "
            "use modbus baudrate, set-id or reboot for managed reconnection."
        )
    with _modbus_connection(port, baudrate, timeout, device_id) as device:
        result = device.write_register(address, value, verify=verify)
        if not as_json:
            _show(result, False)
        if save:
            click.echo("Saving configuration...", err=True)
            device.save_config()
        if reboot:
            click.echo("Rebooting and waiting for reconnection...", err=True)
            device.reboot()
        if as_json:
            _show(result, True)


@modbus_group.command("set-id")
@click.argument("new_id", type=click.IntRange(1, 247))
@click.option("--save", is_flag=True, help="Save after verifying the new station ID.")
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_modbus
def modbus_set_id(device, new_id, save, as_json):
    """Change one station ID and verify the same device at its new address."""
    _show(device.set_id(new_id, save=save), as_json)


@modbus_group.command("baudrate")
@click.argument("new_baud", type=click.IntRange(min=1))
@click.option(
    "--reboot", is_flag=True, help="Apply the stored baudrate by rebooting and reconnecting."
)
@click.option(
    "--save", is_flag=True, help="Save before a requested reboot (required on old firmware)."
)
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_modbus
def modbus_baudrate(device, new_baud, reboot, save, as_json):
    """Store device baudrate; use explicit --reboot to apply it."""
    _show(device.set_baudrate(new_baud, reboot=reboot, save=save), as_json)


@modbus_group.command("reboot")
@click.option("--save", is_flag=True, help="Save current settings before rebooting.")
@_modbus_options
@click.option("--json", "as_json", is_flag=True)
@_errors
@_with_modbus
def modbus_reboot(device, save, as_json):
    """Reboot and verify the addressed device."""
    _show(device.reboot(save=save), as_json)


@contextmanager
def _can_connection(interface: str, *, fd: bool = False) -> Iterator:
    """Own a SocketCAN bus; never load python-can configuration files."""
    try:
        import can
    except ModuleNotFoundError as exc:
        if exc.name != "can":
            raise
        raise TransportError('CAN support requires: python -m pip install ".[can]"') from exc
    if not sys.platform.startswith("linux"):
        raise TransportError("The CAN CLI uses Linux SocketCAN. Use CHCenter for desktop CAN.")
    hint = (
        "Check the SocketCAN interface, link state and bitrate "
        f"with 'ip -details link show {interface}'."
    )
    try:
        bus = can.Bus(interface="socketcan", channel=interface, fd=fd, ignore_config=True)
    except (can.CanError, OSError) as exc:
        raise TransportError(f"Cannot open {interface}: {exc}. {hint}") from exc
    try:
        with bus:
            click.echo(f"Connected to SocketCAN {interface}.", err=True)
            yield bus
    except can.CanError as exc:
        raise TransportError(f"{interface}: {exc}. {hint}") from exc


def _can_options(func=None, *, require_node=False, update=False):
    if func is None:
        return partial(_can_options, require_node=require_node, update=update)
    func = click.option(
        "--id",
        "node_id",
        type=click.IntRange(1, 127) if update else click.IntRange(0, 255),
        required=require_node,
        help="Target node ID; read all sources when omitted.",
    )(func)
    return click.option("-i", "--interface", required=True, help="Linux CAN interface, e.g. can0.")(
        func
    )


@main.group("can", invoke_without_command=True)
@click.pass_context
def can_group(ctx):
    """Linux SocketCAN: read, record, access registers and update firmware."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@can_group.command("read")
@_can_options
@_timeout_option
@_record_options
@click.option("--count", type=click.IntRange(min=1), help="Stop after this many decoded samples.")
@click.option(
    "--fd", is_flag=True, help="Also receive CAN FD frames; configure the interface first."
)
@_errors
def can_read(
    interface, node_id, timeout, record, duration, display_rate, quiet, jsonl, overwrite, count, fd
):
    """Read current-frame measurements continuously; Ctrl-C stops."""
    from .can import decode_message

    samples = frames = invalid = 0
    with ExitStack() as stack:
        bus = stack.enter_context(_can_connection(interface, fd=fd))
        recording = stack.enter_context(Recorder(record, overwrite=overwrite)) if record else None
        stopped = stack.enter_context(_stop_on_interrupt())
        consume = _sample_consumer(
            recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=display_rate
        )
        now = time.monotonic()
        deadline = now + duration if duration is not None else float("inf")
        idle_deadline = now + timeout
        try:
            while not stopped.is_set():
                now = time.monotonic()
                if now >= deadline:
                    break
                if now >= idle_deadline:
                    reason = "No CAN frames received" if not frames else "No valid HiPNUC samples"
                    raise ResponseTimeout(
                        f"{reason} on {interface} within {timeout:g}s. Check the target ID, "
                        "device output, bitrate and wiring; use --timeout for slow output."
                    )
                message = bus.recv(min(0.1, deadline - now, idle_deadline - now))
                if message is None:
                    continue
                frames += 1
                if node_id is not None and message.arbitration_id & 0xFF != node_id:
                    continue
                try:
                    sample = decode_message(message)
                except ValueError:
                    invalid += 1
                    continue
                if sample is None:
                    continue
                # A CAN receive call yields one complete frame. Finish writing it
                # even when SIGINT arrives during reception or decoding.
                consume(sample)
                samples += 1
                idle_deadline = time.monotonic() + timeout
                if count is not None and samples >= count:
                    break
        except KeyboardInterrupt:
            stopped.set()
        finally:
            click.echo(
                f"Stopped: {samples} samples, {frames} CAN frames, {invalid} invalid frames."
                + (f" Recorded: {recording.samples_written} samples." if recording else ""),
                err=True,
            )
    if stopped.is_set():
        raise click.exceptions.Exit(130)
    if not samples:
        raise click.ClickException("No HiPNUC CAN samples collected.")


@can_group.group("reg", invoke_without_command=True)
@click.pass_context
def can_reg_group(ctx):
    """Raw J1939 registers; use the model's command and programming manual."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


def _check_can_register(node_id: int, address: int, value: int = 0) -> None:
    if node_id > 253 or node_id == 0x55:
        raise ValueError("Register target ID must be 0–253, excluding host address 0x55.")
    if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("Register address must be uint16 and value must be uint32.")


@can_reg_group.command("read")
@click.argument("address", type=_integer)
@_can_options(require_node=True)
@_timeout_option
@click.option("--json", "as_json", is_flag=True)
@_errors
def can_reg_read(address, interface, node_id, timeout, as_json):
    """Read one raw 32-bit register value."""
    from .can import read_register

    _check_can_register(node_id, address)
    with _can_connection(interface) as bus:
        value = read_register(bus, node_id, address, timeout=timeout)
    _show({"node_id": node_id, "address": address, "value": value}, as_json)


@can_reg_group.command("write")
@click.argument("address", type=_integer)
@click.argument("value", type=_integer)
@_can_options(require_node=True)
@_timeout_option
@click.option("--json", "as_json", is_flag=True)
@_errors
def can_reg_write(address, value, interface, node_id, timeout, as_json):
    """Write one raw register and check its reply; does not save or reboot."""
    from .can import write_register

    _check_can_register(node_id, address, value)
    with _can_connection(interface) as bus:
        write_register(bus, node_id, address, value, timeout=timeout)
    _show({"node_id": node_id, "address": address, "value": value, "acknowledged": True}, as_json)


def _update_progress() -> Callable[[int, int], None]:
    last_percent = -1

    def report(written: int, total: int) -> None:
        nonlocal last_percent
        percent = written * 100 // total if total else 0
        if (percent in (0, 100) and percent != last_percent) or percent >= last_percent + 10:
            click.echo(f"Writing: {percent}%", err=True)
            last_percent = percent

    return report


def _show_update(result, as_json: bool) -> None:
    if as_json:
        click.echo(_json(asdict(result)))
    else:
        click.echo(
            f"Transfer acknowledged: {result.bytes_written} bytes. Application start requested."
        )
        if not result.start_acknowledged:
            click.echo("No start acknowledgement; the bootloader may already have reset.", err=True)
        if not result.application_verified:
            click.echo("Running application was not verified.", err=True)


@main.command("update")
@click.argument("image", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option("-p", "--port", required=True, help="Target serial port; no automatic discovery.")
@click.option("-b", "--baudrate", type=click.IntRange(min=1), required=True)
@click.option("--json", "as_json", is_flag=True)
@_errors
def serial_update(image, port, baudrate, as_json):
    """Update one device using its model-specific Intel HEX application image."""
    from .update import update_serial

    click.echo(f"Updating {port} at {baudrate} baud from {image}.", err=True)
    _show_update(
        update_serial(image, port=port, baudrate=baudrate, progress=_update_progress()), as_json
    )


@can_group.command("update")
@click.argument("image", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@_can_options(require_node=True, update=True)
@click.option("--bin", "raw_binary", is_flag=True, help="Use a raw binary instead of Intel HEX.")
@click.option("--json", "as_json", is_flag=True)
@_errors
def can_update(image, interface, node_id, raw_binary, as_json):
    """Update one node using its model-specific application image (CAN SDO)."""
    from .update import update_can

    with _can_connection(interface) as bus:
        click.echo(f"Updating node {node_id} from {image}.", err=True)
        result = update_can(bus, node_id, image, raw_binary=raw_binary, progress=_update_progress())
    _show_update(result, as_json)
