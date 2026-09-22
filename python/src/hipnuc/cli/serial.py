"""Serial subcommands: list, scan, info, read, decode, command, baudrate, reboot."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from functools import partial, wraps
import logging
from pathlib import Path
import sys
import time

import click
from serial.tools import list_ports

from .._connection import discovery_error_summary, is_usb_port
from ..decoder import Decoder
from ..errors import HipnucError, ResponseTimeout
from ..recording import Recorder
from ..serial_device import BAUDRATES, SerialDevice, discover
from ._common import (
    SampleConsumer,
    errors,
    finish,
    finite,
    json_option,
    record_options,
    show,
    status,
    stop_on_interrupt,
    stop_summary,
    timeout_option,
    to_json,
    verbose_enabled,
)


def serial_options(func=None, *, require_port=False):
    if func is None:
        return partial(serial_options, require_port=require_port)
    func = timeout_option(func)
    func = click.option(
        "--scan-timeout",
        type=click.FloatRange(min=0, min_open=True),
        callback=finite,
        default=30.0,
        show_default=True,
        help="Total automatic discovery time limit in seconds.",
    )(func)
    func = click.option(
        "-b",
        "--baudrate",
        type=click.IntRange(min=1),
        envvar="HIHOST_BAUDRATE",
        show_envvar=True,
        help="Host baudrate; auto-detect if omitted. Supply -p and -b to skip discovery.",
    )(func)
    return click.option(
        "-p",
        "--port",
        required=require_port,
        envvar="HIHOST_PORT",
        show_envvar=True,
        help=(
            "Target serial port, e.g. COM3 or /dev/ttyUSB0. Use 'hihost scan' to find it."
            if require_port
            else "COM3, /dev/ttyUSB0, etc.; auto-select a USB serial port if omitted."
        ),
    )(func)


@contextmanager
def discovery_progress() -> Iterator[None]:
    """Show SDK discovery progress unless -v already routes every SDK log to stderr."""
    if verbose_enabled():
        yield
        return
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
def serial_connection(
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
        discovery_progress(),
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


def with_serial(func):
    @wraps(func)
    def wrapped(*args, **kwargs):
        connection = {
            name: kwargs.pop(name) for name in ("port", "baudrate", "timeout", "scan_timeout")
        }
        with serial_connection(**connection) as device:
            return func(device, *args, **kwargs)

    return wrapped


def decoder_summary(decoder: Decoder, consumer: SampleConsumer, recording: Recorder | None) -> str:
    stats = decoder.statistics
    detail = (
        f"{stats['bytes_received']} bytes, CRC errors {stats['crc_errors']}, "
        f"length errors {stats['length_errors']}"
    )
    if stats["malformed_packets"]:
        detail += f", undecoded valid frames {stats['malformed_packets']}"
    return stop_summary(consumer, recording, detail)


def no_samples_error(decoder: Decoder) -> click.ClickException:
    stats = decoder.statistics
    if not stats["bytes_received"]:
        reason = "No bytes received."
    elif stats["malformed_packets"]:
        reason = f"Received {stats['malformed_packets']} valid frames this SDK does not decode."
    else:
        reason = "Received bytes but no valid measurement frames."
    return click.ClickException(f"No samples collected. {reason}")


@click.command("list")
@click.option(
    "--all", "show_all", is_flag=True, help="Also show native and other non-USB serial ports."
)
@json_option
@errors
def list_command(show_all, as_json):
    """List USB serial ports; --all or --json includes all system ports. Opens none."""
    ports = list(list_ports.comports())
    if as_json:
        click.echo(
            to_json(
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


@click.command("scan")
@serial_options
@json_option
@errors
def scan_command(port, baudrate, timeout, scan_timeout, as_json):
    """Find HiPNUC devices on USB serial ports, or on the port selected with -p."""
    scope = port if port is not None else "USB serial ports"
    click.echo(f"Searching {scope} for HiPNUC devices (Ctrl-C to cancel)...", err=True)
    with discovery_progress():
        result = discover(
            [port] if port is not None else None,
            baudrates=(baudrate,) if baudrate is not None else BAUDRATES,
            timeout=timeout,
            scan_timeout=scan_timeout,
        )
    if as_json:
        click.echo(to_json(result))
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


@click.command("info")
@serial_options
@json_option
@errors
@with_serial
def info_command(device, as_json):
    """Query product, firmware and serial number."""
    try:
        info = device.info or device.read_info()
    except ResponseTimeout as exc:
        raise ResponseTimeout(f"Identity query timed out. {exc}") from exc
    show(info, as_json)


@click.command("read")
@serial_options
@record_options
@click.option(
    "--record-raw",
    type=click.Path(path_type=Path, dir_okay=False),
    help="Write exact received serial bytes.",
)
@errors
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
        device = stack.enter_context(serial_connection(port, baudrate, timeout, scan_timeout))
        recording = (
            stack.enter_context(Recorder(record, raw_path=record_raw, overwrite=overwrite))
            if record or record_raw
            else None
        )
        device.raw_sink = recording.write_raw if recording else None
        # Recording callbacks are ready before the first acquisition read.
        stopped = stack.enter_context(stop_on_interrupt())
        consumer = SampleConsumer(
            recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=display_rate
        )
        device.sample_sink = consumer
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
            status(f"Stopped: {decoder_summary(device.decoder, consumer, recording)}")
    # Flush and close normally before turning a requested stop into an exit status.
    finish(stopped)
    if not device.decoder.statistics["samples"]:
        error = no_samples_error(device.decoder)
        error.message += (
            " Check the connection, host baudrate and supported output messages. "
            "For slow output, allow a longer --duration."
        )
        raise error


@click.command("decode")
@click.argument("source", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@record_options(finite_input=True)
@errors
def decode_command(source, record, quiet, jsonl, overwrite):
    """Decode a raw serial recording (read --record-raw) offline; prints every sample."""
    if record is not None and (
        source.resolve() == record.resolve() or (record.exists() and source.samefile(record))
    ):
        raise click.UsageError("Decode input and recording output must be different files.")
    decoder = Decoder()
    with ExitStack() as stack:
        recording = stack.enter_context(Recorder(record, overwrite=overwrite)) if record else None
        stopped = stack.enter_context(stop_on_interrupt())
        consumer = SampleConsumer(recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=None)
        with source.open("rb") as stream:
            while not stopped.is_set() and (chunk := stream.read(65536)):
                for sample in decoder.feed(chunk):
                    consumer(sample)
            for sample in decoder.finish():
                consumer(sample)
        status(f"Decoded {source}: {decoder_summary(decoder, consumer, recording)}")
    finish(stopped)
    if not decoder.statistics["samples"]:
        raise no_samples_error(decoder)


@click.command("command")
@serial_options(require_port=True)
@click.argument("command_text", required=False)
@click.option(
    "--file",
    "command_file",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    help="UTF-8 file with one command per line; # and ; start comments.",
)
@click.option("--no-reply", is_flag=True, help="Send one command and close without waiting for OK.")
@click.option("--save", is_flag=True, help="Send SAVECONFIG once, after all commands succeed.")
@click.option(
    "--reboot", is_flag=True, help="Reboot after commands and optional save; wait for reconnection."
)
@json_option
@errors
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
    if no_reply and len(commands) != 1:
        raise click.UsageError(
            "--no-reply accepts only one command; the connection closes after it."
        )
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
    with serial_connection(port, baudrate, timeout, scan_timeout) as device:
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
                show(result, False)
        if save:
            click.echo("Saving configuration...", err=True)
            result = device.save_config()
            results.append(result)
            if not as_json:
                show(result, False)
        if reboot:
            click.echo("Rebooting and waiting for reconnection...", err=True)
            result = device.reboot()
            results.append(result)
            if not as_json:
                show(result, False)
        if as_json:
            click.echo(
                to_json(
                    [result.to_dict() for result in results]
                    if command_file or save or reboot
                    else results[0]
                )
            )


@click.command("baudrate")
@click.argument("new_baud", type=click.IntRange(min=1))
@click.option("--save", is_flag=True, help="Save after verifying the new connection.")
@serial_options(require_port=True)
@json_option
@errors
@with_serial
def baudrate_command(device, new_baud, save, as_json):
    """Change the connected device port's baudrate and verify the new connection."""
    show(device.set_baudrate(new_baud, save=save), as_json)


@click.command("reboot")
@click.option("--save", is_flag=True, help="Save current settings before rebooting.")
@serial_options(require_port=True)
@json_option
@errors
@with_serial
def reboot_command(device, save, as_json):
    """Reboot and verify the returning device identity."""
    if save:
        device.save_config()
    show(device.reboot(), as_json)


COMMANDS = (
    list_command,
    scan_command,
    info_command,
    read_command,
    decode_command,
    command_command,
    baudrate_command,
    reboot_command,
)
