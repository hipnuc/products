"""Modbus RTU subcommands; the optional ``modbus`` extra is required to run them."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from functools import wraps
import time

import click

from ..errors import HipnucError
from ..modbus import ModbusBus, ModbusDevice
from ..recording import Recorder
from ._common import (
    SampleConsumer,
    errors,
    finish,
    finite,
    integer,
    json_option,
    record_options,
    show,
    status,
    stop_on_interrupt,
    stop_summary,
    timeout_option,
)


def modbus_options(func):
    func = timeout_option(func)
    func = click.option(
        "--id",
        "device_id",
        type=click.IntRange(1, 247),
        default=80,
        show_default=True,
        envvar="HIHOST_MODBUS_ID",
        show_envvar=True,
        help="Modbus station ID.",
    )(func)
    func = click.option(
        "-b",
        "--baudrate",
        type=click.IntRange(min=1),
        default=115200,
        show_default=True,
        envvar="HIHOST_BAUDRATE",
        show_envvar=True,
        help="RTU baudrate.",
    )(func)
    return click.option(
        "-p",
        "--port",
        required=True,
        envvar="HIHOST_PORT",
        show_envvar=True,
        help="Serial port of the RTU bus.",
    )(func)


@contextmanager
def modbus_connection(
    port: str, baudrate: int, timeout: float, device_id: int
) -> Iterator[ModbusDevice]:
    """Own the RTU bus for this command and select its addressed device."""
    with ModbusBus(port, baudrate, timeout) as bus:
        device = bus.device(device_id)
        click.echo(f"Connected to {port} at {baudrate} baud, Modbus ID {device_id}.", err=True)
        yield device


def with_modbus(func):
    @wraps(func)
    def wrapped(*args, **kwargs):
        connection = {
            name: kwargs.pop(name) for name in ("port", "baudrate", "timeout", "device_id")
        }
        with modbus_connection(**connection) as device:
            return func(device, *args, **kwargs)

    return wrapped


@click.group("modbus", invoke_without_command=True)
@click.pass_context
def modbus_group(ctx):
    """Addressed Modbus RTU operations; specify -p after the final command."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@modbus_group.command("info")
@modbus_options
@json_option
@errors
@with_modbus
def modbus_info(device, as_json):
    """Read the device identity block."""
    show(device.read_info(), as_json)


@modbus_group.command("read")
@modbus_options
@record_options
@click.option("--count", type=click.IntRange(min=1), help="Stop after this many samples.")
@click.option(
    "--interval",
    type=click.FloatRange(min=0),
    callback=finite,
    default=0.1,
    show_default=True,
    help="Seconds between completed polls.",
)
@click.option("--mru", is_flag=True, help="Include the MRU measurement block.")
@errors
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
        device = stack.enter_context(modbus_connection(port, baudrate, timeout, device_id))
        recording = stack.enter_context(Recorder(record, overwrite=overwrite)) if record else None
        stopped = stack.enter_context(stop_on_interrupt())
        consume = SampleConsumer(
            recording, stopped, jsonl=jsonl, quiet=quiet, display_rate=display_rate
        )
        deadline = time.monotonic() + duration if duration is not None else float("inf")
        try:
            while not stopped.is_set() and time.monotonic() < deadline:
                consume(device.read_sample(include_mru=mru))
                if count is not None and consume.samples >= count:
                    break
                stopped.wait(min(interval, max(0, deadline - time.monotonic())))
        except KeyboardInterrupt:
            stopped.set()
        except HipnucError:
            if not stopped.is_set():
                raise
        finally:
            status(f"Stopped: {stop_summary(consume, recording, f'Modbus ID {device_id}')}")
    finish(stopped)
    if not consume.samples:
        raise click.ClickException("No samples collected from the Modbus device.")


@modbus_group.command("registers")
@click.argument("address", type=integer)
@click.argument("count", type=click.IntRange(1, 125))
@modbus_options
@json_option
@errors
@with_modbus
def modbus_registers(device, address, count, as_json):
    """Read raw FC03 words; accepts decimal or 0x-prefixed addresses."""
    show({"address": address, "registers": device.read_registers(address, count)}, as_json)


@modbus_group.command("write-register")
@click.argument("address", type=integer)
@click.argument("value", type=integer)
@click.option(
    "--verify/--no-verify", default=True, help="Read back; disable for write-only controls."
)
@click.option("--save", is_flag=True, help="Save after the write succeeds.")
@click.option(
    "--reboot",
    is_flag=True,
    help="Reboot after the write and optional save; wait for reconnection.",
)
@modbus_options
@json_option
@errors
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
    with modbus_connection(port, baudrate, timeout, device_id) as device:
        result = device.write_register(address, value, verify=verify)
        if not as_json:
            show(result, False)
        if save:
            click.echo("Saving configuration...", err=True)
            device.save_config()
        if reboot:
            click.echo("Rebooting and waiting for reconnection...", err=True)
            device.reboot()
        if as_json:
            show(result, True)


@modbus_group.command("set-id")
@click.argument("new_id", type=click.IntRange(1, 247))
@click.option("--save", is_flag=True, help="Save after verifying the new station ID.")
@modbus_options
@json_option
@errors
@with_modbus
def modbus_set_id(device, new_id, save, as_json):
    """Change one station ID and verify the same device at its new address."""
    show(device.set_id(new_id, save=save), as_json)


@modbus_group.command("baudrate")
@click.argument("new_baud", type=click.IntRange(min=1))
@click.option(
    "--reboot", is_flag=True, help="Apply the stored baudrate by rebooting and reconnecting."
)
@click.option(
    "--save", is_flag=True, help="Save before a requested reboot (required on old firmware)."
)
@modbus_options
@json_option
@errors
@with_modbus
def modbus_baudrate(device, new_baud, reboot, save, as_json):
    """Store device baudrate; use explicit --reboot to apply it."""
    show(device.set_baudrate(new_baud, reboot=reboot, save=save), as_json)


@modbus_group.command("reboot")
@click.option("--save", is_flag=True, help="Save current settings before rebooting.")
@modbus_options
@json_option
@errors
@with_modbus
def modbus_reboot(device, save, as_json):
    """Reboot and verify the addressed device."""
    show(device.reboot(save=save), as_json)
