"""Linux SocketCAN subcommands; the optional ``can`` extra is required to run them."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import ExitStack, contextmanager
from functools import partial
from pathlib import Path
import sys
import time

import click
from click.core import ParameterSource

from ..can import (
    BITRATES,
    BROADCAST_ADDRESS,
    PERIOD_SYNC_IN,
    PGN_NAMES,
    decode_message,
    factory_reset,
    read_info,
    read_register,
    reboot,
    request_reboot,
    save_config,
    scan,
    send_trigger,
    set_bitrate,
    set_node_id,
    set_output_enabled,
    set_pgn_period,
    wait_for_node,
    write_register,
)
from ..errors import ResponseTimeout, TransportError
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
from .update import show_update, update_progress

_HOST_ADDRESS = 0x55


@contextmanager
def can_connection(interface: str, *, fd: bool = False) -> Iterator:
    """Own one python-can bus; never load python-can configuration files."""
    if not sys.platform.startswith("linux"):
        raise TransportError("SocketCAN requires Linux; use the Python CAN API for other adapters.")
    try:
        import can
    except ModuleNotFoundError as exc:
        if exc.name != "can":
            raise
        raise TransportError('CAN support requires: python -m pip install ".[can]"') from exc
    hint = (
        "Check the SocketCAN interface, link state and bitrate "
        f"with 'ip -details link show {interface}'."
    )
    try:
        bus = can.Bus(interface="socketcan", channel=interface, ignore_config=True, fd=fd)
    except (can.CanError, OSError, ValueError) as exc:
        raise TransportError(f"Cannot open {interface}: {exc}. {hint}") from exc
    try:
        with bus:
            click.echo(f"Connected to SocketCAN {interface}.", err=True)
            yield bus
    except can.CanError as exc:
        raise TransportError(f"{interface}: {exc}. {hint}") from exc


def _node_id(_ctx, param, value):
    if value == _HOST_ADDRESS:
        raise click.BadParameter("85 (0x55) is the host address", param=param)
    return value


def interface_option(func):
    return click.option(
        "-i",
        "--interface",
        required=True,
        envvar="HIHOST_CAN_INTERFACE",
        show_envvar=True,
        help="SocketCAN interface, e.g. can0.",
    )(func)


def can_options(func=None, *, require_node=False, node_help=None):
    if func is None:
        return partial(can_options, require_node=require_node, node_help=node_help)
    func = click.option(
        "--id",
        "node_id",
        type=click.IntRange(1, 127),
        callback=_node_id,
        required=require_node,
        envvar="HIHOST_CAN_ID",
        show_envvar=True,
        help=node_help
        or ("Target node ID (1-127)." if require_node else "Node ID; all sources when omitted."),
    )(func)
    return interface_option(func)


def pgn_argument(value: str) -> int:
    """Accept a PGN number (0xFF34, 65332) or its name (acc, GYR, canfd83)."""
    if value.upper() in PGN_NAMES:
        return PGN_NAMES[value.upper()]
    try:
        pgn = integer(value)
    except click.BadParameter:
        pgn = None
    if pgn not in PGN_NAMES.values():
        names = ", ".join(name.lower() for name in PGN_NAMES)
        raise click.BadParameter(f"expected a PGN number or one of: {names}")
    return pgn


def period_argument(value: str) -> int:
    """Accept a period in ms, ``off`` (0) or ``sync`` (SYNC_IN triggered)."""
    word = value.strip().lower()
    if word == "off":
        return 0
    if word == "sync":
        return PERIOD_SYNC_IN
    period = integer(value)
    if not 0 <= period <= 0x7FFF:
        raise click.BadParameter("expected 0..32767 ms, off, or sync")
    return period


def _bitrate(table: dict[int, int]):
    def convert(value: str) -> int:
        rate = integer(value)
        if rate not in table:
            raise click.BadParameter("expected one of " + ", ".join(str(r) for r in sorted(table)))
        return rate

    return convert


def report(result, as_json: bool, sentence: str) -> None:
    """Print the SDK result as JSON, or one plain sentence for people."""
    if as_json:
        show(result, True)
    else:
        click.echo(sentence)


def recovery_option(func):
    return click.option(
        "--recovery-timeout",
        type=click.FloatRange(min=0, min_open=True),
        callback=finite,
        default=5.0,
        show_default=True,
        help="Seconds to wait for the node's identity after a reset.",
    )(func)


@click.group("can", invoke_without_command=True)
@click.pass_context
def can_group(ctx):
    """Linux SocketCAN: read, configure, access registers and update firmware."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@can_group.command("read")
@can_options
@timeout_option
@record_options
@click.option("--count", type=click.IntRange(min=1), help="Stop after this many decoded samples.")
@click.option(
    "--fd", is_flag=True, help="Also receive CAN FD frames; configure the interface first."
)
@errors
def can_read(
    interface,
    node_id,
    timeout,
    record,
    duration,
    display_rate,
    quiet,
    jsonl,
    overwrite,
    count,
    fd,
):
    """Read current-frame measurements continuously; Ctrl-C stops."""
    frames = invalid = 0
    with ExitStack() as stack:
        bus = stack.enter_context(can_connection(interface, fd=fd))
        recording = stack.enter_context(Recorder(record, overwrite=overwrite)) if record else None
        stopped = stack.enter_context(stop_on_interrupt())
        consume = SampleConsumer(
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
                received = time.time_ns()
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
                sample.received_time_ns = received
                # A CAN receive call yields one complete frame. Finish writing it
                # even when SIGINT arrives during reception or decoding.
                consume(sample)
                idle_deadline = time.monotonic() + timeout
                if count is not None and consume.samples >= count:
                    break
        except KeyboardInterrupt:
            stopped.set()
        finally:
            detail = f"{frames} CAN frames, {invalid} invalid frames"
            status(f"Stopped: {stop_summary(consume, recording, detail)}")
    finish(stopped)
    if not consume.samples:
        raise click.ClickException("No HiPNUC CAN samples collected.")


@can_group.command("scan")
@interface_option
@click.option(
    "--duration",
    type=click.FloatRange(min=0, min_open=True),
    callback=finite,
    default=2.0,
    show_default=True,
    help="Passive listening time in seconds.",
)
@click.option("--identify", is_flag=True, help="Also query each node's identity registers.")
@click.option("--fd", is_flag=True, help="Also receive CAN FD frames.")
@timeout_option(help="Identity query timeout per node, with --identify.")
@json_option
@errors
def can_scan(interface, duration, identify, fd, timeout, as_json):
    """List HiPNUC nodes seen on the bus with their PGNs and rates."""
    with can_connection(interface, fd=fd) as bus:
        click.echo(f"Listening on {interface} for {duration:g}s...", err=True)
        nodes = scan(bus, duration, identify=identify, timeout=timeout)
    if as_json:
        show([node.to_dict() for node in nodes], True)
    else:
        for node in nodes:
            summary = node.to_dict()
            pgns = ", ".join(
                f"{entry['name']} {entry['rate_hz']:.1f} Hz" for entry in summary["pgns"].values()
            )
            identity = ""
            if node.info is not None:
                identity = f"  {node.info.product_name or 'HiPNUC'}"
                if node.info.firmware_version:
                    identity += f" / {node.info.firmware_version}"
            click.echo(f"node {node.node_id}  {node.frames} frames  {pgns}{identity}")
            if node.identity_error:
                click.echo(
                    f"node {node.node_id}: identity unavailable: {node.identity_error}", err=True
                )
    if not nodes:
        raise click.ClickException(
            f"No HiPNUC PGNs seen on {interface} within {duration:g}s. Check the bitrate, "
            "wiring and that periodic output is enabled (can output on)."
        )


@can_group.command("info")
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_info(interface, node_id, timeout, as_json):
    """Query product, firmware and serial number through the identity registers."""
    with can_connection(interface) as bus:
        show(read_info(bus, node_id, timeout), as_json)


@can_group.command("save")
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_save(interface, node_id, timeout, as_json):
    """Send an explicit save; current firmware already persists every write."""
    with can_connection(interface) as bus:
        result = save_config(bus, node_id, timeout)
    report(result, as_json, f"Node {node_id}: save acknowledged.")


@can_group.command("reboot")
@click.option("--save", is_flag=True, help="Save current settings before rebooting.")
@click.option(
    "--factory-reset",
    "restore",
    is_flag=True,
    help="Restore factory settings; the node keeps its ID and restores default bitrates.",
)
@recovery_option
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_reboot(save, restore, recovery_timeout, interface, node_id, timeout, as_json):
    """Reset the node and verify its identity; the reset itself is never acknowledged."""
    with can_connection(interface) as bus:
        if save:
            save_config(bus, node_id, timeout)
        if restore:
            result = factory_reset(bus, node_id, timeout)
            report(
                result,
                as_json,
                f"Node {node_id}: factory reset sent; it keeps its node ID and "
                "restarts with default bitrates.",
            )
            return
        click.echo("Resetting and waiting for the node...", err=True)
        info = reboot(bus, node_id, recovery_timeout=recovery_timeout, timeout=timeout)
    show(info, as_json)


@can_group.command("set-id")
@click.argument("new_id", type=click.IntRange(1, 127), callback=_node_id)
@click.option("--save", is_flag=True, help="Save after storing the new ID.")
@click.option("--reboot", "restart", is_flag=True, help="Reboot and verify the node at its new ID.")
@recovery_option
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_set_id(new_id, save, restart, recovery_timeout, interface, node_id, timeout, as_json):
    """Store a new node ID (1-127); it takes effect after a reboot."""
    with can_connection(interface) as bus:
        result = set_node_id(bus, node_id, new_id, timeout)
        if save:
            save_config(bus, node_id, timeout)
        if not restart:
            report(result, as_json, f"Node {node_id}: ID {new_id} stored; reboot to apply.")
            return
        click.echo(f"Resetting and waiting for the node at ID {new_id}...", err=True)
        request_reboot(bus, node_id, timeout)
        info = wait_for_node(bus, new_id, recovery_timeout, timeout)
    if as_json:
        show({"stored": result.to_dict(), "info": info.to_dict()}, True)
    else:
        click.echo(f"Node {new_id} is back:")
        show(info, False)


@can_group.command("bitrate")
@click.argument("bitrate", type=_bitrate(BITRATES), metavar="BIT/S")
@click.option("--save", is_flag=True, help="Save after storing the bitrate.")
@click.option(
    "--reboot",
    "restart",
    is_flag=True,
    help="Reboot to apply; reconfigure the interface afterwards.",
)
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_bitrate(bitrate, save, restart, interface, node_id, timeout, as_json):
    """Store the nominal bitrate (10000-1000000 bit/s); applied after a reboot."""
    with can_connection(interface) as bus:
        result = set_bitrate(bus, node_id, bitrate, timeout=timeout)
        if save:
            save_config(bus, node_id, timeout)
        if restart:
            request_reboot(bus, node_id, timeout)
    report(
        result,
        as_json,
        f"Node {node_id}: bitrate {bitrate} stored; "
        + ("the node is restarting." if restart else "reboot to apply."),
    )
    click.echo(f"Reconfigure {interface} for the new bitrate before the next command.", err=True)


@can_group.command("period")
@click.argument("pgn", type=pgn_argument, metavar="PGN")
@click.argument("period", type=period_argument, metavar="MS|off|sync")
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_period(pgn, period, interface, node_id, timeout, as_json):
    """Set a PGN's period in ms, off, or sync (SYNC_IN triggered); applied immediately."""
    with can_connection(interface) as bus:
        result = set_pgn_period(bus, node_id, pgn, period, timeout)
    state = {0: "off", PERIOD_SYNC_IN: "SYNC_IN triggered"}.get(period, f"every {period} ms")
    report(result, as_json, f"Node {node_id}: PGN 0x{pgn:04X} {state}.")


@can_group.command("output")
@click.argument("state", type=click.Choice(["on", "off"]))
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_output(state, interface, node_id, timeout, as_json):
    """Switch all periodic PGN output on or off; applied immediately."""
    with can_connection(interface) as bus:
        result = set_output_enabled(bus, node_id, state == "on", timeout)
    report(result, as_json, f"Node {node_id}: periodic output {state}.")


@can_group.command("trigger")
@click.argument("pgn", type=pgn_argument, metavar="PGN")
@click.option("--all", "broadcast", is_flag=True, help="Trigger every node (broadcast).")
@can_options(node_help="Target node ID, or use --all for every node.")
@timeout_option
@json_option
@errors
@click.pass_context
def can_trigger(ctx, pgn, broadcast, interface, node_id, timeout, as_json):
    """Request one PGN now from a node, or from all nodes with --all; no acknowledgement."""
    # An --id from the environment is a default, so an explicit --all wins over it.
    explicit = ctx.get_parameter_source("node_id") == ParameterSource.COMMANDLINE
    if (broadcast and explicit) or (not broadcast and node_id is None):
        raise click.UsageError("Specify --id NODE or --all, exclusively.")
    target = BROADCAST_ADDRESS if broadcast else node_id
    with can_connection(interface) as bus:
        send_trigger(bus, target, pgn, timeout)
    if as_json:
        show({"node_id": target, "pgn": pgn, "acknowledged": False}, True)
    else:
        who = "every node" if broadcast else f"node {node_id}"
        click.echo(f"Trigger for PGN 0x{pgn:04X} sent to {who}; no acknowledgement expected.")


@can_group.group("reg", invoke_without_command=True)
@click.pass_context
def can_reg_group(ctx):
    """Raw J1939 registers; use the model's command and programming manual."""
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


def check_register(address: int, value: int = 0) -> None:
    """Reject out-of-range input before the bus is opened."""
    if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("Register address must be uint16 and value must be uint32.")


@can_reg_group.command("read")
@click.argument("address", type=integer)
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_reg_read(address, interface, node_id, timeout, as_json):
    """Read one raw 32-bit register value."""
    check_register(address)
    with can_connection(interface) as bus:
        value = read_register(bus, node_id, address, timeout=timeout)
    if as_json:
        show({"node_id": node_id, "address": address, "value": value}, True)
    else:
        click.echo(f"Node {node_id}: register 0x{address:04X} = 0x{value:08X} ({value})")


@can_reg_group.command("write")
@click.argument("address", type=integer)
@click.argument("value", type=integer)
@can_options(require_node=True)
@timeout_option
@json_option
@errors
def can_reg_write(address, value, interface, node_id, timeout, as_json):
    """Write one raw register and check its echo; does not save or reboot."""
    check_register(address, value)
    if address == 0 and value in (0x01, 0xFF):
        raise click.UsageError(
            "Resets are never acknowledged; use 'can reboot' (or --factory-reset) instead."
        )
    with can_connection(interface) as bus:
        result = write_register(bus, node_id, address, value, timeout=timeout)
    if as_json:
        show({"node_id": node_id, **result.to_dict()}, True)
    else:
        click.echo(f"Node {node_id}: register 0x{address:04X} = 0x{value:08X} acknowledged.")


@can_group.command("update")
@click.argument("image", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@can_options(require_node=True)
@click.option("--bin", "raw_binary", is_flag=True, help="Use a raw binary instead of Intel HEX.")
@json_option
@errors
def can_update(image, interface, node_id, raw_binary, as_json):
    """Update one node using its model-specific application image (CAN SDO)."""
    from ..update import update_can

    with can_connection(interface) as bus:
        click.echo(f"Updating node {node_id} from {image}.", err=True)
        result = update_can(bus, node_id, image, raw_binary=raw_binary, progress=update_progress())
    show_update(result, as_json)
