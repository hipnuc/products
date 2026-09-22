"""CLI for the public SDK. Machine data goes to stdout, diagnostics to stderr."""

from __future__ import annotations

import logging
import sys

import click

from .. import __version__
from . import can, modbus, serial, update


def _use_utf8(stream) -> None:
    """Pipes and files get UTF-8 on Windows too; a missing or odd stream is left alone."""
    try:
        if stream is not None and hasattr(stream, "reconfigure") and not stream.isatty():
            stream.reconfigure(encoding="utf-8")
    except (OSError, ValueError):
        pass


@click.group(
    invoke_without_command=True,
    help=(
        f"hihost {__version__}\n\n"
        "HiPNUC IMU/INS tools. Start with list, info, or read.\n\n"
        "Connection options follow the final command: read -p COM3 -b 115200. "
        "HIHOST_PORT, HIHOST_BAUDRATE, HIHOST_MODBUS_ID, HIHOST_CAN_INTERFACE and "
        "HIHOST_CAN_ID supply defaults."
    ),
)
@click.version_option(__version__, prog_name="hihost", message="%(prog)s %(version)s")
@click.option("-v", "--verbose", is_flag=True, help="Show SDK debug logs on stderr.")
@click.pass_context
def main(ctx, verbose):
    """Run the HiPNUC command-line tools."""
    _use_utf8(sys.stdout)
    _use_utf8(sys.stderr)
    if verbose:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
        logger = logging.getLogger("hipnuc")
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        def restore() -> None:
            logger.removeHandler(handler)
            logger.setLevel(logging.NOTSET)
            handler.close()

        ctx.call_on_close(restore)
    if ctx.invoked_subcommand is None:
        click.echo(ctx.get_help())


@main.command("help")
@click.argument("commands", nargs=-1)
@click.pass_context
def help_command(ctx, commands):
    """Show help for a command, for example: help can read."""
    context = ctx.parent
    for name in commands:
        group = context.command
        command = group.get_command(context, name) if isinstance(group, click.Group) else None
        if command is None:
            raise click.UsageError(f"Unknown command {name!r}")
        context = click.Context(command, info_name=name, parent=context)
    click.echo(context.get_help())


for _command in (*serial.COMMANDS, update.serial_update, modbus.modbus_group, can.can_group):
    main.add_command(_command)

__all__ = ["main"]
