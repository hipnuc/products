"""Serial firmware update subcommand and progress helpers shared with CAN."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict
from pathlib import Path

import click

from ._common import errors, json_option, to_json


def update_progress() -> Callable[[int, int], None]:
    last_percent = -1

    def report(written: int, total: int) -> None:
        nonlocal last_percent
        percent = written * 100 // total if total else 0
        if (percent in (0, 100) and percent != last_percent) or percent >= last_percent + 10:
            click.echo(f"Writing: {percent}%", err=True)
            last_percent = percent

    return report


def show_update(result, as_json: bool) -> None:
    if as_json:
        click.echo(to_json(asdict(result)))
    else:
        click.echo(
            f"Transfer acknowledged: {result.bytes_written} bytes. Application start requested."
        )
        if not result.start_acknowledged:
            click.echo("No start acknowledgement; the bootloader may already have reset.", err=True)
        click.echo("Reconnect and read the identity to confirm the new application.", err=True)


@click.command("update")
@click.argument("image", type=click.Path(exists=True, dir_okay=False, path_type=Path))
@click.option(
    "-p",
    "--port",
    required=True,
    envvar="HIHOST_PORT",
    show_envvar=True,
    help="Target serial port; no automatic discovery.",
)
@click.option(
    "-b",
    "--baudrate",
    type=click.IntRange(min=1),
    required=True,
    envvar="HIHOST_BAUDRATE",
    show_envvar=True,
    help="Baudrate of the target port.",
)
@json_option
@errors
def serial_update(image, port, baudrate, as_json):
    """Update one device using its model-specific Intel HEX application image."""
    from ..update import update_serial

    click.echo(f"Updating {port} at {baudrate} baud from {image}.", err=True)
    show_update(
        update_serial(image, port=port, baudrate=baudrate, progress=update_progress()), as_json
    )
