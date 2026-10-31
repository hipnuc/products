"""Serial firmware update subcommand and helpers shared with CAN."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict
import logging
from pathlib import Path
import time

import click

from ._common import errors, json_option, sdk_messages, to_json


def update_options(func):
    return click.option(
        "--no-verify", is_flag=True, help="Do not reconnect to confirm the new firmware."
    )(func)


def update_progress(verify: bool) -> Callable[[int, int], None]:
    last_percent = -1

    def report(written: int, total: int) -> None:
        nonlocal last_percent
        percent = written * 100 // total if total else 0
        if (percent in (0, 100) and percent != last_percent) or percent >= last_percent + 10:
            click.echo(f"Writing: {percent}%", err=True)
            last_percent = percent
            if percent == 100 and verify:
                click.echo("Waiting for the new firmware to start...", err=True)

    return report


def run_update(image: Path, as_json: bool, update: Callable) -> None:
    """Print the image, run ``update(path)`` and summarize the result."""
    from .._firmware_image import load_image

    loaded = load_image(image)
    built = f", built {loaded.build}" if loaded.build else ""
    click.echo(
        f"Image {image.name}: version {loaded.version or 'unknown'}{built}, "
        f"{len(loaded.data)} bytes.",
        err=True,
    )
    started = time.monotonic()
    with sdk_messages("hipnuc.update", logging.WARNING):  # The power-cycle request.
        result = update(image)
    elapsed = time.monotonic() - started
    if as_json:
        click.echo(to_json({**asdict(result), "elapsed_s": round(elapsed, 1)}))
        return
    previous = f" from {result.previous_version}" if result.previous_version else ""
    if result.verified_version:
        click.echo(
            f"Updated{previous} to {result.verified_version} in {elapsed:.0f} s; "
            "the new firmware is running."
        )
    else:
        click.echo(
            f"Transferred {result.bytes_written} bytes in {elapsed:.0f} s. "
            "Reconnect and read the identity to confirm the new firmware."
        )


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
    envvar="HIHOST_BAUDRATE",
    show_envvar=True,
    help="Current baudrate of the device; detected when omitted.",
)
@update_options
@json_option
@errors
def serial_update(image, port, baudrate, no_verify, as_json):
    """Update one device from its model-specific Intel HEX application image.

    Keep the device still until the update finishes. If the device does not
    answer, for example after an interrupted update, run the same command
    again and power-cycle the device when asked.
    """
    from ..update import update_serial

    click.echo(f"Updating the device on {port}; keep it still until the update finishes.", err=True)
    run_update(
        image,
        as_json,
        lambda path: update_serial(
            path,
            port=port,
            baudrate=baudrate,
            verify=not no_verify,
            progress=update_progress(not no_verify),
        ),
    )
