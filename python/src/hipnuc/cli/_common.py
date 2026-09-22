"""Helpers shared by the hihost subcommands: output, options and sample display."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import errno
from functools import partial, wraps
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import click

from ..errors import HipnucError
from ..models import Sample
from ..recording import Recorder


def to_json(value) -> str:
    return json.dumps(
        value.to_dict() if hasattr(value, "to_dict") else value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )


def show(value, as_json: bool) -> None:
    if as_json:
        click.echo(to_json(value))
    elif hasattr(value, "text"):
        click.echo(value.text or "Sent; no acknowledgement received.")
    else:
        fields = value.to_dict() if hasattr(value, "to_dict") else value
        for key, item in fields.items():
            if item is not None and key != "raw_response":
                click.echo(f"{key}: {item}")


def errors(func):
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


def verbose_enabled() -> bool:
    """True when the running command was started with ``hihost -v``."""
    ctx = click.get_current_context(silent=True)
    return bool(ctx is not None and ctx.find_root().params.get("verbose"))


def finite(_ctx, param, value):
    if value is not None and not math.isfinite(value):
        raise click.BadParameter("must be finite", param=param)
    return value


def timeout_option(func=None, *, help="Response/idle timeout in seconds."):
    if func is None:
        return partial(timeout_option, help=help)
    return click.option(
        "--timeout",
        type=click.FloatRange(min=0, min_open=True),
        callback=finite,
        default=2.0,
        show_default=True,
        help=help,
    )(func)


def json_option(func):
    return click.option("--json", "as_json", is_flag=True, help="Print JSON.")(func)


def record_options(func=None, *, finite_input=False):
    """Recording and display switches shared by read, decode and CAN commands."""
    if func is None:
        return partial(record_options, finite_input=finite_input)
    func = click.option("--overwrite", is_flag=True, help="Replace existing recording files.")(func)
    func = click.option("--quiet", is_flag=True, help="Collect without printing samples.")(func)
    func = click.option("--jsonl", is_flag=True, help="Print every sample as SI JSONL.")(func)
    if not finite_input:
        func = click.option(
            "--display-rate",
            type=click.FloatRange(min=0, min_open=True),
            callback=finite,
            default=5.0,
            show_default=True,
            help="Human display limit per message type, in Hz.",
        )(func)
        func = click.option(
            "--duration",
            type=click.FloatRange(min=0, min_open=True),
            callback=finite,
            help="Acquisition time in seconds; finish the current input before stopping.",
        )(func)
    return click.option(
        "--record",
        type=click.Path(path_type=Path, dir_okay=False),
        help="Write every sample as SI JSONL.",
    )(func)


def integer(value: str) -> int:
    try:
        return int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as exc:
        raise click.BadParameter("expected a decimal integer or 0x-prefixed hexadecimal") from exc


@contextmanager
def stop_on_interrupt() -> Iterator[threading.Event]:
    """Turn Ctrl-C and SIGTERM into a stop request; a second Ctrl-C aborts."""
    stopped = threading.Event()

    def request_stop(*_args) -> None:
        if stopped.is_set():
            raise KeyboardInterrupt
        stopped.set()

    previous = {}
    if threading.current_thread() is threading.main_thread():
        for signum in (signal.SIGINT, getattr(signal, "SIGTERM", None)):
            if signum is None:
                continue
            try:
                previous[signum] = signal.signal(signum, request_stop)
            except (ValueError, OSError):
                pass  # Unsupported here; KeyboardInterrupt handling still applies.
    try:
        yield stopped
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler if handler is not None else signal.SIG_DFL)


def finish(stopped: threading.Event) -> None:
    """Exit 130 for a requested stop, after acquisition resources have closed."""
    if stopped.is_set():
        raise click.exceptions.Exit(130)


def human_sample(sample: Sample) -> str:
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
    scalar("course over ground", values.get("course_over_ground_rad"), "°", 180 / math.pi)
    if len(parts) == 1:
        parts.append(to_json(values))
    if not sample.complete or sample.issues:
        parts.append("partial/invalid: " + ", ".join(sample.issues or ("incomplete decode",)))
    return "  ".join(parts)


def _silence_pipe(stream) -> None:
    """Redirect a broken stream so the interpreter's final flush cannot fail."""
    try:
        with open(os.devnull, "w") as devnull:
            os.dup2(devnull.fileno(), stream.fileno())
    except (OSError, ValueError, AttributeError):
        pass


class SampleConsumer:
    """Record every sample, limit its display independently and measure rates.

    Rates use host reception time per message type (and station/node ID) and
    need at least two timestamped samples. CANFD83 frame counters are checked
    per node: a gap below 128 counts as lost frames, a larger jump as a
    counter reset (device restart), because an 8-bit counter cannot tell
    them apart. A closed output pipe ends the display, not the acquisition.
    """

    def __init__(
        self,
        recording: Recorder | None,
        stopped: threading.Event,
        *,
        jsonl: bool,
        quiet: bool,
        display_rate: float | None,
    ) -> None:
        self.recording = recording
        self.stopped = stopped
        self.jsonl, self.quiet, self.display_rate = jsonl, quiet, display_rate
        self.samples = 0
        self.canfd83_lost = 0
        self.canfd83_resets = 0
        self._counts: dict[tuple[str, int | None], int] = {}
        self._first: dict[tuple[str, int | None], int] = {}
        self._last: dict[tuple[str, int | None], int] = {}
        self._last_display: dict[tuple[str, int | None], float] = {}
        self._sequence: dict[int | None, int] = {}

    def __call__(self, sample: Sample) -> None:
        if self.recording is not None:
            self.recording.write(sample)
        key = (sample.type, sample.metadata.get("device_id", sample.values.get("node_id")))
        self.samples += 1
        if sample.received_time_ns is not None:
            self._counts[key] = self._counts.get(key, 0) + 1
            self._first.setdefault(key, sample.received_time_ns)
            self._last[key] = sample.received_time_ns
        if sample.type == "CANFD83" and "sequence" in sample.metadata:
            sequence = sample.metadata["sequence"]
            previous = self._sequence.get(key[1])
            if previous is not None:
                gap = (sequence - previous - 1) & 0xFF
                if gap < 128:
                    self.canfd83_lost += gap
                else:
                    self.canfd83_resets += 1
            self._sequence[key[1]] = sequence
        now = time.monotonic()
        if self.quiet or (self.stopped.is_set() and not self.jsonl):
            return
        if (
            self.jsonl
            or self.display_rate is None
            or now - self._last_display.get(key, float("-inf")) >= 1 / self.display_rate
        ):
            try:
                click.echo(to_json(sample) if self.jsonl else human_sample(sample))
                self._last_display[key] = now
            except KeyboardInterrupt:
                self.stopped.set()
            except OSError as exc:
                if exc.errno not in (errno.EPIPE, errno.EINVAL):
                    raise
                self._close_output()

    def _close_output(self) -> None:
        """The reader went away: silence display while acquisition continues."""
        self.quiet = True
        _silence_pipe(sys.stdout)

    def rate_summary(self) -> str:
        """Measured rates such as ``HI91 100.0 Hz``; empty without two samples."""
        parts = []
        for key, count in self._counts.items():
            if count < 2:
                continue
            elapsed = (self._last[key] - self._first[key]) / 1e9
            if elapsed <= 0:
                continue
            label = key[0] if key[1] is None else f"{key[0]} id={key[1]}"
            parts.append(f"{label} {(count - 1) / elapsed:.1f} Hz")
        return ", ".join(parts)


def status(message: str) -> None:
    """Diagnostics to stderr; a stderr shared with a closed pipe must not fail the command."""
    try:
        click.echo(message, err=True)
    except OSError as exc:
        if exc.errno not in (errno.EPIPE, errno.EINVAL):
            raise
        _silence_pipe(sys.stderr)


def stop_summary(consumer: SampleConsumer, recording: Recorder | None, detail: str = "") -> str:
    """One stop line: sample count, transport detail, rates, losses, recording."""
    text = f"{consumer.samples} samples"
    if detail:
        text += f", {detail}"
    text += "."
    rates = consumer.rate_summary()
    if rates:
        text += f" Rates: {rates}."
    if consumer.canfd83_lost or consumer.canfd83_resets:
        text += (
            f" CANFD83 lost frames: {consumer.canfd83_lost}, "
            f"counter resets: {consumer.canfd83_resets}."
        )
    if recording is not None:
        text += f" Recorded: {recording.samples_written} samples"
        if recording.raw_bytes_written:
            text += f", {recording.raw_bytes_written} raw bytes"
        text += "."
    return text
