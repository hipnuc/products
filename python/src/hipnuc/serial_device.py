"""Synchronous serial sessions. One reader owns binary data and command responses."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
import logging
import math
import re
import threading
import time
from types import TracebackType
from typing import NoReturn

import serial
from serial.tools import list_ports

from ._connection import (
    SERIAL_OPEN_ERRORS,
    discovery_error_summary,
    is_baudrate_error,
    is_usb_port,
    open_error,
)
from .decoder import Decoder
from .errors import CommandTimeout, DeviceError, ResponseTimeout, TransportError, VerificationError
from .models import CommandResult, DeviceInfo, Sample

BAUDRATES = (115200, 921600, 460800, 230400, 256000, 57600, 38400, 19200, 9600, 4800)
_logger = logging.getLogger(__name__)
_ONCE_PATTERN = re.compile(r"LOG\s+(?:COM[1-4]\s+)?(?:HI91|HI81|HI83|GGA|RMC)\s+ONMARK\s+ONCE")


def _positive_timeout(value: float) -> float:
    if not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be finite and positive")
    return value


def _fields(text: str) -> dict[str, str]:
    """Read KEY=VALUE identity lines; later occurrences replace earlier ones."""
    fields = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            fields[key.strip().upper()] = value.strip()
    return fields


def _version(value: str | None) -> str | None:
    if value and re.fullmatch(r"\d{3}", value):
        return ".".join(value)
    return value


@dataclass(frozen=True)
class DiscoveredDevice:
    """A usable connection, with identity when the device answered LOG VERSION."""

    port: str
    baudrate: int
    info: DeviceInfo | None = None
    protocol: str | None = None
    identity_error: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            "port": self.port,
            "baudrate": self.baudrate,
            "info": self.info.to_dict() if self.info else None,
            "protocol": self.protocol,
            "identity_error": self.identity_error,
        }


@dataclass
class DiscoveryResult:
    """Scan results. Incomplete scans cannot establish that a device is unique."""

    devices: list[DiscoveredDevice]
    errors: dict[str, str]
    complete: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "devices": [device.to_dict() for device in self.devices],
            "errors": self.errors,
            "complete": self.complete,
        }


class SerialDevice:
    """A serial IMU/INS connection with a bounded pending sample queue.

    Opening does not change output configuration. Commands and reads are serialized.
    ``sample_sink`` receives every decoded sample, including during commands; the
    read queue holds at most ``queue_size`` samples, dropping oldest on overflow.
    ``dropped_samples`` makes a slow consumer observable. Sinks execute in the caller.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int | None = None,
        timeout: float = 2.0,
        *,
        scan_timeout: float = 30.0,
        decoder: Decoder | None = None,
        raw_sink: Callable[[bytes], None] | None = None,
        sample_sink: Callable[[Sample], None] | None = None,
        queue_size: int = 4096,
    ) -> None:
        if (baudrate is not None and baudrate <= 0) or queue_size <= 0:
            raise ValueError("baudrate and queue_size must be positive")
        if port is not None and not port.strip():
            raise ValueError("port must not be empty")
        self.port, self.baudrate = port, baudrate
        self.timeout = _positive_timeout(timeout)
        self.scan_timeout = _positive_timeout(scan_timeout)
        self.discovery_result: DiscoveryResult | None = None
        self.decoder = decoder if decoder is not None else Decoder()
        self.raw_sink, self.sample_sink = raw_sink, sample_sink
        self._samples: deque[Sample] = deque(maxlen=queue_size)
        self.dropped_samples = 0
        self._serial: serial.Serial | None = None
        self._lock = threading.RLock()
        self._command_prepared = False
        self.info: DeviceInfo | None = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def open(self) -> SerialDevice:
        """Open once, discovering omitted parameters; return this session."""
        with self._lock:
            if not self.is_open:
                selected = None
                if self.port is None or self.baudrate is None:
                    result = discover(
                        [self.port] if self.port is not None else None,
                        baudrates=(self.baudrate,) if self.baudrate is not None else BAUDRATES,
                        timeout=self.timeout,
                        scan_timeout=self.scan_timeout,
                    )
                    self.discovery_result = result
                    candidates = "; ".join(f"{d.port} at {d.baudrate} baud" for d in result.devices)
                    if not result.complete:
                        raise TransportError(
                            "Device search incomplete; increase scan_timeout or specify port "
                            f"and baudrate. Candidates: {candidates or 'none'}"
                        )
                    if not result.devices:
                        raise TransportError(
                            "No HiPNUC device found. " + discovery_error_summary(result.errors)
                        )
                    if len(result.devices) > 1:
                        raise TransportError(
                            f"Multiple HiPNUC devices found; specify port. Candidates: {candidates}"
                        )
                    selected = result.devices[0]
                    self.port, self.baudrate = selected.port, selected.baudrate
                try:
                    self._serial = serial.Serial(
                        self.port, self.baudrate, timeout=0, write_timeout=self.timeout
                    )
                except SERIAL_OPEN_ERRORS as exc:
                    raise open_error(self.port, exc) from exc
                self.decoder.reset()
                self._samples.clear()
                self._command_prepared = False
                self.info = selected.info if selected else None
            return self

    def close(self) -> None:
        """Release the port; safe to call again after it has been closed."""
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    def __enter__(self) -> SerialDevice:
        return self.open()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        self.close()

    def _require_open(self) -> serial.Serial:
        if not self.is_open:
            raise TransportError("SerialDevice is closed; use open() or a with block")
        return self._serial

    def _pump(self, remaining: float) -> list[Sample]:
        """Receive one chunk, dispatch recording callbacks, and queue its samples.

        The caller owns the session lock. When no bytes are ready, sleep for at
        most one millisecond of the caller's remaining timeout (in seconds).
        """
        ser = self._require_open()
        try:
            # Keep transport settings fixed: pySerial timeout setters reconfigure
            # the open UART on Windows and can interrupt an incoming USB stream.
            waiting = ser.in_waiting
            if not waiting:
                if remaining > 0:
                    time.sleep(min(0.001, remaining))
                return []
            data = ser.read(min(waiting, 65536))
        except (serial.SerialException, OSError) as exc:
            raise TransportError(f"Read from {self.port} failed: {exc}") from exc
        if not data:
            return []
        received = time.time_ns()
        if self.raw_sink is not None:
            self.raw_sink(data)
        samples = [replace(sample, received_time_ns=received) for sample in self.decoder.feed(data)]
        if samples and not self.decoder.buffered_bytes:
            self._command_prepared = True
        for sample in samples:
            if self.sample_sink is not None:
                self.sample_sink(sample)
            if len(self._samples) == self._samples.maxlen:
                self.dropped_samples += 1
            self._samples.append(sample)
        return samples

    def read(self, timeout: float | None = None) -> Sample:
        """Return the next queued/new sample; raise ResponseTimeout on an idle link."""
        with self._lock:
            self._require_open()
            duration = _positive_timeout(self.timeout if timeout is None else timeout)
            deadline = time.monotonic() + duration
            received = self.decoder.statistics["bytes_received"]
            unsupported = self.decoder.statistics["malformed_packets"]
            while not self._samples:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    count = self.decoder.statistics["bytes_received"] - received
                    rejected = self.decoder.statistics["malformed_packets"] - unsupported
                    if not count:
                        state = (
                            "No bytes received. Check power, TX/RX wiring and whether output "
                            "is enabled. For slow output, increase timeout (--timeout in the "
                            "CLI) beyond the output interval."
                        )
                    elif rejected:
                        # The checksum matched, so the connection is fine.
                        state = (
                            f"Received {rejected} valid frames this SDK does not decode. "
                            "Select a supported output message (HI91, HI81, HI83, GGA or RMC) "
                            "or update the SDK for newer firmware."
                        )
                    else:
                        state = (
                            f"Received {count} bytes but no valid measurement frames. "
                            "Check the baudrate and that a supported output message is enabled; "
                            "allow a longer timeout for slow or fragmented output."
                        )
                    raise ResponseTimeout(
                        f"{state} ({self.port} at {self.baudrate} baud, {duration:g}s)"
                    )
                self._pump(remaining)
            return self._samples.popleft()

    def iter_samples(self, idle_timeout: float | None = None) -> Iterator[Sample]:
        """Yield samples; raise on disconnect or idle for the specified/default timeout."""
        self._require_open()
        while self.is_open:
            yield self.read(idle_timeout)

    def _collect_response_lines(self, command: str, lines: list[str]) -> bool:
        acknowledged = False
        for line in self.decoder.drain_lines():
            line = line.strip()
            if not line or line == command:
                continue
            _logger.debug("Response line from %s: %r", self.port, line)
            lines.append(line)
            if re.match(r"^(ERROR|ERR|FAIL)(?:$|[\s:])", line, re.IGNORECASE):
                raise DeviceError(line, response="\n".join(lines))
            if line == "OK":
                acknowledged = True
        if sum(map(len, lines)) > 65536:
            self._close_after_error()
            raise DeviceError("Command response exceeds 64 KiB", response="\n".join(lines))
        return acknowledged

    def _close_after_error(self) -> None:
        """Release a failed session without replacing its original exception."""
        try:
            self.close()
        except OSError:
            pass

    def _command_timeout(self, message: str, *, sent: bool) -> NoReturn:
        """End an unconfirmed transaction without replaying or reopening it."""
        if self.decoder.needs_resync or self.decoder.buffered_bytes:
            message += " Serial input is incomplete or desynchronized."
        if sent:
            self._close_after_error()
            message += (
                " Session closed. Wait for the device operation to finish before reconnecting; "
                "reopening the port does not cancel it."
            )
        raise CommandTimeout(message, sent=sent)

    def command(
        self, command: str, *, timeout: float | None = None, response: str = "auto"
    ) -> CommandResult:
        """Send one ASCII command line and wait for its ``OK``/``ERR`` reply.

        ``response="auto"`` waits for a terminal ``OK``. ``"none"`` sends and
        closes this session; reopening does not synchronize old replies.
        ``LOG [COMn] <MSG> ONMARK ONCE`` requires ``response="none"`` because
        measurements do not confirm that a command executed.
        The device prints nothing for an unknown command, so an unknown
        command surfaces as ``ResponseTimeout``. Binary bytes never count as an
        acknowledgement. The first command after opening waits for a clean
        frame boundary so the reply cannot be confused with an ongoing stream.
        Preparation and response waiting share one timeout. Known corruption
        waits for a valid frame within that timeout before sending.
        ``CommandTimeout.sent`` reports whether the command was
        written; an unconfirmed write closes the session. No command is retried
        and no failed session is automatically reopened. Send-only calls skip
        the corruption wait.
        """
        command = command.strip()
        if not command or "\r" in command or "\n" in command:
            raise ValueError("command must contain exactly one nonempty line")
        if response not in {"auto", "none"}:
            raise ValueError("response must be auto or none")
        if response == "auto" and _ONCE_PATTERN.fullmatch(command.upper()):
            raise ValueError(
                "LOG [COMn] <MSG> ONMARK ONCE has no command acknowledgement; "
                "use response='none' (CLI: --no-reply)."
            )
        encoded = (command + "\r\n").encode("ascii")
        duration = _positive_timeout(self.timeout if timeout is None else timeout)
        with self._lock:
            ser = self._require_open()
            deadline = time.monotonic() + duration
            write_started = False
            try:
                if not self._command_prepared:
                    # Opening a USB UART can start in the middle of a frame.
                    # Receive a clean boundary before issuing the first command;
                    # silent devices proceed after this bounded observation window.
                    # Leave part of a short budget for the actual response.
                    startup = min(
                        duration / 2,
                        max(0.05, (self.decoder.max_payload_size + 6) * 10 / self.baudrate),
                    )
                    ready_by = min(deadline, time.monotonic() + startup)
                    while (
                        not self._command_prepared
                        and (remaining := ready_by - time.monotonic()) > 0
                    ):
                        self._pump(remaining)
                        if self.decoder.drain_lines() and not self.decoder.buffered_bytes:
                            break
                    self._command_prepared = True
                # Route bytes already queued by the OS before starting this
                # transaction; they may include measurements to record.
                queued = ser.in_waiting
                while queued:
                    if time.monotonic() >= deadline:
                        self._command_timeout(
                            "Timed out draining the serial receive queue; command was not sent",
                            sent=False,
                        )
                    before = self.decoder.statistics["bytes_received"]
                    self._pump(0)
                    consumed = self.decoder.statistics["bytes_received"] - before
                    if not consumed:
                        break
                    queued = max(0, queued - consumed)
                if response == "auto":
                    while self.decoder.needs_resync:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            self._command_timeout(
                                f"Cannot synchronize serial input before {command!r}; "
                                "command was not sent",
                                sent=False,
                            )
                        self._pump(remaining)
                if time.monotonic() >= deadline:
                    self._command_timeout(
                        f"Timed out preparing {command!r}; command was not sent", sent=False
                    )
                self.decoder.drain_lines(discard_partial=True)
                _logger.debug("Sending command %r to %s", command, self.port)
                write_started = True
                try:
                    written = ser.write(encoded)
                except OSError as exc:
                    raise TransportError(f"Write to {self.port} failed: {exc}") from exc
                if written != len(encoded):
                    raise TransportError("Short serial write")
                if response == "none":
                    self.close()
                    return CommandResult(command, "", False)
                lines: list[str] = []
                while time.monotonic() < deadline:
                    try:
                        self._pump(deadline - time.monotonic())
                    except DeviceError:
                        self._close_after_error()
                        raise
                    if time.monotonic() >= deadline:
                        break
                    if self._collect_response_lines(command, lines):
                        return CommandResult(command, "\n".join(lines), True)
                self._command_timeout(
                    f"No OK reply to {command!r} within {duration:g}s. Check the command "
                    "spelling; the device does not answer unknown commands. "
                    "The command was sent; execution is unconfirmed.",
                    sent=True,
                )
            except CommandTimeout:
                if write_started and self.is_open:
                    self._close_after_error()
                raise
            except (TransportError, OSError) as exc:
                self._close_after_error()
                if not write_started:
                    self._command_timeout(
                        f"Cannot prepare {command!r}; command was not sent: {exc}", sent=False
                    )
                raise
            except DeviceError:
                raise
            except BaseException:
                if write_started:
                    self._close_after_error()
                raise

    def read_info(self, *, timeout: float | None = None) -> DeviceInfo:
        """Query ``LOG VERSION`` and require a recognizable product identity."""
        result = self.command("LOG VERSION", timeout=timeout)
        values = _fields(result.text)
        info = DeviceInfo(
            product_name=values.get("PNAME"),
            firmware_version=_version(values.get("APP_VER", values.get("SW_VERSION"))),
            bootloader_version=_version(values.get("BL_VER", values.get("BL_VERSION"))),
            serial_number=values.get("UUID", values.get("SN")),
            build=values.get("BUILD"),
            raw_response=result.text,
        )
        if not info.product_name or not (info.firmware_version or info.serial_number):
            raise VerificationError(
                "LOG VERSION returned no usable device identity", response=result.text
            )
        self.info = info
        return info

    def save_config(self) -> CommandResult:
        return self.command("SAVECONFIG")

    def _wait_for_info(self, timeout: float) -> DeviceInfo:
        """Wait for a reboot; close if repeated identity probes may still reply."""
        deadline = time.monotonic() + timeout
        last_error: Exception | None = None
        queries = 0
        try:
            while time.monotonic() < deadline:
                try:
                    if not self.is_open:
                        self.open()
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self.decoder.reset()
                    queries += 1
                    info = self.read_info(timeout=min(self.timeout, remaining))
                    if time.monotonic() >= deadline:
                        break
                    if queries > 1:
                        self.close()
                    return info
                except (TransportError, VerificationError) as exc:
                    last_error = exc
                    if isinstance(exc, TransportError):
                        # A USB reset can leave an open-looking but invalid handle.
                        self._close_after_error()
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(min(0.05, remaining))
            raise ResponseTimeout(
                f"Device did not answer on {self.port} at {self.baudrate} baud within "
                f"{timeout:g}s; reconnect with `hihost scan` if it moved."
            ) from last_error
        except BaseException:
            self._close_after_error()
            raise

    def set_baudrate(
        self,
        baudrate: int,
        *,
        recovery_timeout: float = 5.0,
        save: bool = False,
    ) -> DeviceInfo:
        """Send ``SERIALCONFIG``, switch the host to the new speed and query identity.

        Only the currently connected device port is changed. The device answers
        ``OK`` and switches immediately. A missing ACK ends the session without
        retrying at another speed. ``save=True`` saves after the identity query.
        """
        if baudrate not in BAUDRATES:
            raise ValueError(f"unsupported baudrate; expected one of {BAUDRATES}")
        _positive_timeout(recovery_timeout)
        with self._lock:
            self._require_open()
            self.command(f"SERIALCONFIG {baudrate}", timeout=0.5)
            self.baudrate = baudrate
            try:
                self._require_open().baudrate = baudrate
            except SERIAL_OPEN_ERRORS as exc:
                self._close_after_error()
                raise TransportError(
                    f"Cannot configure {self.port} at {baudrate} baud: {exc}. "
                    f"The device acknowledged the change and may already use {baudrate} baud. "
                    "Session closed; reconnect using an adapter that supports this speed."
                ) from exc
            self.decoder.reset()
            try:
                info = self.read_info(timeout=recovery_timeout)
            except (TransportError, DeviceError):
                self._close_after_error()
                raise
            if save:
                self.save_config()
            return info

    def reboot(self, *, recovery_timeout: float = 5.0) -> DeviceInfo:
        """Restart once and wait for identity; repeated identity probes end the session."""
        _positive_timeout(recovery_timeout)
        with self._lock:
            self._require_open()
            try:
                self.command("REBOOT", timeout=0.5)
            except CommandTimeout as exc:
                if not exc.sent:
                    raise
            except TransportError:
                # A native USB port may disappear while the device resets;
                # _wait_for_info reopens it.
                self._close_after_error()
            # Do not query identity while the pre-reset application still runs.
            time.sleep(0.01)
            self._samples.clear()
            self.decoder.reset()
            return self._wait_for_info(recovery_timeout)


def discover(
    ports: list[str] | None = None,
    *,
    baudrates: tuple[int, ...] = BAUDRATES,
    timeout: float = 2.0,
    scan_timeout: float = 30.0,
) -> DiscoveryResult:
    """Find usable USB serial devices without changing their configuration.

    An explicit ``ports`` list also permits native UARTs and other serial devices.
    Observe incoming data, then request identity with LOG VERSION. Valid HiPNUC
    binary data is usable even without an identity response; NMEA alone never
    identifies a brand. Each probe terminates any unfinished device command line
    before querying identity once. Every probe handle is closed, including on Ctrl-C.
    ``timeout`` limits each identity response; ``scan_timeout`` bounds the scan.
    """
    _positive_timeout(timeout)
    deadline = time.monotonic() + _positive_timeout(scan_timeout)
    if not baudrates or any(baud <= 0 for baud in baudrates):
        raise ValueError("baudrates must contain positive integers")
    names = (
        ports if ports is not None else [p.device for p in list_ports.comports() if is_usb_port(p)]
    )
    result = DiscoveryResult([], {}, True)
    for name in dict.fromkeys(names):
        fallback = None
        best_error = "No bytes received or identity response at the tested baudrates"
        received_any = False
        for baud in baudrates:
            if time.monotonic() >= deadline:
                result.complete = False
                if fallback is not None:
                    result.devices.append(fallback)
                result.errors[name] = "Search deadline reached before all baudrates were tested"
                return result
            remaining = deadline - time.monotonic()
            observation = min(timeout, 0.05, remaining / 2)
            device = SerialDevice(name, baud, min(timeout, remaining - observation))
            _logger.info("Checking %s at %s baud (identity timeout %gs)...", name, baud, timeout)
            try:
                device.open()
            except TransportError as exc:
                if is_baudrate_error(exc):
                    best_error = str(exc)
                    _logger.info("%s at %s baud: %s; trying the next baudrate.", name, baud, exc)
                    continue
                # An occupied/missing port cannot be fixed by changing its baudrate.
                result.errors[name] = str(exc)
                _logger.info("%s: %s; skipping this port.", name, exc)
                if fallback is not None:
                    result.devices.append(fallback)
                break
            try:
                protocol = None

                def observe(sample: Sample) -> None:
                    nonlocal protocol
                    if sample.metadata.get("protocol") == "hipnuc_binary":
                        protocol = "hipnuc_binary"

                device.sample_sink = observe
                # Wrong-baud probes can leave an unfinished line in the device CLI.
                try:
                    if device._require_open().write(b"\r\n") != 2:
                        raise TransportError("Short serial write while preparing identity query")
                except OSError as exc:
                    raise TransportError(f"Cannot prepare identity query on {name}: {exc}") from exc
                passive_until = min(deadline, time.monotonic() + observation)
                while (remaining := passive_until - time.monotonic()) > 0:
                    device._pump(remaining)
                # The bounded observation above serves as discovery's startup
                # window; do not spend a second window at every baudrate.
                device._command_prepared = True
                info = None
                identity_error = None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    identity_error = "Identity query skipped: search deadline reached"
                else:
                    try:
                        info = device.read_info(timeout=min(timeout, remaining))
                    except (ResponseTimeout, DeviceError) as exc:
                        identity_error = str(exc)
                count = device.decoder.statistics["bytes_received"]
                received_any = received_any or bool(count)
                if info is not None or protocol is not None:
                    candidate = DiscoveredDevice(name, baud, info, protocol, identity_error)
                    if info is not None:
                        _logger.info("%s at %s baud: found %s.", name, baud, info.product_name)
                        result.devices.append(candidate)
                        if time.monotonic() >= deadline:
                            result.complete = False
                        break
                    # A USB bridge may deliver buffered valid frames after a
                    # baud change. Prefer a completed identity exchange at a
                    # later baud; retain data-only access if none answers.
                    if fallback is None:
                        fallback = candidate
                    _logger.info(
                        "%s at %s baud: valid binary data, identity unavailable; "
                        "checking remaining baudrates.",
                        name,
                        baud,
                    )
                elif count:
                    _logger.info(
                        "%s at %s baud: received %s bytes, no HiPNUC match (%s).",
                        name,
                        baud,
                        count,
                        identity_error or "unrecognized data",
                    )
                else:
                    _logger.info("%s at %s baud: no bytes received.", name, baud)
                if count:
                    best_error = (
                        f"Received bytes at {baud} baud but no HiPNUC identity or valid "
                        f"HiPNUC binary frames. {identity_error or ''}"
                    )
                elif not received_any and identity_error:
                    best_error = f"No bytes received at tested baudrates. {identity_error}"
                if time.monotonic() >= deadline:
                    result.complete = False
                    if fallback is not None:
                        result.devices.append(fallback)
                    result.errors[name] = best_error
                    return result
            except TransportError as exc:
                result.errors[name] = str(exc)
                _logger.info("%s: %s; skipping this port.", name, exc)
                # Keep a data-only candidate found at an earlier baudrate.
                if fallback is not None:
                    result.devices.append(fallback)
                break
            finally:
                device.close()
        else:
            if fallback is not None:
                result.devices.append(fallback)
            else:
                result.errors[name] = best_error
    return result
