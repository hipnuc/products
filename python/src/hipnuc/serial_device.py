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

import serial
from serial.tools import list_ports

from .decoder import Decoder
from .errors import DeviceError, ResponseTimeout, TransportError, VerificationError
from .models import CommandResult, DeviceInfo, Sample

BAUDRATES = (115200, 921600, 460800, 230400, 256000, 57600, 38400, 19200, 9600, 4800)
_logger = logging.getLogger(__name__)


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
        self._write_completed = False
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
                        details = "; ".join(f"{p}: {e}" for p, e in result.errors.items())
                        raise TransportError(
                            "No HiPNUC device found. " + (details or "No serial ports found.")
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
                except (serial.SerialException, OSError) as exc:
                    raise TransportError(f"Cannot open {self.port}: {exc}") from exc
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
            while not self._samples:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    count = self.decoder.statistics["bytes_received"] - received
                    state = (
                        "No bytes received"
                        if not count
                        else f"Received {count} bytes but no valid measurement frames"
                    )
                    raise ResponseTimeout(
                        f"{state} on {self.port} at {self.baudrate} baud within {duration:g}s"
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
            lines.append(line)
            if re.match(r"^(ERROR|ERR|FAIL)(?:$|[\s:])", line, re.IGNORECASE):
                raise DeviceError(line, response="\n".join(lines))
            if line == "OK":
                acknowledged = True
        if sum(map(len, lines)) > 65536:
            raise DeviceError("Command response exceeds 64 KiB", response="\n".join(lines))
        return acknowledged

    def _finish_ack(self, command: str, lines: list[str], deadline: float) -> CommandResult:
        """Consume the documented old SERIALCONFIG double-ACK tail.

        Older firmware prints OK, waits 5 ms, then the CLI prints another OK.
        Allow six 8N1 character times plus that delay, with a small host read
        batching floor. Binary/NMEA samples do not extend the ASCII quiet time.
        A hard limit keeps unsolicited ASCII output from extending this forever.
        """
        quiet = max(0.01, 0.005 + 6 * 10 / self.baudrate)
        quiet_until = time.monotonic() + quiet
        stop_at = min(time.monotonic() + 3 * quiet, deadline)
        while (remaining := min(quiet_until, stop_at) - time.monotonic()) > 0:
            self._pump(remaining)
            before = len(lines)
            self._collect_response_lines(command, lines)
            if len(lines) != before:
                quiet_until = time.monotonic() + quiet
        return CommandResult(command, "\n".join(lines), True)

    def command(
        self, command: str, *, timeout: float | None = None, response: str = "auto"
    ) -> CommandResult:
        """Send one ASCII command without changing the stream enable state.

        response: auto (ACK or matching ONMARK ONCE data), ack (terminal OK),
        text (nonempty text after a short idle gap), none (send only).
        Errors are terminal lines, never an arbitrary 'OK' substring in a packet.
        Raw lifecycle commands remain raw; managed helpers verify communication changes.
        timeout limits sending and waiting for the response; the transport write
        timeout is set at open. Blocking driver calls also obey that transport limit.
        The first command briefly aligns with an existing stream before sending.
        After OK, a bounded ASCII quiet interval collects older duplicate ACKs.
        This protocol has no transaction ID: arbitrarily delayed responses cannot
        be assigned with certainty. Use readback for configuration verification.
        """
        command = command.strip()
        if not command or "\r" in command or "\n" in command:
            raise ValueError("command must contain exactly one nonempty line")
        if response not in {"auto", "ack", "text", "none"}:
            raise ValueError("response must be auto, ack, text, or none")
        duration = _positive_timeout(self.timeout if timeout is None else timeout)
        with self._lock:
            self._write_completed = False
            ser = self._require_open()
            # Route bytes already queued by the OS before starting this transaction.
            # No reset_input_buffer(): those bytes may include measurements to record.
            try:
                if not self._command_prepared:
                    # Opening a USB UART can start in the middle of a frame.
                    # Receive a clean boundary before issuing the first command;
                    # silent devices proceed after this bounded observation window.
                    startup = min(
                        duration,
                        max(0.05, (self.decoder.max_payload_size + 6) * 10 / self.baudrate),
                    )
                    ready_by = time.monotonic() + startup
                    while (
                        not self._command_prepared
                        and (remaining := ready_by - time.monotonic()) > 0
                    ):
                        self._pump(remaining)
                        if self.decoder.drain_lines() and not self.decoder.buffered_bytes:
                            break
                    self._command_prepared = True
                deadline = time.monotonic() + duration
                queued = ser.in_waiting
                while queued:
                    if time.monotonic() >= deadline:
                        raise ResponseTimeout("Timed out draining the serial receive queue")
                    before = self.decoder.statistics["bytes_received"]
                    self._pump(0)
                    consumed = self.decoder.statistics["bytes_received"] - before
                    if not consumed:
                        break
                    queued = max(0, queued - consumed)
                self.decoder.drain_lines(discard_partial=True)
                encoded = (command + "\r\n").encode("ascii")
                if ser.write(encoded) != len(encoded):
                    raise TransportError("Short serial write")
                self._write_completed = True
            except (serial.SerialException, OSError) as exc:
                raise TransportError(f"Write to {self.port} failed: {exc}") from exc
            if response == "none":
                return CommandResult(command, "", False)
            lines: list[str] = []
            last_text = time.monotonic()
            once = re.fullmatch(
                r"LOG\s+(?:COM[1-4]\s+)?(HI91|HI81|HI83|GGA|RMC|SXT|VTG|GSA|GSV)\s+ONMARK\s+ONCE",
                command.upper(),
            )
            while time.monotonic() < deadline:
                samples = self._pump(deadline - time.monotonic())
                before = len(lines)
                acknowledged = self._collect_response_lines(command, lines)
                if len(lines) != before:
                    last_text = time.monotonic()
                if acknowledged:
                    return self._finish_ack(command, lines, deadline)
                if response == "auto" and once and any(s.type == once[1] for s in samples):
                    return CommandResult(command, "\n".join(lines), False, verified=True)
                if response == "text" and lines and time.monotonic() - last_text >= 0.1:
                    return CommandResult(command, "\n".join(lines), False)
            if response == "text" and lines:
                return CommandResult(command, "\n".join(lines), False)
            raise ResponseTimeout(f"No complete response to {command!r} within {duration:g}s")

    def read_info(self) -> DeviceInfo:
        result = self.command("LOG VERSION")
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

    def _recover_identity(
        self, expected: DeviceInfo, baudrates: tuple[int, ...], timeout: float
    ) -> DeviceInfo:
        deadline = time.monotonic() + _positive_timeout(timeout)
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            for baud in dict.fromkeys(baudrates):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                try:
                    self.baudrate = baud
                    if not self.is_open:
                        self.open()
                    else:
                        try:
                            self._require_open().baudrate = baud
                        except (serial.SerialException, OSError) as exc:
                            raise TransportError(
                                f"Cannot configure {self.port} at {baud}: {exc}"
                            ) from exc
                    self.decoder.reset()
                    old_timeout = self.timeout
                    self.timeout = min(old_timeout, remaining)
                    try:
                        try:
                            found = self.read_info()
                        except VerificationError as exc:
                            # A delayed old OK is not an identity response.
                            last_error = exc
                            continue
                    finally:
                        self.timeout = old_timeout
                    if expected.serial_number:
                        if found.serial_number != expected.serial_number:
                            self.info = expected
                            raise VerificationError("Device identity changed during recovery")
                    elif not found.product_name or found.product_name != expected.product_name:
                        self.info = expected
                        raise VerificationError("Cannot verify the same product after recovery")
                    return found
                except ResponseTimeout as exc:
                    last_error = exc
                    continue
                except TransportError as exc:
                    last_error = exc
                    # A USB reset can leave an open-looking but invalid handle.
                    # Retry the same port name after closing it, never a new port
                    # selected only by a similar product name.
                    try:
                        self.close()
                    except (serial.SerialException, OSError):
                        pass
                    self.info = expected
            time.sleep(min(0.05, max(0, deadline - time.monotonic())))
        raise ResponseTimeout(
            f"Device did not return with matching identity on {self.port}; "
            "reopen it when available, using its new port name if USB enumeration changed"
        ) from last_error

    def set_baudrate(
        self,
        baudrate: int,
        *,
        device_port: str | None = None,
        recovery_timeout: float = 5.0,
        save: bool = False,
    ) -> DeviceInfo:
        """Change the current device port and verify its identity at the new baud.

        The default ``SERIALCONFIG <baud>`` syntax is shared by IMU and INS.
        Pass ``device_port`` only to explicitly name a supported device COM port.
        """
        if baudrate not in BAUDRATES:
            raise ValueError(f"unsupported baudrate; expected one of {BAUDRATES}")
        if device_port is not None and not re.fullmatch(r"COM[1-4]", device_port.upper()):
            raise ValueError("device_port must be COM1..COM4")
        _positive_timeout(recovery_timeout)
        with self._lock:
            expected = self.info or self.read_info()
            old_baud = self.baudrate
            target = f"{device_port.upper()} " if device_port is not None else ""
            try:
                self.command(f"SERIALCONFIG {target}{baudrate}", timeout=0.3)
            except ResponseTimeout:
                pass  # Some old firmware switches baud before the ACK reaches the host.
            except TransportError:
                if not self._write_completed:
                    raise
                try:
                    self.close()
                except (serial.SerialException, OSError):
                    pass
            found = self._recover_identity(expected, (baudrate, old_baud), recovery_timeout)
            if self.baudrate != baudrate:
                raise VerificationError("Device still responds only at the previous baudrate")
            if save:
                self.save_config()
            return found

    def reboot(self, *, recovery_timeout: float = 5.0) -> DeviceInfo:
        """Send reset once and confirm the same identity is reachable afterward.

        Reconnection does not prove that every setting took effect or that a
        reset physically occurred; no unrelated configuration is auto-saved.
        """
        _positive_timeout(recovery_timeout)
        with self._lock:
            expected = self.info or self.read_info()
            try:
                self.command("REBOOT", timeout=0.3)
            except ResponseTimeout:
                pass
            except TransportError:
                if not self._write_completed:
                    raise
                # A native USB port may disappear while sending/receiving reset.
                try:
                    self.close()
                except (serial.SerialException, OSError):
                    pass
            # Some fielded builds ACK before their 5-ms reset timer expires.
            # Do not query identity while that pre-reset application still runs.
            time.sleep(0.01)
            return self._recover_identity(expected, (self.baudrate,), recovery_timeout)


def discover(
    ports: list[str] | None = None,
    *,
    baudrates: tuple[int, ...] = BAUDRATES,
    timeout: float = 2.0,
    scan_timeout: float = 30.0,
) -> DiscoveryResult:
    """Find usable serial devices without changing their configuration.

    Observe incoming data, then request identity with LOG VERSION. Valid HiPNUC
    binary data is usable even without an identity response; NMEA alone never
    identifies a brand. A binary candidate's timed-out identity query is retried
    once at the same baudrate. Every probe handle is closed, including on Ctrl-C.
    ``timeout`` limits each identity response; ``scan_timeout`` bounds the scan.
    OS port opening itself is subject to the operating system's driver timing.
    """
    _positive_timeout(timeout)
    deadline = time.monotonic() + _positive_timeout(scan_timeout)
    if not baudrates or any(baud <= 0 for baud in baudrates):
        raise ValueError("baudrates must contain positive integers")
    names = ports if ports is not None else [p.device for p in list_ports.comports()]
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
                # An occupied/missing port cannot be fixed by changing its baudrate.
                result.errors[name] = str(exc)
                _logger.info("%s: %s; skipping this port.", name, exc)
                break
            try:
                protocol = None

                def observe(sample: Sample) -> None:
                    nonlocal protocol
                    if sample.metadata.get("protocol") == "hipnuc_binary":
                        protocol = "hipnuc_binary"

                device.sample_sink = observe
                passive_until = min(deadline, time.monotonic() + observation)
                while (remaining := passive_until - time.monotonic()) > 0:
                    device._pump(remaining)
                # The bounded observation above serves as discovery's startup
                # window; do not spend a second window at every baudrate.
                device._command_prepared = True
                info = None
                identity_error = None
                for attempt in range(2):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        identity_error = identity_error or (
                            "Identity query skipped: search deadline reached"
                        )
                        break
                    device.timeout = min(timeout, remaining)
                    if attempt:
                        _logger.info(
                            "%s at %s baud: valid binary data, identity timed out; "
                            "retrying once (up to %gs)...",
                            name,
                            baud,
                            device.timeout,
                        )
                    try:
                        info = device.read_info()
                        identity_error = None
                        break
                    except ResponseTimeout as exc:
                        identity_error = str(exc)
                        # Wrong-baud probe bytes can leave an unfinished device
                        # command line. Its first correct-baud query may be lost;
                        # retry only this read-only query on a binary candidate.
                        if protocol is None:
                            break
                    except DeviceError as exc:
                        identity_error = str(exc)
                        break
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
                break
            finally:
                device.close()
        else:
            if fallback is not None:
                result.devices.append(fallback)
            else:
                result.errors[name] = best_error
    return result
