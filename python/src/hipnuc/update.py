"""Synchronous application updates over serial kboot or CAN SDO.

Each update checks the image before touching the device and by default
reconnects afterwards to confirm that the new application runs. Use firmware
made for the exact product.
"""

from __future__ import annotations

from binascii import crc_hqx
from collections.abc import Callable
from dataclasses import dataclass
import errno
import logging
import math
import os
from pathlib import Path
import struct
import time
from typing import TYPE_CHECKING

import serial

from . import _postupdate
from ._connection import SERIAL_OPEN_ERRORS, open_error, reset_line_coding
from .can import _bus_hint
from ._firmware_image import FirmwareImage, load_image
from .errors import DeviceError, HipnucError, ResponseTimeout, TransportError, VerificationError
from .models import DeviceInfo

if TYPE_CHECKING:
    import can

_logger = logging.getLogger(__name__)
_Progress = Callable[[int, int], None]
_RESTART_TIMEOUT = 10.0  # The new application normally answers within a few seconds.
_POWER_CYCLE_WAIT = 60.0  # Time to reach the device's power after the prompt.


@dataclass(frozen=True)
class UpdateResult:
    """Versions are ``None`` when unknown: an image without version data, a
    device that did not answer before the update (for example one left in the
    bootloader), or verification turned off."""

    bytes_written: int
    image_version: str | None
    previous_version: str | None
    verified_version: str | None


def _version_key(version: str | None) -> tuple[int, ...] | None:
    try:
        return tuple(int(part) for part in version.split(".")) if version else None
    except ValueError:
        return None


def _check_wait(wait: float) -> None:
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not 0 <= wait < math.inf:
        raise ValueError("power_cycle_wait must be a finite number of seconds, 0 or more")


def _stopped(error: HipnucError) -> HipnucError:
    """The same failure, saying how to finish an update the erase already started."""
    message = f"{str(error).rstrip('.')}. The update stopped part-way; run the update again."
    if isinstance(error, DeviceError):
        return type(error)(message, code=error.code, response=error.response)
    return type(error)(message)


def _check_started(image: FirmwareImage, info: DeviceInfo) -> str | None:
    running = info.firmware_version
    if image.version and _version_key(running) != _version_key(image.version):
        raise VerificationError(
            f"The device runs version {running} after the update, not the image's {image.version}"
        )
    return running


def _frame(kind: int, payload: bytes) -> bytes:
    header = struct.pack("<BBH", 0x5A, kind, len(payload))
    return header + struct.pack("<H", crc_hqx(header + payload, 0)) + payload


class _Kboot:
    """One serial consumer; each reply shares one monotonic deadline."""

    def __init__(self, connection: serial.Serial):
        self.connection = connection

    def write(self, data: bytes) -> None:
        try:
            written = self.connection.write(data)
        except OSError as error:
            raise TransportError(f"Bootloader serial write failed: {error}") from error
        if written != len(data):
            raise TransportError("Bootloader serial write was incomplete")

    def purge(self) -> None:
        try:
            self.connection.reset_input_buffer()
        except OSError as error:
            raise TransportError(f"Bootloader serial input purge failed: {error}") from error

    def read(self, size: int, deadline: float) -> bytes:
        # Keep the 0.05 s timeout the port was opened with: setting it
        # reconfigures a Windows port and corrupts bytes already in flight.
        data = bytearray()
        while len(data) < size:
            try:
                block = self.connection.read(size - len(data))
            except OSError as error:
                raise TransportError(f"Bootloader serial read failed: {error}") from error
            if time.monotonic() >= deadline:
                raise ResponseTimeout("Bootloader response timed out")
            data.extend(block)
        return bytes(data)

    def ack(self, deadline: float) -> None:
        reply = self.read(2, deadline)
        if reply == b"\x5a\xa2":
            raise DeviceError("Bootloader rejected the packet (NAK)", code=0xA2)
        if reply == b"\x5a\xa3":
            raise DeviceError("Bootloader aborted the transfer (AckAbort)", code=0xA3)
        if reply != b"\x5a\xa1":
            raise VerificationError("Invalid bootloader acknowledgement")

    def response(self, tag: int, deadline: float) -> int:
        header = self.read(6, deadline)
        if header[:2] != b"\x5a\xa4":
            raise VerificationError("Invalid bootloader response frame")
        size, checksum = struct.unpack("<HH", header[2:])
        if not 4 <= size <= 512:
            raise VerificationError("Invalid bootloader response length")
        payload = self.read(size, deadline)
        if crc_hqx(header[:4] + payload, 0) != checksum:
            raise VerificationError("Bootloader response CRC mismatch")
        expected_tag = 0xA7 if tag == 7 else 0xA0
        if payload[0] != expected_tag or payload[3] < 2 or len(payload) != 4 + 4 * payload[3]:
            raise VerificationError("Invalid bootloader response tag or parameter count")
        status, value = struct.unpack_from("<II", payload, 4)
        if status:
            raise DeviceError(f"Bootloader reported status 0x{status:08X}", code=status)
        if tag != 7 and value != tag:
            raise VerificationError("Bootloader command echo mismatch")
        return value

    def command(self, tag: int, *parameters: int, timeout: float = 1.0) -> int:
        payload = bytes((tag, 0, 0, len(parameters)))
        payload += struct.pack(f"<{len(parameters)}I", *parameters)
        self.purge()
        self.write(_frame(0xA4, payload))
        deadline = time.monotonic() + timeout
        self.ack(deadline)
        return self.response(tag, deadline)

    def drain(self, quiet: float = 0.05, limit: float = 1.5) -> None:
        """Discard input until the line stays quiet; the bootloader sends nothing unasked.

        Purging alone is not enough: a USB adapter can still hold application
        output from before the reset and deliver it after the purge.
        """
        self.purge()
        end = time.monotonic() + limit
        last = time.monotonic()
        while (now := time.monotonic()) < end and now - last < quiet:
            try:
                if self.connection.read(256):
                    last = time.monotonic()
            except OSError as error:
                raise TransportError(f"Bootloader serial read failed: {error}") from error
        try:
            # Otherwise every reply can arrive without its last byte.
            reset_line_coding(self.connection)
        except (OSError, ValueError) as error:
            raise TransportError(f"Bootloader serial setup failed: {error}") from error

    def connect(self, attempts: int = 10) -> None:
        # Only the non-destructive ping is retried. Never retry erase/data.
        for _ in range(attempts):
            self.drain()
            self.write(b"\x5a\xa6")
            # A USB adapter can deliver application bytes ahead of the reply, in
            # batches (CH347 on Linux: about every 60 ms), so search a window.
            received = b""
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                try:
                    received += self.connection.read(self.connection.in_waiting or 1)
                except OSError as error:
                    raise TransportError(f"Bootloader serial read failed: {error}") from error
                if _ping_reply(received):
                    return
                received = received[-9:]
            time.sleep(0.1)
        raise ResponseTimeout("Bootloader did not answer the ping")


def _running_application(port: str, baudrate: int | None):
    """Return identity and baudrate, or Nones if no application answers."""
    from .serial_device import SerialDevice

    try:
        # Enough for every rate down to 9600 when detecting.
        with SerialDevice(port, baudrate, timeout=1.0, scan_timeout=15.0) as device:
            return device.info or device.read_info(), device.baudrate
    except HipnucError as error:
        _logger.info("No application answered on %s (%s); trying the bootloader", port, error)
        return None, None


def _wait_for_application(port: str, baudrate: int) -> tuple[DeviceInfo, int]:
    from .serial_device import SerialDevice

    deadline = time.monotonic() + _RESTART_TIMEOUT
    while time.monotonic() < deadline:
        try:
            with SerialDevice(port, baudrate, timeout=1.0) as device:
                return device.read_info(), baudrate
        except HipnucError as error:
            _logger.debug("Waiting for the application: %s", error)
            time.sleep(0.3)
    # A restart may also return at another baudrate.
    try:
        with SerialDevice(port, None, timeout=1.0, scan_timeout=10.0) as device:
            return device.info or device.read_info(), device.baudrate
    except HipnucError as error:
        raise ResponseTimeout(
            f"The device did not answer on {port} after the update. Run the update again "
            "with an image made for this product, and power-cycle the device when asked."
        ) from error


def _open_serial(port: str, baudrate: int) -> serial.Serial:
    options = {"exclusive": True} if os.name == "posix" else {}
    try:
        return serial.Serial(port, baudrate, timeout=0.05, write_timeout=2, **options)
    except SERIAL_OPEN_ERRORS as error:
        raise open_error(port, error) from error


def _close_quietly(connection: serial.Serial) -> None:
    try:
        connection.close()
    except OSError:
        pass  # Preserve the original failure or cancellation.


def _waiting_bootloader(
    port: str, baudrate: int, attempts: int = 10
) -> tuple[serial.Serial, _Kboot] | None:
    """Connect to a bootloader already waiting at ``baudrate``, without any command."""
    connection = _open_serial(port, baudrate)
    try:
        boot = _Kboot(connection)
        boot.connect(attempts)
        _logger.info("Found a waiting bootloader on %s at %d baud", port, baudrate)
        return connection, boot
    except (ResponseTimeout, VerificationError):
        _close_quietly(connection)
        return None
    except BaseException:
        _close_quietly(connection)
        raise


def _ping_reply(data: bytes) -> bool:
    """Whether ``data`` holds a complete ping reply with a valid CRC."""
    start = data.find(b"\x5a\xa7")
    while 0 <= start <= len(data) - 10:
        if crc_hqx(data[start : start + 8], 0) == int.from_bytes(
            data[start + 8 : start + 10], "little"
        ):
            return True
        start = data.find(b"\x5a\xa7", start + 1)
    return False


def _catch_bootloader(port: str, wait: float) -> tuple[serial.Serial, _Kboot]:
    """Ping at 115200 through a power cycle for up to ``wait`` seconds.

    After power-up the bootloader listens for about 100 ms before it starts a
    valid application, so this is the only way in when that application hangs
    (for example an image for another MCU) or the bootloader stopped answering.
    """
    if not wait:
        raise ResponseTimeout(f"Neither the application nor the bootloader answered on {port}")
    _logger.warning(
        "Power-cycle the device on %s now: unplug its connector, power and serial lines "
        "together, and plug it back; a USB serial adapter stays in the computer. "
        "Waiting up to %.0f s for its bootloader.",
        port,
        wait,
    )
    connection = _open_serial(port, 115200)
    try:
        boot = _Kboot(connection)
        deadline = time.monotonic() + wait
        received = b""
        while time.monotonic() < deadline:
            boot.write(b"\x5a\xa6")
            time.sleep(0.02)  # Several pings land in the window.
            try:
                received = (received + connection.read(connection.in_waiting))[-64:]
            except OSError as error:
                raise TransportError(f"Bootloader serial read failed: {error}") from error
            if _ping_reply(received):  # Application output at 115200 can hold 5A A7 too.
                # A cold bootloader still starts the application about 1 s after
                # a ping that no command follows, and draining late adapter bytes
                # can take longer: hold it with a command, then resynchronize.
                boot.write(_frame(0xA4, bytes((7, 0, 0, 1)) + struct.pack("<I", 0x0B)))
                boot.connect(3)
                _logger.info("Found the bootloader on %s at 115200 baud", port)
                return connection, boot
        raise ResponseTimeout(
            f"No bootloader answered on {port}. Check the port, then run the update again "
            "and power-cycle the device itself when asked."
        )
    except BaseException:
        _close_quietly(connection)
        raise


def _enter_bootloader(port: str, baudrate: int, wait: float) -> tuple[serial.Serial, _Kboot]:
    connection = _open_serial(port, baudrate)
    try:
        boot = _Kboot(connection)
        _logger.info("Entering the serial bootloader on %s at %d baud", port, baudrate)
        # Terminate whatever the CLI line buffer holds first: bytes of an earlier
        # Modbus request stay there, and a command appended to them is ignored.
        boot.write(b"\r\nREBOOT BL\r\n")
        time.sleep(0.05)
        try:
            boot.connect()
            return connection, boot
        except (ResponseTimeout, VerificationError):
            pass
    except BaseException:
        _close_quietly(connection)
        raise
    _close_quietly(connection)
    # A device left in the bootloader by an interrupted update answers at 115200 only.
    waiting = _waiting_bootloader(port, 115200) if baudrate != 115200 else None
    return waiting or _catch_bootloader(port, wait)


def _program(boot: _Kboot, image: FirmwareImage, progress: _Progress | None) -> None:
    time.sleep(0.3)
    packet_size = boot.command(7, 0x0B)
    flash_size = boot.command(7, 0x04)
    if packet_size < 4 or not flash_size:
        raise VerificationError("Bootloader reported invalid packet or flash size")
    if len(image.data) > flash_size:
        boot.command(0x0B)  # Return to the application unchanged.
        raise ValueError(f"Image exceeds the device's reported flash size {flash_size}")
    if progress:
        progress(0, len(image.data))
    _logger.info("Erasing application flash")
    try:
        boot.command(2, image.address, len(image.data), timeout=10)
    except DeviceError as error:
        # Only a status in the response means a refused range; a NAK (A2),
        # an abort (A3) or a malformed reply is a transfer failure.
        if isinstance(error, VerificationError) or error.code in (None, 0xA2, 0xA3):
            raise _stopped(error) from error
        # A refused range left the flash untouched; running again cannot help.
        try:
            boot.command(0x0B)  # Return to the application unchanged.
        except HipnucError:
            pass
        raise DeviceError(
            f"{error}: the bootloader refused the image's address range; "
            "use the image made for this product",
            code=error.code,
        ) from error
    except HipnucError as error:
        raise _stopped(error) from error
    try:
        boot.command(4, image.address, len(image.data))
        # HC32 writes need word-aligned starts; AT32 otherwise drops an odd tail.
        chunk_size = min(packet_size, 512) & ~3
        for offset in range(0, len(image.data), chunk_size):
            chunk = image.data[offset : offset + chunk_size]
            boot.write(_frame(0xA5, chunk))
            deadline = time.monotonic() + 2
            boot.ack(deadline)
            written = offset + len(chunk)
            if written == len(image.data):
                boot.response(4, deadline)
            if progress:
                progress(written, len(image.data))
        _logger.info("Image transfer acknowledged; restarting the device")
        boot.command(0x0B)
    except HipnucError as error:
        raise _stopped(error) from error


def update_serial(
    image_path: str | Path,
    *,
    port: str,
    baudrate: int | None = None,
    verify: bool = True,
    progress: _Progress | None = None,
    power_cycle_wait: float = _POWER_CYCLE_WAIT,
) -> UpdateResult:
    """Update the device on one explicit serial port from its Intel HEX image.

    ``baudrate`` is the application's current rate; ``None`` detects it. A
    device already in the bootloader is also reached at 115200. The image is
    checked before the port opens. Keep the device still until the call returns.

    When nothing answers, a WARNING on the ``hipnuc.update`` logger asks for a
    power cycle, and the call blocks up to ``power_cycle_wait`` seconds to
    catch the bootloader as the device starts; 0 fails at once instead.

    ``progress(written, total)`` receives acknowledged byte counts; its
    exceptions, including KeyboardInterrupt, cancel without a reset. With
    ``verify`` the device must come back running the image's version.

    An unreadable image file raises OSError; invalid images and arguments
    ValueError; transport failures TransportError, missing
    replies ResponseTimeout, rejected requests DeviceError and wrong replies
    VerificationError.
    """
    if not port:
        raise ValueError("Specify a serial port")
    _check_wait(power_cycle_wait)
    if baudrate is not None and (
        not isinstance(baudrate, int)
        or isinstance(baudrate, bool)
        or not 9600 <= baudrate <= 2000000
    ):
        raise ValueError("Serial firmware update requires an integer baudrate in 9600..2000000")
    image = load_image(image_path)
    if image.address % 4 or len(image.data) % 4:
        raise ValueError("Serial firmware start and size must be four-byte aligned")
    # Probe one rate before any other: bytes at a wrong rate can stall a
    # bootloader that already waits at this one.
    first = baudrate or 115200
    info = found = None
    # A bootloader started by power-up after an interrupted update listens at
    # 115200 only; a short ping there comes first, and is harmless to an app.
    # A waiting bootloader answers at once, so these probes try only twice.
    waiting = _waiting_bootloader(port, 115200, attempts=2) if first != 115200 else None
    if not waiting:
        info, found = _running_application(port, first)
        waiting = None if info else _waiting_bootloader(port, first, attempts=2)
        if not info and not waiting:
            info, found = _running_application(port, None)
        if not info and not waiting:
            waiting = _catch_bootloader(port, power_cycle_wait)
    previous = info.firmware_version if info else None
    connection, boot = waiting or _enter_bootloader(port, found, power_cycle_wait)
    try:
        _program(boot, image, progress)
    except BaseException:
        _close_quietly(connection)
        raise
    try:
        connection.close()
    except OSError as error:
        raise TransportError(f"Serial firmware update close failed: {error}") from error
    if not verify:
        return UpdateResult(len(image.data), image.version, previous, None)
    restarted, rate = _wait_for_application(port, found or first)
    verified = _check_started(image, restarted)
    _postupdate.finish(port, rate)
    return UpdateResult(len(image.data), image.version, previous, verified)


def _can_send(bus: can.BusABC, message: can.Message, timeout: float) -> None:
    import can

    try:
        bus.send(message, timeout=timeout)
    except can.CanError as error:
        if getattr(error, "error_code", None) != errno.ENOBUFS:
            raise
        # The kernel refused the frame, so nothing reached the bus; try once more.
        time.sleep(0.05)
        bus.send(message, timeout=timeout)


def _can_exchange(bus: can.BusABC, node_id: int, request: bytes, timeout: float) -> bytes:
    import can

    error_class = 0
    try:
        deadline = time.monotonic() + timeout
        _can_send(
            bus,
            can.Message(arbitration_id=0x600 + node_id, is_extended_id=False, data=request),
            timeout,
        )
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                hint = _bus_hint(error_class)
                raise ResponseTimeout(
                    f"CAN node {node_id} did not reply" + (f": {hint}" if hint else "")
                )
            reply = bus.recv(min(remaining, 0.05))
            if reply is None or time.monotonic() >= deadline:
                continue
            if reply.is_error_frame:
                error_class |= reply.arbitration_id
                continue
            if reply.arbitration_id != 0x580 + node_id or reply.is_extended_id:
                continue
            if reply.is_remote_frame or reply.is_fd or reply.dlc != 8 or len(reply.data) != 8:
                raise VerificationError("Invalid CAN bootloader reply frame")
            payload = bytes(reply.data)
            # Delayed/repeated handshake ACKs must not derail an erase already
            # requested. The deadline still bounds a stream of stale replies.
            if payload[:3] == b"\x60\x51\x1f" and payload[3] in (5, 6):
                if request[:4] != b"\x23" + payload[1:4]:
                    continue
            if payload[0] == 0x80:
                expected = request[1:4] if request[0] in (0x21, 0x23) else b"\x51\x1f\x01"
                if payload[1:4] != expected:
                    raise VerificationError("Unmatched CAN bootloader abort")
                code = int.from_bytes(payload[4:], "little")
                raise DeviceError(f"CAN bootloader aborted: 0x{code:08X}", code=code)
            return payload
    except (can.CanError, OSError) as error:
        code = getattr(error, "error_code", None) or getattr(error, "errno", None)
        hint = _bus_hint(error_class, code)
        raise TransportError(
            f"CAN firmware update failed: {error}" + (f"; {hint}" if hint else "")
        ) from error


def _drain_can(bus: can.BusABC) -> None:
    """Do not accept an acknowledgement queued before this update started."""
    import can

    deadline = time.monotonic() + 0.1
    try:
        while bus.recv(0) is not None:
            if time.monotonic() >= deadline:
                raise ResponseTimeout("CAN receive queue did not drain; use a newly opened bus")
    except (can.CanError, OSError) as error:
        raise TransportError(f"CAN firmware update failed: {error}") from error


def _sdo_write(bus: can.BusABC, node_id: int, subindex: int, timeout: float = 0.1) -> None:
    request = bytes((0x23, 0x51, 0x1F, subindex, 0, 0, 0, 0))
    reply = _can_exchange(bus, node_id, request, timeout)
    if reply[:4] != b"\x60" + request[1:4]:
        raise VerificationError("CAN bootloader index or subindex echo mismatch")


def _can_confirm(bus: can.BusABC, node_id: int) -> bool:
    """Whether the bootloader acknowledges :06 within ten short tries.

    A full transmit queue counts as no answer: a node that is off acknowledges
    nothing. A link that is down has already failed the identity read.
    """
    for _ in range(10):
        try:
            _sdo_write(bus, node_id, 6)
            return True
        except (TransportError, VerificationError):  # See _catch_can_bootloader.
            time.sleep(0.05)
    return False


def _catch_can_bootloader(bus: can.BusABC, node_id: int, wait: float) -> None:
    """Repeat :06 through a power cycle, as ``_catch_bootloader`` does on serial.

    While the device is off nothing acknowledges the frames, so a full transmit
    queue is expected; the queued requests reach the bootloader once it starts.
    """
    if not wait:
        raise ResponseTimeout(f"CAN node {node_id} answered neither as application nor bootloader")
    _logger.warning(
        "Power-cycle the device at CAN node %d now; waiting up to %.0f s for its bootloader.",
        node_id,
        wait,
    )
    deadline = time.monotonic() + wait
    last = None
    while time.monotonic() < deadline:
        try:
            _sdo_write(bus, node_id, 6, timeout=0.05)
            _logger.info("Found the CAN bootloader at node %d", node_id)
            return
        except ResponseTimeout:
            pass
        except VerificationError:
            pass  # The 1.1.0 bootloader answers its first frame after power-up with command 0.
        except TransportError as error:
            last = error
            time.sleep(0.05)
    raise ResponseTimeout(
        f"CAN node {node_id} did not answer"
        + (f" ({last})" if last else "")
        + ". Check the bus at 500 kbit/s, then run the update again and power-cycle "
        "the device when asked."
    )


def update_can(
    bus: can.BusABC,
    node_id: int,
    image_path: str | Path,
    *,
    verify: bool = True,
    progress: _Progress | None = None,
    power_cycle_wait: float = _POWER_CYCLE_WAIT,
) -> UpdateResult:
    """Update one CAN node (1..127); the caller exclusively owns and closes ``bus``.

    The bus must run at the node's current bitrate; Classic frames are used even
    on CAN FD products. A node left in the bootloader by an interrupted update
    answers at 500 kbit/s as node 8. When node 8 does not answer at all, the
    power-cycle request and ``power_cycle_wait`` work as in ``update_serial``.
    The bootloader writes at its own application address, so use an image made
    for this exact product. No erase or data frame is ever resent. Version,
    progress and exception contracts match ``update_serial``.
    """
    if not isinstance(node_id, int) or isinstance(node_id, bool) or not 1 <= node_id <= 127:
        raise ValueError("CAN firmware-update node_id must be 1..127")
    _check_wait(power_cycle_wait)
    from .can import read_info, wait_for_node

    image = load_image(image_path)
    try:
        previous = read_info(bus, node_id, timeout=1.0).firmware_version
    except (ResponseTimeout, DeviceError, ValueError):
        previous = None  # Possibly already in the bootloader.
    except TransportError as error:
        # Frames left unacknowledged, also by an earlier run, fill the transmit
        # queue while the node is off; that is no answer, not a broken link.
        if getattr(error.__cause__, "error_code", None) != errno.ENOBUFS:
            raise
        previous = None
    _drain_can(bus)
    _logger.info("Entering the CAN bootloader at node %d", node_id)
    # Both application and bootloader acknowledge :05. Even if its ACK is lost,
    # try :06; resending :05 could reboot an application that has already moved.
    try:
        _sdo_write(bus, node_id, 5)
    except (TransportError, VerificationError):
        pass  # Includes no reply; the confirm below decides.
    time.sleep(0.02)
    if not _can_confirm(bus, node_id):
        if node_id != 8:
            raise ResponseTimeout(
                f"CAN node {node_id} did not answer. After power-up the bootloader answers "
                "at 500 kbit/s as node 8; run the update again with those settings."
            )
        _catch_can_bootloader(bus, node_id, power_cycle_wait)
    if progress:
        progress(0, len(image.data))
    request = b"\x21\x51\x1f\x01" + struct.pack("<I", len(image.data))
    _logger.info("Erasing CAN application flash")
    try:
        reply = _can_exchange(bus, node_id, request, 8)
        if reply[:4] != b"\x60\x51\x1f\x01":
            raise VerificationError("CAN bootloader download-initiate echo mismatch")
        toggle = 0
        for offset in range(0, len(image.data), 7):
            chunk = image.data[offset : offset + 7]
            written = offset + len(chunk)
            command = toggle
            if written == len(image.data):
                # HiPNUC's final-byte convention differs from CiA 301: seven -> 03.
                command |= 17 - 2 * len(chunk)
            request = bytes((command,)) + chunk.ljust(7, b"\x00")
            reply = _can_exchange(bus, node_id, request, 4)
            if reply[0] != (0x20 | toggle) or reply[1:] != request[1:]:
                raise VerificationError("CAN bootloader segment toggle or data echo mismatch")
            toggle ^= 0x10
            if progress:
                progress(written, len(image.data))
    except HipnucError as error:
        raise _stopped(error) from error
    _logger.info("Image transfer acknowledged; starting the application")
    # The bootloader jumps before it can reply; it replies only when it refuses.
    try:
        _sdo_write(bus, node_id, 9, timeout=0.2)
    except ResponseTimeout:
        pass
    else:
        raise DeviceError(
            "The bootloader rejected the image and stays in the bootloader; "
            "check that the image is made for this product"
        )
    if not verify:
        return UpdateResult(len(image.data), image.version, previous, None)
    try:
        restarted = wait_for_node(bus, node_id, recovery_timeout=_RESTART_TIMEOUT, timeout=1.0)
    except ResponseTimeout as error:
        raise ResponseTimeout(
            f"CAN node {node_id} did not answer after the update. It may still be in the "
            "bootloader or have restarted with default CAN settings; run the update again "
            "at 500 kbit/s with node 8 and an image made for this product, and "
            "power-cycle the device when asked."
        ) from error
    return UpdateResult(len(image.data), image.version, previous, _check_started(image, restarted))
