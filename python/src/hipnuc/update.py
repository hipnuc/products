"""Explicit, synchronous application updates over serial kboot or CAN SDO.

Use firmware for the exact product. Protocol acknowledgements do not prove that
the new application starts or that its model matches the connected hardware.
"""

from __future__ import annotations

from binascii import crc_hqx
from collections.abc import Callable
from dataclasses import dataclass
import logging
import os
from pathlib import Path
import struct
import time
from typing import TYPE_CHECKING

import serial

from ._connection import SERIAL_OPEN_ERRORS, open_error
from ._firmware_image import load_image
from .errors import DeviceError, ResponseTimeout, TransportError, VerificationError

if TYPE_CHECKING:
    import can

_logger = logging.getLogger(__name__)
_Progress = Callable[[int, int], None]


@dataclass(frozen=True)
class UpdateResult:
    """Transfer/start acknowledgements; application verification is a separate step."""

    bytes_written: int
    transfer_acknowledged: bool
    start_requested: bool
    start_acknowledged: bool
    application_verified: bool = False


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
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ResponseTimeout("Bootloader response timed out")
            try:
                self.connection.timeout = min(remaining, 0.05)
                block = self.connection.read(size - len(data))
            except OSError as error:
                raise TransportError(f"Bootloader serial read failed: {error}") from error
            if time.monotonic() >= deadline:
                raise ResponseTimeout("Bootloader response timed out")
            if len(block) > size - len(data):
                raise VerificationError("Serial backend returned more bytes than requested")
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

    def connect(self) -> None:
        # Only the non-destructive ping is retried. Never retry erase/data.
        for attempt in range(10):
            self.purge()
            self.write(b"\x5a\xa6")
            try:
                reply = self.read(10, time.monotonic() + 0.1)
                if reply[:2] != b"\x5a\xa7" or crc_hqx(reply[:8], 0) != int.from_bytes(
                    reply[8:], "little"
                ):
                    raise VerificationError("Bootloader ping CRC or header mismatch")
                return
            except (ResponseTimeout, VerificationError):
                if attempt == 9:
                    raise
                time.sleep(0.1)


def update_serial(
    image_path: str | Path,
    *,
    port: str,
    baudrate: int,
    progress: _Progress | None = None,
) -> UpdateResult:
    """Update one explicit serial port from Intel HEX and close it on every exit.

    ``progress(written, total)`` receives acknowledged byte counts. Its exceptions,
    including KeyboardInterrupt, cancel without sending an automatic reset.
    Open/transport failures raise TransportError, missing replies ResponseTimeout,
    rejected requests DeviceError, malformed replies VerificationError. Invalid
    images raise ValueError; file errors propagate as OSError before connection.
    The image start and size must be four-byte aligned for flash programming.
    """
    if not port or not isinstance(baudrate, int) or isinstance(baudrate, bool) or baudrate <= 0:
        raise ValueError("Specify a port and a positive integer baudrate")
    image = load_image(image_path)
    if image.address % 4 or len(image.data) % 4:
        raise ValueError("Serial firmware start and size must be four-byte aligned")
    options = {"exclusive": True} if os.name == "posix" else {}
    try:
        connection = serial.Serial(port, baudrate, timeout=0.05, write_timeout=2, **options)
    except SERIAL_OPEN_ERRORS as error:
        raise open_error(port, error) from error
    try:
        boot = _Kboot(connection)
        _logger.info("Entering serial bootloader on %s at %d baud", port, baudrate)
        boot.write(b"REBOOT BL\r\n")
        time.sleep(0.05)
        boot.connect()
        time.sleep(0.3)
        packet_size = boot.command(7, 0x0B)
        flash_size = boot.command(7, 0x04)
        if packet_size < 4 or not flash_size:
            raise VerificationError("Bootloader reported invalid packet or flash size")
        if len(image.data) > flash_size:
            raise ValueError(f"Image exceeds the device's reported flash size {flash_size}")
        if progress:
            progress(0, len(image.data))
        _logger.info("Erasing application flash")
        boot.command(2, image.address, len(image.data), timeout=10)
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
        _logger.info("Image transfer acknowledged; requesting application start")
        boot.command(0x0B)
    except BaseException:
        try:
            connection.close()
        except OSError:
            pass  # Preserve the original transfer failure or cancellation.
        raise
    else:
        try:
            connection.close()
        except OSError as error:
            raise TransportError(f"Serial firmware update close failed: {error}") from error
    return UpdateResult(len(image.data), True, True, True)


def _can_exchange(bus: can.BusABC, node_id: int, request: bytes, timeout: float) -> bytes:
    import can

    try:
        deadline = time.monotonic() + timeout
        sent_time = time.time()
        bus.send(
            can.Message(arbitration_id=0x600 + node_id, is_extended_id=False, data=request),
            timeout=timeout,
        )
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ResponseTimeout(f"CAN bootloader {node_id} response timed out")
            reply = bus.recv(min(remaining, 0.05))
            if time.monotonic() >= deadline:
                raise ResponseTimeout(f"CAN bootloader {node_id} response timed out")
            if reply is None or reply.arbitration_id != 0x580 + node_id or reply.is_extended_id:
                continue
            if 0 < reply.timestamp < sent_time:
                continue
            if (
                reply.is_error_frame
                or reply.is_remote_frame
                or reply.is_fd
                or reply.dlc != 8
                or len(reply.data) != 8
            ):
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
        raise TransportError(f"CAN firmware update failed: {error}") from error


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


def _sdo_write(bus: can.BusABC, node_id: int, subindex: int) -> None:
    request = bytes((0x23, 0x51, 0x1F, subindex, 0, 0, 0, 0))
    reply = _can_exchange(bus, node_id, request, 0.1)
    if reply[:4] != b"\x60" + request[1:4]:
        raise VerificationError("CAN bootloader index or subindex echo mismatch")


def update_can(
    bus: can.BusABC,
    node_id: int,
    image_path: str | Path,
    *,
    raw_binary: bool = False,
    progress: _Progress | None = None,
) -> UpdateResult:
    """Update one CAN node (1..127); the caller exclusively owns and closes bus.

    CAN writes sequentially at the bootloader's fixed application address; HEX
    addresses are not transmitted. Use an image made for this exact product.
    Only application-start reply timeout is returned as start_acknowledged=False.
    Erase, data and explicit rejections always raise; no destructive request is
    retried. progress and exception contracts match update_serial.
    """
    if not isinstance(node_id, int) or isinstance(node_id, bool) or not 1 <= node_id <= 127:
        raise ValueError("CAN firmware-update node_id must be 1..127")
    image = load_image(image_path, raw_binary=raw_binary)
    _drain_can(bus)
    _logger.info("Entering CAN bootloader at node %d", node_id)
    # Both application and bootloader acknowledge :05. Even if its ACK is lost,
    # try :06; resending :05 could reboot an application that has already moved.
    try:
        _sdo_write(bus, node_id, 5)
    except ResponseTimeout:
        pass
    time.sleep(0.02)
    for attempt in range(5):
        try:
            _sdo_write(bus, node_id, 6)
            break
        except ResponseTimeout:
            if attempt == 4:
                raise
            time.sleep(0.05)
    if progress:
        progress(0, len(image.data))
    request = b"\x21\x51\x1f\x01" + struct.pack("<I", len(image.data))
    _logger.info("Erasing CAN application flash")
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
    _logger.info("Image transfer acknowledged; requesting application start")
    acknowledged = True
    try:
        _sdo_write(bus, node_id, 9)
    except ResponseTimeout:
        # The product bootloader jumps before it sends the SDO acknowledgement.
        acknowledged = False
    return UpdateResult(len(image.data), True, True, acknowledged)
