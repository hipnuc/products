"""HiPNUC J1939 functions for an ordinary ``python-can`` Bus and Message.

Install the optional ``can`` extra to send requests. Decoding does not open a
connection. A register transaction owns reception until it returns: do not
run another reader, updater, or transaction on the same Bus concurrently.

Configuration writes persist immediately in the device; ``save_config`` is
kept for older firmware. Node address and bitrates take effect after a
reboot. The device resets before it can echo a reboot request, so ``reboot``
waits for the identity block instead of an acknowledgement.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import struct
import time
from typing import TYPE_CHECKING, Any

from .decoder import GRAVITY, INS_STATUS_NAMES, _make_sample, _utc, status_flags
from .errors import DeviceError, ResponseTimeout, TransportError, VerificationError
from .models import DeviceInfo, Sample, WriteResult, identity_from_registers

if TYPE_CHECKING:
    import can


_logger = logging.getLogger(__name__)
_HOST_ADDRESS = 0x55  # Product firmware always addresses configuration replies here.
_CONFIG_ID = 0x0CEF0000
_PGNS = {
    0xFF10: ("POSITION", 8),
    0xFF14: ("ALTITUDE", 8),
    0xFF18: ("GNSS_STATUS", 5),
    0xFF26: ("VELOCITY", 6),
    0xFF2F: ("TIME", 8),
    0xFF34: ("ACC", 6),
    0xFF37: ("GYR", 6),
    0xFF3A: ("MAG", 6),
    0xFF3D: ("ROLL_PITCH", 8),
    0xFF41: ("YAW", 4),
    0xFF43: ("TEMP", 2),
    0xFF46: ("QUAT", 8),
    0xFF4A: ("INCLINATION", 8),
    0xFF5B: ("CANFD83", 8),
}
PGN_NAMES = {name: pgn for pgn, (name, _) in _PGNS.items()}
_FD_SIZES = {0: 12, 1: 12, 2: 12, 3: 12, 4: 16, 5: 8, 6: 8, 8: 4}

BROADCAST_ADDRESS = 0xFF
PERIOD_SYNC_IN = 0x8000  # Period value: send the PGN once per SYNC_IN edge.
CANFD83_MAP_SUPPORTED = 0x17F
CANFD83_MAP_DEFAULT = 0x12B
# Register codes for the nominal and CAN FD data bitrates (bit/s -> code).
BITRATES = {
    1_000_000: 0,
    800_000: 1,
    500_000: 2,
    250_000: 3,
    125_000: 4,
    100_000: 5,
    50_000: 6,
    20_000: 7,
    10_000: 8,
}
DATA_BITRATES = {1_000_000: 0, 2_000_000: 10, 4_000_000: 11, 5_000_000: 12, 8_000_000: 13}

_REG_CONTROL = 0x0000
_REG_TRIGGER = 0x0096
_REG_NOMINAL_BITRATE = 0x009A
_REG_NODE_ID = 0x009C
_REG_OUTPUT_ENABLE = 0x009D
_CONTROL_SAVE = 0x00
_CONTROL_FACTORY_RESET = 0x01
_CONTROL_REBOOT = 0xFF
_IDENTITY_SLOTS = (*range(0x70, 0x7C), *range(0x7F, 0x83))  # 0x7C-0x7E are unmapped.
_FD_HINT = " CANFD83 registers exist only on CAN FD products."


def _pgn(identifier: int) -> int:
    pgn = (identifier >> 8) & 0x3FFFF
    if ((pgn >> 8) & 0xFF) < 0xF0:
        pgn &= 0x3FF00
    return pgn


def decode_message(message: can.Message) -> Sample | None:
    """Decode one frame; return ``None`` for unrelated IDs or standard frames.

    A recognized PGN with invalid flags, length or bitmap raises ``ValueError``.
    Measurements describe this frame alone, in SI units. ``raw`` contains the
    data bytes; ID, PGN, channel, FD flags and the backend's original timestamp
    are in ``metadata``. Its time base depends on the backend, so this decoder
    leaves ``received_time_ns`` unset; the caller may supply host reception time.
    """
    if not message.is_extended_id:
        return None
    identifier = message.arbitration_id
    pgn = _pgn(identifier)
    if pgn not in _PGNS:
        return None
    name, minimum = _PGNS[pgn]
    raw = bytes(message.data)
    if (
        not 0 <= identifier <= 0x1FFFFFFF
        or message.is_remote_frame
        or message.is_error_frame
        or message.dlc != len(raw)
        or len(raw) < minimum
        or len(raw) > (64 if pgn == 0xFF5B else 8)
    ):
        raise ValueError(f"Invalid J1939 {name} frame flags or length")
    metadata: dict[str, Any] = {
        "protocol": "canfd83" if pgn == 0xFF5B else "j1939",
        "arbitration_id": identifier,
        "pgn": pgn,
        "is_fd": message.is_fd,
        "bitrate_switch": message.bitrate_switch,
        "channel": message.channel,
        "can_timestamp_s": message.timestamp if math.isfinite(message.timestamp) else None,
        "can_timestamp_reference": "backend_defined",
        "body_frame": "device_configured_axes",
        "navigation_frame": "device_configured",
    }
    values: dict[str, Any] = {"node_id": identifier & 0xFF}
    issues: list[str] = []
    if pgn == 0xFF5B:
        if not message.is_fd or len(raw) not in (12, 16, 20, 24, 32, 48, 64):
            raise ValueError("CANFD83 requires a CAN FD frame with a legal data length")
        _decode_fd(raw, values, metadata, issues)
    else:
        _decode_classic(pgn, raw, values, metadata, issues)
    if "euler_rad" in values:
        metadata.update(
            euler_components=["roll", "pitch", "yaw"], euler_convention="device_configured"
        )
    kind = name if name == "CANFD83" else f"J1939_{name}"
    return _make_sample(kind, values, raw, metadata, issues)


def _decode_classic(
    pgn: int, raw: bytes, values: dict[str, Any], metadata: dict[str, Any], issues: list[str]
) -> None:
    if pgn in (0xFF34, 0xFF37, 0xFF3A):
        name, scale = {
            0xFF34: ("acceleration_m_s2", GRAVITY / 2048),
            0xFF37: ("angular_velocity_rad_s", math.radians(2000) / 32768),
            0xFF3A: ("magnetic_field_t", 0.001 / 32768),
        }[pgn]
        values[name] = [n * scale for n in struct.unpack_from("<3h", raw)]
    elif pgn == 0xFF3D:
        roll, pitch = [math.radians(n / 1000) for n in struct.unpack("<2i", raw)]
        values.update(roll_rad=roll, pitch_rad=pitch, euler_rad=[roll, pitch, None])
    elif pgn == 0xFF41:
        # Heading is clockwise; the separate Euler yaw is counter-clockwise.
        values["heading_rad"] = math.radians(struct.unpack_from("<i", raw)[0] / 1000)
        if len(raw) >= 8:
            yaw = math.radians(struct.unpack_from("<i", raw, 4)[0] / 1000)
            values.update(yaw_rad=yaw, euler_rad=[None, None, yaw])
    elif pgn == 0xFF43:
        values["temperature_c"] = struct.unpack_from("<h", raw)[0] / 100
        # The remaining bytes include a reserved 999 placeholder, not pressure.
    elif pgn == 0xFF46:
        values["quaternion_wxyz"] = [n / 10000 for n in struct.unpack("<4h", raw)]
        metadata.update(quaternion_order="wxyz", quaternion_direction="body_to_navigation")
    elif pgn == 0xFF4A:
        values["inclination_rad"] = [math.radians(n / 1000) for n in struct.unpack("<2i", raw)]
    elif pgn == 0xFF2F:
        year, month, day, hour, minute, second, ms = struct.unpack("<6BH", raw)
        metadata["utc_raw_components"] = [year, month, day, hour, minute, second, ms]
        if second > 59 or ms > 999:
            values["utc"] = None
            issues.append("invalid_utc")
        else:
            parts = (year, month, day, hour, minute, second * 1000 + ms)
            values["utc"] = _utc(parts, issues)
    elif pgn == 0xFF10:
        latitude, longitude = struct.unpack("<2i", raw)
        values.update(latitude_deg=latitude / 1e7, longitude_deg=longitude / 1e7)
        metadata.update(position_source="ins", position_datum="WGS84")
    elif pgn == 0xFF14:
        altitude, geoid, age = struct.unpack("<ihh", raw)
        values.update(
            altitude_msl_m=altitude / 100,
            geoid_separation_m=geoid / 100,
            differential_age_s=age / 100,
        )
        metadata.update(position_source="ins", altitude_reference="mean_sea_level")
    elif pgn == 0xFF18:
        values.update(
            position_quality=raw[0],
            heading_quality=raw[1],
            position_satellites=raw[2],
            heading_satellites=raw[3],
            ins_status=raw[4],
            ins_status_name=INS_STATUS_NAMES.get(raw[4], "unknown"),
        )
        metadata["quality_source"] = "raw_gnss"
    elif pgn == 0xFF26:
        values["velocity_enu_m_s"] = [n / 100 for n in struct.unpack_from("<3h", raw)]
        metadata["velocity_source"] = "ins"
        if len(raw) >= 8:
            values["ground_speed_m_s"] = struct.unpack_from("<h", raw, 6)[0] / 100


def _decode_fd(
    raw: bytes, values: dict[str, Any], metadata: dict[str, Any], issues: list[str]
) -> None:
    bitmap, status, ins_status, sequence = struct.unpack_from("<IHBB", raw)
    logical_length = canfd83_length(bitmap)
    if not logical_length:
        raise ValueError(f"Unsupported CANFD83 bitmap: 0x{bitmap:08X}")
    if logical_length > len(raw):
        raise ValueError("CANFD83 bitmap exceeds the payload length")
    values.update(
        main_status=status,
        status_flags=status_flags(status),
        ins_status=ins_status,
        ins_status_name=INS_STATUS_NAMES.get(ins_status, "unknown"),
        data_bitmap=bitmap,
    )
    metadata.update(sequence=sequence, logical_length=logical_length)
    cursor = 8
    for bit, size in _FD_SIZES.items():
        if not bitmap & (1 << bit):
            continue
        if bit < 4:
            name, scale = (
                ("acceleration_m_s2", 1),
                ("angular_velocity_rad_s", 1),
                ("magnetic_field_t", 1e-6),
                ("euler_rad", math.pi / 180),
            )[bit]
            values[name] = [n * scale for n in struct.unpack_from("<3f", raw, cursor)]
        elif bit == 4:
            values["quaternion_wxyz"] = list(struct.unpack_from("<4f", raw, cursor))
            metadata.update(quaternion_order="wxyz", quaternion_direction="body_to_navigation")
        elif bit == 5:
            stamp = struct.unpack_from("<Q", raw, cursor)[0]
            values.update(device_time_us=stamp, device_time_s=stamp / 1e6)
            metadata["device_time_reference"] = "local_counter"
        elif bit == 6:
            parts = struct.unpack_from("<5BH", raw, cursor)
            values["utc"] = _utc(parts, issues, synchronized=not bool(status & (1 << 11)))
            metadata["utc_raw_components"] = list(parts)
        elif bit == 8:
            values["temperature_c"] = struct.unpack_from("<f", raw, cursor)[0]
        cursor += size


def canfd83_length(bitmap: int) -> int:
    """Logical CANFD83 payload length (12..64) for ``bitmap``, or 0 when invalid."""
    if not isinstance(bitmap, int) or bitmap <= 0 or bitmap & ~CANFD83_MAP_SUPPORTED:
        return 0
    length = 8 + sum(size for bit, size in _FD_SIZES.items() if bitmap & (1 << bit))
    return length if length <= 64 else 0


def _uint(name: str, value: int, maximum: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an integer from 0 to {maximum}")


def _node(node_id: int, *, broadcast: bool = False) -> int:
    """Validate a device address: 1..127 excluding the host address, or 255."""
    if isinstance(node_id, bool) or not isinstance(node_id, int):
        raise ValueError("node_id must be an integer")
    if broadcast and node_id == BROADCAST_ADDRESS:
        return node_id
    if not 1 <= node_id <= 127 or node_id == _HOST_ADDRESS:
        raise ValueError(
            "node_id must be 1..127 excluding the host address 0x55"
            + (", or 255 for broadcast" if broadcast else "")
        )
    return node_id


def _timeout(value: float) -> float:
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError("timeout must be finite and greater than zero")
    return value


def _period_address(pgn: int) -> int:
    _uint("pgn", pgn, 0xFFFF)
    if pgn not in _PGNS:
        raise ValueError("pgn must name a supported HiPNUC measurement PGN")
    return 0x0100 | (pgn & 0xFF)


def _config_message(node_id: int, address: int, command: int, value: int) -> can.Message:
    _uint("address", address, 0xFFFF)
    _uint("value", value, 0xFFFFFFFF)
    import can

    return can.Message(
        arbitration_id=_CONFIG_ID | (node_id << 8) | _HOST_ADDRESS,
        is_extended_id=True,
        data=struct.pack("<HBBI", address, command, 0, value),
        check=True,
    )


def make_trigger(node_id: int, pgn: int) -> can.Message:
    """Build a one-shot PGN trigger; send it with ``bus.send()``.

    ``node_id`` 255 triggers every device on the bus. The device sends the
    requested measurement, without a register ACK. Periodic triggering can
    use python-can's ``bus.send_periodic()``.
    """
    _uint("pgn", pgn, 0xFFFF)
    if pgn not in _PGNS:
        raise ValueError("pgn must name a supported HiPNUC measurement PGN")
    return _config_message(_node(node_id, broadcast=True), _REG_TRIGGER, 6, pgn)


def send_trigger(bus: can.BusABC, node_id: int, pgn: int, timeout: float = 2.0) -> None:
    """Send one PGN trigger to a node or, with ``node_id`` 255, to every node."""
    import can

    request = make_trigger(node_id, pgn)
    try:
        bus.send(request, timeout=_timeout(timeout))
    except can.CanError as exc:
        raise TransportError(f"CAN trigger failed: {exc}") from exc


def read_register(bus: can.BusABC, node_id: int, address: int, timeout: float = 2.0) -> int:
    """Read one raw 32-bit HIREG value; ``timeout`` is total elapsed seconds.

    The caller owns and closes the Bus. Other frames are consumed while waiting
    for the matching response. Firmware may reject an address silently, so a
    timeout does not distinguish rejection from a lost request or response.
    The protocol has no transaction counter: a late response to an earlier
    identical request cannot always be distinguished from a new response.
    """
    return _register_transaction(bus, node_id, address, 3, 0, timeout)


def write_register(
    bus: can.BusABC, node_id: int, address: int, value: int, timeout: float = 2.0
) -> WriteResult:
    """Write one raw 32-bit HIREG value, confirming only its matching echo.

    This does not perform readback. No request is retried, including on
    timeout. Use ``make_trigger`` for the no-ACK trigger and ``reboot`` for
    resets, which the device performs before it can echo the request.
    """
    echoed = _register_transaction(bus, node_id, address, 6, value, timeout)
    if echoed != value:
        raise VerificationError(
            f"CAN node {node_id} register 0x{address:04X} echoed 0x{echoed:08X}, "
            f"expected 0x{value:08X}"
        )
    return WriteResult(address, value, True)


def _register_transaction(
    bus: can.BusABC, node_id: int, address: int, command: int, value: int, timeout: float
) -> int:
    _node(node_id)
    _timeout(timeout)
    request = _config_message(node_id, address, command, value)
    import can

    deadline = time.monotonic() + timeout
    sent_time = time.time()
    expected_id = _CONFIG_ID | (_HOST_ADDRESS << 8) | node_id
    _logger.debug("CAN request 0x%08X %s", request.arbitration_id, bytes(request.data).hex(" "))
    try:
        bus.send(request, timeout=timeout)
        while (remaining := deadline - time.monotonic()) > 0:
            reply = bus.recv(timeout=remaining)
            if reply is None or time.monotonic() >= deadline:
                break
            # A reply stamped on the host clock before this request was queued
            # earlier. Adapter-relative timestamps (seconds since power-up) are
            # far below the epoch and are not judged.
            stale = sent_time - 86400 < reply.timestamp < sent_time
            if (
                reply.arbitration_id != expected_id
                or not reply.is_extended_id
                or not reply.is_rx
                or reply.is_remote_frame
                or reply.is_error_frame
                or reply.is_fd
                or reply.dlc != 8
                or len(reply.data) != 8
                or stale
            ):
                continue
            reply_address, reply_command, status, result = struct.unpack("<HBBI", reply.data)
            if reply_address != address or reply_command != command:
                continue
            _logger.debug("CAN reply   0x%08X %s", reply.arbitration_id, bytes(reply.data).hex(" "))
            if status:
                raise DeviceError(
                    f"CAN node {node_id} rejected register 0x{address:04X} (status {status})",
                    code=status,
                    response=bytes(reply.data).hex(),
                )
            return result
    except can.CanError as exc:
        raise TransportError(f"CAN register transaction failed: {exc}") from exc
    raise ResponseTimeout(
        f"No matching CAN response from node {node_id} for register 0x{address:04X} "
        f"within {timeout:g}s; the device may not support the address or may have "
        "changed its communication settings"
    )


def read_info(bus: can.BusABC, node_id: int, timeout: float = 2.0) -> DeviceInfo:
    """Read product, firmware and serial number through the identity registers.

    Sixteen register reads share the Modbus identity layout; ``timeout`` is
    the total elapsed budget for all reads.
    """
    _node(node_id)
    deadline = time.monotonic() + _timeout(timeout)
    expired = f"CAN node {node_id} identity was not read within {timeout:g}s"
    words = [0] * 19
    for address in _IDENTITY_SLOTS:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ResponseTimeout(expired)
        words[address - 0x70] = (
            _register_transaction(bus, node_id, address, 3, 0, remaining) & 0xFFFF
        )
    if time.monotonic() >= deadline:
        raise ResponseTimeout(expired)
    return identity_from_registers(words)


def save_config(bus: can.BusABC, node_id: int, timeout: float = 2.0) -> WriteResult:
    """Request an explicit save; current firmware persists every write already."""
    return write_register(bus, node_id, _REG_CONTROL, _CONTROL_SAVE, timeout)


def _request_reset(bus: can.BusABC, node_id: int, value: int, timeout: float) -> WriteResult:
    import can

    request = _config_message(node_id, _REG_CONTROL, 6, value)
    try:
        bus.send(request, timeout=_timeout(timeout))
    except can.CanError as exc:
        raise TransportError(f"CAN reset request failed: {exc}") from exc
    return WriteResult(_REG_CONTROL, value, False)


def request_reboot(bus: can.BusABC, node_id: int, timeout: float = 2.0) -> WriteResult:
    """Send the reset request without waiting; it is never acknowledged.

    Use this before the host changes its own bitrate, then ``wait_for_node``.
    """
    return _request_reset(bus, _node(node_id), _CONTROL_REBOOT, timeout)


def factory_reset(bus: can.BusABC, node_id: int, timeout: float = 2.0) -> WriteResult:
    """Restore factory settings; the node ID is retained and bitrates return to defaults."""
    return _request_reset(bus, _node(node_id), _CONTROL_FACTORY_RESET, timeout)


def wait_for_node(
    bus: can.BusABC, node_id: int, recovery_timeout: float = 5.0, timeout: float = 2.0
) -> DeviceInfo:
    """Poll the identity registers until ``node_id`` answers or the wait expires."""
    _node(node_id)
    _timeout(timeout)
    deadline = time.monotonic() + _timeout(recovery_timeout)
    last_error: Exception | None = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ResponseTimeout(
                f"CAN node {node_id} did not answer within {recovery_timeout:g}s after the reset"
            ) from last_error
        try:
            info = read_info(bus, node_id, timeout=min(timeout, remaining))
            if time.monotonic() < deadline:
                return info
        except ResponseTimeout as exc:
            last_error = exc
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(0.05, remaining))


def reboot(
    bus: can.BusABC, node_id: int, *, recovery_timeout: float = 5.0, timeout: float = 2.0
) -> DeviceInfo:
    """Reset the node and wait for its identity at the same address and bitrate."""
    request_reboot(bus, node_id, timeout)
    return wait_for_node(bus, node_id, recovery_timeout, timeout)


def set_node_id(bus: can.BusABC, node_id: int, new_id: int, timeout: float = 2.0) -> WriteResult:
    """Store a new node address (1..127); it takes effect after a reboot.

    Follow with ``request_reboot(bus, node_id)`` and ``wait_for_node(bus, new_id)``.
    """
    _node(new_id)
    return write_register(bus, node_id, _REG_NODE_ID, new_id, timeout)


def _fd_timeout(exc: ResponseTimeout) -> ResponseTimeout:
    return ResponseTimeout(f"{exc}.{_FD_HINT}")


def set_bitrate(
    bus: can.BusABC,
    node_id: int,
    bitrate: int,
    *,
    timeout: float = 2.0,
) -> WriteResult:
    """Store the nominal bitrate; applied after a reboot.

    After ``request_reboot`` the host must reconfigure its own interface before
    the node can be reached again. Set a CAN FD data bitrate separately with
    ``write_register(bus, node_id, 0x009B, DATA_BITRATES[bitrate])``.
    """
    if bitrate not in BITRATES:
        raise ValueError(f"unsupported CAN bitrate; expected one of {sorted(BITRATES)}")
    return write_register(bus, node_id, _REG_NOMINAL_BITRATE, BITRATES[bitrate], timeout)


def set_pgn_period(
    bus: can.BusABC, node_id: int, pgn: int, period_ms: int, timeout: float = 2.0
) -> WriteResult:
    """Set one PGN's broadcast period: 0 disables, ``PERIOD_SYNC_IN`` follows SYNC_IN.

    Periods apply immediately and persist. The firmware does not check the
    value; keep it within 1..32767 ms.
    """
    address = _period_address(pgn)
    if (
        isinstance(period_ms, bool)
        or not isinstance(period_ms, int)
        or not (0 <= period_ms <= 0x7FFF or period_ms == PERIOD_SYNC_IN)
    ):
        raise ValueError("period_ms must be 0..32767 or PERIOD_SYNC_IN")
    try:
        return write_register(bus, node_id, address, period_ms, timeout)
    except ResponseTimeout as exc:
        if pgn == 0xFF5B:
            raise _fd_timeout(exc) from exc
        raise


def set_output_enabled(
    bus: can.BusABC, node_id: int, enabled: bool, timeout: float = 2.0
) -> WriteResult:
    """Switch all periodic PGN output on or off; applied immediately."""
    return write_register(bus, node_id, _REG_OUTPUT_ENABLE, 1 if enabled else 0, timeout)


@dataclass
class ScannedNode:
    """Traffic seen from one source address during ``scan``."""

    node_id: int
    duration_s: float
    frames: int = 0
    fd_frames: int = 0
    invalid_frames: int = 0
    pgns: dict[int, int] = field(default_factory=dict)
    info: DeviceInfo | None = None
    identity_error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "node_id": self.node_id,
            "frames": self.frames,
            "fd_frames": self.fd_frames,
            "invalid_frames": self.invalid_frames,
            "pgns": {
                f"0x{pgn:04X}": {
                    "name": _PGNS[pgn][0],
                    "frames": count,
                    "rate_hz": count / self.duration_s,
                }
                for pgn, count in sorted(self.pgns.items())
            },
            "info": self.info.to_dict() if self.info else None,
            "identity_error": self.identity_error,
        }


def scan(
    bus: can.BusABC, duration: float = 2.0, *, identify: bool = False, timeout: float = 2.0
) -> list[ScannedNode]:
    """Listen passively for ``duration`` seconds and summarize HiPNUC sources.

    Only supported measurement PGNs are counted. With ``identify=True`` each
    source is then asked for its identity; a failed query is kept in
    ``identity_error`` instead of aborting the scan.
    """
    import can

    _timeout(duration)
    nodes: dict[int, ScannedNode] = {}
    deadline = time.monotonic() + duration
    try:
        while (remaining := deadline - time.monotonic()) > 0:
            message = bus.recv(min(remaining, 0.1))
            if message is None or not message.is_extended_id:
                continue
            pgn = _pgn(message.arbitration_id)
            if pgn not in _PGNS:
                continue
            source = message.arbitration_id & 0xFF
            node = nodes.get(source)
            if node is None:
                node = nodes[source] = ScannedNode(source, duration)
            node.frames += 1
            node.fd_frames += bool(message.is_fd)
            node.pgns[pgn] = node.pgns.get(pgn, 0) + 1
            try:
                decode_message(message)
            except ValueError:
                node.invalid_frames += 1
    except can.CanError as exc:
        raise TransportError(f"CAN scan failed: {exc}") from exc
    result = [nodes[node_id] for node_id in sorted(nodes)]
    if identify:
        for node in result:
            try:
                node.info = read_info(bus, node.node_id, timeout)
            except (ValueError, ResponseTimeout, DeviceError) as exc:
                node.identity_error = str(exc)
    return result
