"""HiPNUC J1939 functions for an ordinary ``python-can`` Bus and Message.

Install the optional ``can`` extra to send requests. Decoding does not open a
connection. A register transaction owns reception until it returns: do not
run another reader, updater, or transaction on the same Bus concurrently.
"""

from __future__ import annotations

import math
import struct
import time
from typing import TYPE_CHECKING, Any

from .decoder import GRAVITY, INS_STATUS_NAMES, _make_sample, _utc, status_flags
from .errors import DeviceError, ResponseTimeout, TransportError, VerificationError
from .models import Sample

if TYPE_CHECKING:
    import can


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
_FD_SIZES = {0: 12, 1: 12, 2: 12, 3: 12, 4: 16, 5: 8, 6: 8, 8: 4}


def decode_message(message: can.Message) -> Sample | None:
    """Decode one frame; return ``None`` for unrelated IDs or standard frames.

    A recognized PGN with invalid flags, length or bitmap raises ``ValueError``.
    Measurements describe this frame alone, in SI units. ``raw`` contains the
    data bytes; ID, PGN, channel and FD flags are in ``metadata``. The receive
    time comes from python-can's epoch timestamp, never the device sample clock.
    """
    if not message.is_extended_id:
        return None
    identifier = message.arbitration_id
    pgn = (identifier >> 8) & 0x3FFFF
    if ((pgn >> 8) & 0xFF) < 0xF0:
        pgn &= 0x3FF00
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
        "timestamp_reference": "python_can_receive_time",
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
    sample = _make_sample(kind, values, raw, metadata, issues)
    if math.isfinite(message.timestamp) and message.timestamp > 0:
        sample.received_time_ns = round(message.timestamp * 1_000_000_000)
    return sample


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
    if not bitmap or bitmap & ~0x17F:
        raise ValueError(f"Unsupported CANFD83 bitmap: 0x{bitmap:08X}")
    logical_length = 8 + sum(size for bit, size in _FD_SIZES.items() if bitmap & (1 << bit))
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


def _uint(name: str, value: int, maximum: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an integer from 0 to {maximum}")


def _config_message(node_id: int, address: int, command: int, value: int) -> can.Message:
    _uint("node_id", node_id, 253)
    if node_id == _HOST_ADDRESS:
        raise ValueError("node_id 0x55 is reserved for the configuration host")
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
    """Build a unicast one-shot PGN trigger; send it with ``bus.send()``.

    The device sends the requested measurement, without a register ACK.
    Periodic triggering can use python-can's ``bus.send_periodic()``.
    """
    _uint("pgn", pgn, 0xFFFF)
    if pgn not in _PGNS:
        raise ValueError("pgn must name a supported HiPNUC measurement PGN")
    return _config_message(node_id, 0x0096, 6, pgn)


def read_register(bus: can.BusABC, node_id: int, address: int, timeout: float = 2.0) -> int:
    """Read one raw 32-bit HIREG value; ``timeout`` is total elapsed seconds.

    The caller owns and closes the Bus. Other frames are consumed while waiting
    for the matching response. Firmware may reject an address silently, so a
    timeout does not distinguish rejection from a lost request or response.
    The protocol has no transaction counter: a late response to an earlier
    identical request cannot always be distinguished from a new response.
    """
    return _register_transaction(bus, node_id, address, 3, 1, timeout)


def write_register(
    bus: can.BusABC, node_id: int, address: int, value: int, timeout: float = 2.0
) -> None:
    """Write one raw 32-bit HIREG value, confirming only its matching echo.

    This does not perform readback or guarantee persistence. No request is
    retried, including on timeout. Use ``make_trigger`` for the no-ACK trigger.
    Communication changes and reboot can time out after taking effect.
    """
    echoed = _register_transaction(bus, node_id, address, 6, value, timeout)
    if echoed != value:
        raise VerificationError(
            f"CAN node {node_id} register 0x{address:04X} echoed 0x{echoed:08X}, "
            f"expected 0x{value:08X}"
        )


def _register_transaction(
    bus: can.BusABC, node_id: int, address: int, command: int, value: int, timeout: float
) -> int:
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and greater than zero")
    request = _config_message(node_id, address, command, value)
    import can

    deadline = time.monotonic() + timeout
    sent_time = time.time()
    expected_id = _CONFIG_ID | (_HOST_ADDRESS << 8) | node_id
    try:
        bus.send(request, timeout=timeout)
        while (remaining := deadline - time.monotonic()) > 0:
            reply = bus.recv(timeout=remaining)
            if reply is None or time.monotonic() >= deadline:
                break
            if (
                reply.arbitration_id != expected_id
                or not reply.is_extended_id
                or not reply.is_rx
                or reply.is_remote_frame
                or reply.is_error_frame
                or reply.is_fd
                or reply.dlc != 8
                or len(reply.data) != 8
                or 0 < reply.timestamp < sent_time
            ):
                continue
            reply_address, reply_command, status, result = struct.unpack("<HBBI", reply.data)
            if reply_address != address or reply_command != command:
                continue
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
