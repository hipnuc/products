"""Bounded, transport-independent HiPNUC binary and NMEA stream decoding."""

from __future__ import annotations

from collections import deque
from datetime import date, datetime, time, timezone
import math
import struct
from typing import Any

from .models import Sample


# Product wire convention: HI91 acceleration is encoded as acc / 9.8 (firmware
# GRAVITY constant), so 9.8 recovers the value in m/s^2. Not local gravity.
GRAVITY = 9.8
_SYNC = b"\x5a\xa5"
_UTC_UNSYNC = 1 << 11

# MAIN_STATUS bits shared by HI91/HI81/HI83 (IMU manual, "MAIN_STATUS").
# The names follow the manual; a set bit is a warning, so WB_CONV and ATT_CONV
# mean "NOT converged" while they are present in ``status_flags``.
MAIN_STATUS_FLAGS = {
    3: "WB_CONV",  # set: gyro bias NOT yet converged (keep still for a few seconds)
    4: "MAG_DIST",  # set: magnetic disturbance detected
    5: "ACC_SAT",  # set: accelerometer over range
    6: "GYR_SAT",  # set: gyroscope over range
    7: "ATT_CONV",  # set: attitude NOT yet converged
    9: "STATIC",  # set: device detected as static
    10: "MAG_AIDING",  # set: magnetometer aiding enabled
    11: "UTC_UNSYNC",  # set: device time is NOT synchronized to UTC
    12: "SOUT_PULSE",  # set: this frame corresponds to a SYNC_OUT pulse
}
INS_STATUS_NAMES = {0: "invalid", 1: "aligning", 3: "navigating", 6: "dead_reckoning"}

# HI83 bitmap fields decoded by this SDK. Bits 0-11 are the public IMU/MRU
# fields; 12-19, 30 and 31 are INS delivery extensions. Bits 25-29 are
# internal diagnostics and are not decoded. On the wire, bits 30/31 follow
# bit 27 and precede bits 28/29, so ascending decoding is only valid while
# no undecoded bit is present. Unknown layouts are rejected as a whole.
_HI83_SIZES = {
    0: 12,
    1: 12,
    2: 12,
    3: 12,
    4: 16,
    5: 8,
    6: 8,
    7: 4,
    8: 4,
    9: 12,
    10: 12,
    11: 12,
    12: 12,
    13: 12,
    14: 24,
    15: 4,
    16: 4,
    17: 4,
    18: 4,
    19: 4,
    30: 24,
    31: 12,
}
_HI83_EXTENSION_MASK = 0xC00FF000
_HI83_VECTOR_NAMES = {
    0: "acceleration_m_s2",
    1: "angular_velocity_rad_s",
    2: "magnetic_field_t",
    3: "euler_rad",
    9: "inclination_rad",
    10: "heave_surge_sway_m",
    11: "heave_surge_sway_hz",
    12: "velocity_enu_m_s",
    13: "acceleration_enu_m_s2",
    31: "gnss_velocity_enu_m_s",
}
_HI83_SCALAR_NAMES = {
    7: "pressure_pa",
    8: "temperature_c",
    16: "odometer_speed_m_s",
    17: "geoid_separation_m",
    18: "differential_age_s",
}


def _crc16(data: bytes, crc: int = 0) -> int:
    """CRC-16/XMODEM (poly 0x1021, init 0), as used by the HiPNUC frame header."""
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def status_flags(main_status: int) -> list[str]:
    """Names of the set MAIN_STATUS bits, in bit order. Every listed flag is a warning."""
    return [name for bit, name in MAIN_STATUS_FLAGS.items() if main_status & (1 << bit)]


def _scaled(values: tuple[float, ...], scale: float) -> list[float]:
    return [value * scale for value in values]


def _finite_values(value: Any, issues: list[str], path: str = "") -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        issues.append(f"non_finite:{path}")
        return None
    if isinstance(value, dict):
        return {key: _finite_values(item, issues, key) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_values(item, issues, f"{path}[{i}]") for i, item in enumerate(value)]
    return value


def _units(values: dict[str, Any]) -> dict[str, str]:
    suffixes = (
        ("_m_s2", "m/s^2"),
        ("_rad_s", "rad/s"),
        ("_m_s", "m/s"),
        ("_deg", "deg"),
        ("_rad", "rad"),
        ("_pa", "Pa"),
        ("_hz", "Hz"),
        ("_ms", "ms"),
        ("_us", "us"),
        ("_s", "s"),
        ("_m", "m"),
    )
    result = {}
    for name in values:
        if name == "magnetic_field_t":
            result[name] = "T"
        elif name == "temperature_c":
            result[name] = "degC"
        elif name == "quaternion_wxyz":
            result[name] = "1"
        else:
            for suffix, unit in suffixes:
                if name.endswith(suffix):
                    result[name] = unit
                    break
    return result


def _make_sample(
    kind: str,
    values: dict[str, Any],
    raw: bytes,
    metadata: dict[str, Any],
    issues: list[str] | None = None,
    complete: bool = True,
) -> Sample:
    problems = list(issues or ())
    values = _finite_values(values, problems)
    metadata["units"] = _units(values)
    return Sample(kind, values, raw, complete=complete, issues=tuple(problems), metadata=metadata)


def _binary_metadata(offset: int) -> dict[str, Any]:
    return {
        "protocol": "hipnuc_binary",
        "payload_offset": offset,
        "body_frame": "device_configured_axes",
        "navigation_frame": "device_configured",
        "quaternion_order": "wxyz",
        "quaternion_direction": "body_to_navigation",
        "euler_convention": "device_configured",
    }


def _utc(parts: tuple[int, ...], issues: list[str], synchronized: bool = True) -> datetime | None:
    year, month, day, hour, minute, sec_ms = parts[:6]
    if not any(parts[:6]):
        issues.append("utc_unavailable")
        return None
    if not synchronized:
        issues.append("utc_unsynchronized")
        return None
    try:
        return datetime(
            2000 + year,
            month,
            day,
            hour,
            minute,
            sec_ms // 1000,
            (sec_ms % 1000) * 1000,
            tzinfo=timezone.utc,
        )
    except ValueError:
        issues.append("invalid_utc")
        return None


class Decoder:
    """Incrementally decode arbitrary chunks without opening ports or files.

    Binary frames take precedence over ASCII and NMEA, including when their
    payload contains newlines or ``OK``. A plausible incomplete binary length
    waits for the announced bytes; call ``finish`` at EOF to recover a valid
    suffix after a truncated frame. No host clock or date is inferred.
    After a damaged binary header/checksum, text stays quarantined until a
    checksum-valid binary/NMEA frame or ``reset`` establishes a new boundary,
    so binary payload bytes can never be mistaken for a command reply.

    The default 506-byte payload limit matches the C SDK's 512-byte receive
    buffer, including its 6-byte header.
    """

    def __init__(
        self,
        *,
        max_payload_size: int = 506,
        max_line_size: int = 1024,
        max_pending_lines: int = 128,
    ) -> None:
        if not 1 <= max_payload_size <= 65535:
            raise ValueError("max_payload_size must be between 1 and 65535")
        if max_line_size < 1 or max_pending_lines < 1:
            raise ValueError("line bounds must be positive")
        self.max_payload_size = max_payload_size
        self.max_line_size = max_line_size
        self._capacity = max(max_payload_size + 6, max_line_size)
        self._buffer = bytearray()
        self._text = bytearray()
        self._text_invalid = False
        self._quarantine = False
        self._binary_exclusion = 0
        self._lines: deque[str] = deque(maxlen=max_pending_lines)
        self.statistics = {
            "bytes_received": 0,
            "binary_frames": 0,
            "nmea_frames": 0,
            "samples": 0,
            "crc_errors": 0,
            "length_errors": 0,
            "malformed_packets": 0,
            "nmea_errors": 0,
            "noise_bytes": 0,
            "lines_dropped": 0,
        }

    @property
    def buffered_bytes(self) -> int:
        """Number of undecoded bytes, excluding the bounded text-line queue."""
        return len(self._buffer) + len(self._text)

    def reset(self) -> None:
        """Discard pending input and response lines; retain diagnostic counts."""
        self._buffer.clear()
        self._text.clear()
        self._lines.clear()
        self._text_invalid = False
        self._quarantine = False
        self._binary_exclusion = 0

    def drain_lines(self, *, discard_partial: bool = False) -> list[str]:
        """Drain response lines, optionally abandoning an old ASCII half-line.

        ``discard_partial`` starts a command transaction without joining old
        text to its response. An unfinished binary/NMEA frame is preserved.
        """
        result = list(self._lines)
        self._lines.clear()
        if discard_partial:
            self._text.clear()
            self._text_invalid = False
        return result

    def feed(self, data: bytes | bytearray | memoryview) -> list[Sample]:
        """Return new messages in wire order; retain incomplete input for later.

        Malformed frames are counted and skipped. Decodable messages can
        still contain invalid fields, recorded as ``None`` with ``issues``.
        """
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("Decoder.feed expects bytes-like data")
        data = memoryview(data).cast("B")
        self.statistics["bytes_received"] += len(data)
        result: list[Sample] = []
        cursor = 0
        while cursor < len(data):
            count = min(self._capacity - len(self._buffer), len(data) - cursor)
            self._buffer.extend(data[cursor : cursor + count])
            cursor += count
            result.extend(self._drain())
        return result

    def finish(self) -> list[Sample]:
        """End a finite stream, recovering binary suffixes without fake ACKs."""
        result = self._drain(final=True)
        self._buffer.clear()
        self._text.clear()
        self._text_invalid = False
        self._quarantine = False
        self._binary_exclusion = 0
        return result

    def _consume(self, count: int) -> None:
        del self._buffer[:count]
        self._binary_exclusion = max(0, self._binary_exclusion - count)

    def _ascii(self, data: bytes) -> None:
        for value in data:
            if value == 10:
                if self._text and not self._text_invalid:
                    line = self._text.decode("ascii").strip()
                    if line:
                        if len(self._lines) == self._lines.maxlen:
                            self.statistics["lines_dropped"] += 1
                        self._lines.append(line)
                self._text.clear()
                self._text_invalid = False
            elif value in (9, 13) or 32 <= value <= 126:
                if not self._text_invalid:
                    if len(self._text) < self.max_line_size:
                        self._text.append(value)
                    else:
                        self._text.clear()
                        self._text_invalid = True
                        self.statistics["lines_dropped"] += 1
            else:
                self._text.clear()
                self._text_invalid = True
                self.statistics["noise_bytes"] += 1

    def _drain(self, final: bool = False) -> list[Sample]:
        result: list[Sample] = []
        while self._buffer:
            if self._buffer.startswith(_SYNC):
                self._text.clear()
                self._text_invalid = False
                if len(self._buffer) < 6:
                    if not final:
                        break
                    self._quarantine = True
                    self._binary_exclusion = max(self._binary_exclusion, len(self._buffer))
                    self._consume(1)
                    continue
                length = struct.unpack_from("<H", self._buffer, 2)[0]
                frame_size = length + 6
                if length == 0 or length > self.max_payload_size:
                    self.statistics["length_errors"] += 1
                    self._quarantine = True
                    self._binary_exclusion = max(self._binary_exclusion, 6)
                    self._consume(1)
                    continue
                if len(self._buffer) < frame_size:
                    if not final:
                        break
                    self.statistics["length_errors"] += 1
                    self._quarantine = True
                    self._binary_exclusion = max(self._binary_exclusion, len(self._buffer))
                    self._consume(1)
                    continue
                frame = bytes(self._buffer[:frame_size])
                crc = _crc16(frame[6:], _crc16(frame[:4]))
                if crc != struct.unpack_from("<H", frame, 4)[0]:
                    self.statistics["crc_errors"] += 1
                    self._quarantine = True
                    self._binary_exclusion = max(self._binary_exclusion, frame_size)
                    self._consume(1)
                    continue
                self._consume(frame_size)
                self._quarantine = False
                self._binary_exclusion = 0
                self.statistics["binary_frames"] += 1
                try:
                    result.extend(self._decode_payload(frame))
                except (struct.error, ValueError):
                    self.statistics["malformed_packets"] += 1
                continue
            if self._quarantine and not (
                self._binary_exclusion == 0 and self._buffer[0] == ord("$")
            ):
                boundaries = [
                    index
                    for index in (
                        self._buffer.find(_SYNC),
                        self._buffer.find(b"$", self._binary_exclusion),
                    )
                    if index >= 0
                ]
                count = len(self._buffer)
                if boundaries:
                    count = min(count, min(boundaries))
                elif self._buffer[-1] == _SYNC[0] and not final:
                    count = min(count, len(self._buffer) - 1)
                if count == 0:
                    break
                self._consume(count)
                continue
            if self._buffer[0] == ord("$"):
                self._text.clear()
                self._text_invalid = False
                newline = self._buffer.find(b"\n")
                restart = [
                    index
                    for index in (self._buffer.find(b"$", 1), self._buffer.find(_SYNC, 1))
                    if index >= 0
                ]
                if restart and (newline < 0 or min(restart) < newline):
                    self.statistics["nmea_errors"] += 1
                    self._consume(min(restart))
                    continue
                if newline < 0:
                    if len(self._buffer) < self.max_line_size and not final:
                        break
                    self.statistics["nmea_errors"] += 1
                    count = len(self._buffer)
                    if self._buffer[-1] == _SYNC[0] and not final:
                        count -= 1
                    self._consume(count)
                    self._text_invalid = True
                    continue
                line = bytes(self._buffer[: newline + 1])
                self._consume(newline + 1)
                if len(line) > self.max_line_size:
                    self.statistics["nmea_errors"] += 1
                    continue
                try:
                    sample = _decode_nmea(line)
                except (ValueError, IndexError, UnicodeError):
                    self.statistics["nmea_errors"] += 1
                else:
                    self.statistics["nmea_frames"] += 1
                    self._quarantine = False
                    self._binary_exclusion = 0
                    result.append(sample)
                continue
            starts = [
                index for index in (self._buffer.find(_SYNC), self._buffer.find(b"$")) if index >= 0
            ]
            count = min(starts) if starts else len(self._buffer)
            if not starts and self._buffer[-1] == _SYNC[0] and not final:
                count -= 1
            if not count:
                break
            self._ascii(bytes(self._buffer[:count]))
            self._consume(count)
        self.statistics["samples"] += len(result)
        return result

    @staticmethod
    def _decode_payload(frame: bytes) -> list[Sample]:
        payload = frame[6:]
        if not payload:
            raise ValueError("empty binary payload")
        # Current products emit exactly one packet per CRC envelope. Do not
        # silently prefer one packet or invent a multi-packet delivery policy.
        if payload[0] == 0x91 and len(payload) == 76:
            return [_decode_hi91(payload, frame, 0)]
        if payload[0] == 0x81 and len(payload) == 104:
            return [_decode_hi81(payload, frame, 0)]
        if payload[0] == 0x83:
            return [_decode_hi83(payload, frame, 0)]
        raise ValueError("unsupported tag or payload length")


def _decode_hi91(data: bytes, frame: bytes, offset: int) -> Sample:
    status, temperature, pressure, timestamp = struct.unpack_from("<HbfI", data, 1)
    values = {
        "main_status": status,
        "status_flags": status_flags(status),
        "temperature_c": temperature,
        "pressure_pa": pressure,
        "device_time_ms": timestamp,
        "device_time_s": timestamp / 1000,
        "acceleration_m_s2": _scaled(struct.unpack_from("<3f", data, 12), GRAVITY),
        "angular_velocity_rad_s": _scaled(struct.unpack_from("<3f", data, 24), math.pi / 180),
        "magnetic_field_t": _scaled(struct.unpack_from("<3f", data, 36), 1e-6),
        "euler_rad": _scaled(struct.unpack_from("<3f", data, 48), math.pi / 180),
        "quaternion_wxyz": list(struct.unpack_from("<4f", data, 60)),
    }
    metadata = _binary_metadata(offset)
    metadata["device_time_reference"] = (
        "local_counter" if status & _UTC_UNSYNC else "utc_time_of_day"
    )
    metadata["device_time_date_known"] = False
    metadata["euler_components"] = ["roll", "pitch", "yaw"]
    return _make_sample("HI91", values, frame, metadata)


def _decode_hi81(data: bytes, frame: bytes, offset: int) -> Sample:
    status, ins_status, week, tow = struct.unpack_from("<HBHI", data, 1)
    issues: list[str] = []
    utc_parts = struct.unpack_from("<5BH", data, 35)
    values = {
        "main_status": status,
        "status_flags": status_flags(status),
        "ins_status": ins_status,
        "ins_status_name": INS_STATUS_NAMES.get(ins_status, "unknown"),
        "gps_week": week,
        "gps_time_of_week_ms": tow,
        "gps_time_of_week_s": tow * 0.001,
        "angular_velocity_rad_s": _scaled(struct.unpack_from("<3h", data, 12), 0.001),
        "acceleration_m_s2": _scaled(struct.unpack_from("<3h", data, 18), 0.0048828),
        "magnetic_field_t": _scaled(struct.unpack_from("<3h", data, 24), 0.030517e-6),
        "pressure_pa": struct.unpack_from("<h", data, 30)[0] + 100000,
        "odometer_speed_m_s": struct.unpack_from("<h", data, 32)[0] * 0.01,
        "temperature_c": struct.unpack_from("<b", data, 34)[0],
        "utc": _utc(utc_parts, issues, synchronized=not bool(status & _UTC_UNSYNC)),
        "roll_rad": math.radians(struct.unpack_from("<h", data, 42)[0] * 0.01),
        "pitch_rad": math.radians(struct.unpack_from("<h", data, 44)[0] * 0.01),
        "heading_rad": math.radians(struct.unpack_from("<H", data, 46)[0] * 0.01),
        "quaternion_wxyz": _scaled(struct.unpack_from("<4h", data, 48), 0.0001),
        "longitude_deg": struct.unpack_from("<i", data, 56)[0] * 1e-7,
        "latitude_deg": struct.unpack_from("<i", data, 60)[0] * 1e-7,
        "altitude_msl_m": struct.unpack_from("<i", data, 64)[0] * 0.001,
        "pdop": data[68] * 0.1,
        "hdop": data[69] * 0.1,
        "position_quality": data[70],
        "position_satellites": data[71],
        "heading_quality": data[72],
        "heading_satellites": data[73],
        "differential_age_s": data[74],
        "geoid_separation_m": struct.unpack_from("<h", data, 75)[0] * 0.01,
        "antenna_status": data[77],
        "velocity_enu_m_s": _scaled(struct.unpack_from("<3h", data, 78), 0.01),
        "acceleration_enu_m_s2": _scaled(struct.unpack_from("<3h", data, 84), 0.0048828),
    }
    metadata = _binary_metadata(offset)
    metadata.update(
        heading_reference="north_clockwise",
        position_source="ins",
        position_datum="WGS84",
        altitude_reference="mean_sea_level",
        navigation_vector_frame="ENU",
        gps_time_reference="GPST",
        gps_time_available=bool(week or tow),
        utc_raw_components=list(utc_parts),
        # Bytes 90..103 are reserved and always zero on current firmware.
        reserved_tail_hex=data[90:104].hex(),
    )
    return _make_sample("HI81", values, frame, metadata, issues)


def _decode_hi83(data: bytes, frame: bytes, offset: int) -> Sample:
    _, status, ins_status, bitmap = struct.unpack_from("<BHBI", data)
    unknown = bitmap & ~sum(1 << bit for bit in _HI83_SIZES)
    if unknown:
        raise ValueError(f"unsupported HI83 bitmap: 0x{unknown:08x}")
    expected = 8 + sum(size for bit, size in _HI83_SIZES.items() if bitmap & (1 << bit))
    if len(data) != expected:
        raise ValueError("HI83 bitmap does not match payload length")
    values: dict[str, Any] = {
        "main_status": status,
        "status_flags": status_flags(status),
        "ins_status": ins_status,
        "ins_status_name": INS_STATUS_NAMES.get(ins_status, "unknown"),
        "data_bitmap": bitmap,
    }
    metadata = _binary_metadata(offset)
    metadata["euler_components"] = ["roll", "pitch", "yaw"]
    issues: list[str] = []
    cursor = 8
    for bit in range(32):
        if not bitmap & (1 << bit):
            continue
        size = _HI83_SIZES[bit]
        if cursor + size > len(data):
            raise ValueError("truncated HI83 field")
        if bit in _HI83_VECTOR_NAMES:
            if bit == 2:
                scale = 1e-6
            elif bit in (3, 9):
                scale = math.pi / 180
            else:
                scale = 1
            vector = _scaled(struct.unpack_from("<3f", data, cursor), scale)
            if bit == 9:
                values["inclination_rad"] = vector[:2]
                values["inclination_yaw_rad"] = vector[2]
            else:
                values[_HI83_VECTOR_NAMES[bit]] = vector
        elif bit == 4:
            values["quaternion_wxyz"] = list(struct.unpack_from("<4f", data, cursor))
        elif bit == 5:
            stamp = struct.unpack_from("<Q", data, cursor)[0]
            values["device_time_us"] = stamp
            values["device_time_s"] = stamp / 1_000_000
            metadata["device_time_reference"] = "local_counter"
        elif bit == 6:
            parts = struct.unpack_from("<5BH", data, cursor)
            values["utc"] = _utc(parts, issues, synchronized=not bool(status & _UTC_UNSYNC))
            metadata["utc_raw_components"] = list(parts)
        elif bit in _HI83_SCALAR_NAMES:
            values[_HI83_SCALAR_NAMES[bit]] = struct.unpack_from("<f", data, cursor)[0]
        elif bit in (14, 30):
            longitude, latitude, altitude = struct.unpack_from("<3d", data, cursor)
            prefix = "gnss_" if bit == 30 else ""
            values.update(
                {
                    prefix + "longitude_deg": longitude,
                    prefix + "latitude_deg": latitude,
                    prefix + "altitude_msl_m": altitude,
                }
            )
            metadata.update(position_datum="WGS84", altitude_reference="mean_sea_level")
        elif bit == 15:
            values.update(
                zip(
                    (
                        "position_quality",
                        "position_satellites",
                        "heading_quality",
                        "heading_satellites",
                    ),
                    struct.unpack_from("<4B", data, cursor),
                )
            )
        elif bit == 19:
            values["node_id"] = data[cursor]
        cursor += size
    if bitmap & _HI83_EXTENSION_MASK:
        metadata["hi83_extension_bits"] = "bits 12-19, 30, 31 are INS delivery extensions"
    return _make_sample("HI83", values, frame, metadata, issues)


def _nmea_number(value: str, scale: float = 1) -> float | None:
    return float(value) * scale if value else None


def _nmea_int(value: str) -> int | None:
    return int(value) if value else None


def _nmea_time(value: str, issues: list[str]) -> time | None:
    if not value:
        return None
    try:
        whole, _, fraction = value.partition(".")
        if len(whole) != 6 or not whole.isdigit() or (fraction and not fraction.isdigit()):
            raise ValueError("invalid time syntax")
        return time(
            int(whole[:2]),
            int(whole[2:4]),
            int(whole[4:]),
            int((fraction + "000000")[:6]),
            tzinfo=timezone.utc,
        )
    except ValueError:
        issues.append("invalid_utc_time")
        return None


def _nmea_date(value: str, issues: list[str]) -> date | None:
    if not value:
        return None
    try:
        if len(value) != 6 or not value.isdigit():
            raise ValueError("invalid date syntax")
        year = int(value[4:])
        return date(1900 + year if year >= 69 else 2000 + year, int(value[2:4]), int(value[:2]))
    except ValueError:
        issues.append("invalid_utc_date")
        return None


def _nmea_coordinate(value: str, direction: str, axis: str, issues: list[str]) -> float | None:
    if not value and not direction:
        return None
    try:
        raw = float(value)
        degrees = int(raw // 100)
        minutes = raw - degrees * 100
        limit = 90 if axis == "latitude" else 180
        signs = {"N": 1, "S": -1} if axis == "latitude" else {"E": 1, "W": -1}
        if raw < 0 or not 0 <= minutes < 60 or degrees > limit or (degrees == limit and minutes):
            raise ValueError("invalid coordinate")
        return (degrees + minutes / 60) * signs[direction]
    except (ValueError, KeyError, OverflowError):
        issues.append(f"invalid_{axis}")
        return None


def _decode_nmea(raw: bytes) -> Sample:
    """Decode the GGA and RMC sentences emitted by HiPNUC INS products."""
    sentence = raw.rstrip(b"\r\n").decode("ascii")
    header = sentence[1:6]
    if len(header) != 5 or not header.isalpha() or not header.isupper():
        raise ValueError("invalid NMEA header")
    body, _, checksum = sentence[1:].rpartition("*")
    if len(checksum) != 2 or any(char not in "0123456789abcdefABCDEF" for char in checksum):
        raise ValueError("invalid NMEA checksum syntax")
    computed = 0
    for value in body.encode("ascii"):
        computed ^= value
    if computed != int(checksum, 16):
        raise ValueError("NMEA checksum mismatch")
    fields = body.split(",")
    kind = header[2:]
    metadata: dict[str, Any] = {"protocol": "nmea", "talker_id": header[:2]}
    minimum = {"GGA": 15, "RMC": 10}
    if kind not in minimum:
        return _make_sample(
            kind,
            {"fields": fields[1:]},
            raw,
            metadata,
            ["unsupported_nmea_sentence"],
            complete=False,
        )
    if len(fields) < minimum[kind]:
        raise ValueError("truncated NMEA sentence")
    issues: list[str] = []
    latitude_index = 2 if kind == "GGA" else 3
    longitude_index = 4 if kind == "GGA" else 5
    values: dict[str, Any] = {
        "utc_time": _nmea_time(fields[1], issues),
        "latitude_deg": _nmea_coordinate(
            fields[latitude_index], fields[latitude_index + 1], "latitude", issues
        ),
        "longitude_deg": _nmea_coordinate(
            fields[longitude_index], fields[longitude_index + 1], "longitude", issues
        ),
    }
    metadata.update(position_source="gnss", position_datum="WGS84")
    if kind == "GGA":
        values.update(
            position_quality=_nmea_int(fields[6]),
            position_satellites=_nmea_int(fields[7]),
            hdop=_nmea_number(fields[8]),
            altitude_msl_m=_nmea_number(fields[9]),
            geoid_separation_m=_nmea_number(fields[11]),
            differential_age_s=_nmea_number(fields[13]),
            differential_station_id=fields[14] or None,
        )
        quality = values["position_quality"]
        values["fix_valid"] = quality > 0 if quality is not None else None
        if values["fix_valid"] is False:
            issues.append("gnss_fix_invalid")
        for index, name in ((10, "altitude_msl_m"), (12, "geoid_separation_m")):
            if values[name] is not None and fields[index] != "M":
                issues.append(f"unsupported_unit:{name}")
                values[name] = None
        metadata.update(
            utc_reference="time_of_day_without_date", altitude_reference="mean_sea_level"
        )
    else:
        date_value = _nmea_date(fields[9], issues)
        values.update(
            position_status=fields[2],
            utc_date=date_value,
            utc=datetime.combine(date_value, values["utc_time"])
            if date_value and values["utc_time"]
            else None,
            speed_over_ground_m_s=_nmea_number(fields[7], 1852 / 3600),
            course_over_ground_rad=_nmea_number(fields[8], math.pi / 180),
            mode=fields[12] if len(fields) > 12 else None,
        )
        values["fix_valid"] = fields[2] == "A" if fields[2] else None
        if values["fix_valid"] is False:
            issues.append("gnss_fix_invalid")
        metadata.update(course_reference="true_north_clockwise", two_digit_year_window="1969-2068")
    return _make_sample(kind, values, raw, metadata, issues)
