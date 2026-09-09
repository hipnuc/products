"""Public data models shared by serial, CAN, file, and Modbus clients."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date, datetime, time
import math
from typing import Any, cast


Vector2 = tuple[float | None, float | None]
Vector3 = tuple[float | None, float | None, float | None]
Vector4 = tuple[float | None, float | None, float | None, float | None]


def _finite_number(value: Any) -> float | None:
    return float(value) if value is not None and math.isfinite(value) else None


def _json_value(value: Any) -> Any:
    """Return a JSON-compatible copy, without emitting NaN or Infinity."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (datetime, time)):
        return value.isoformat().replace("+00:00", "Z")
    if isinstance(value, date):
        return value.isoformat()
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


@dataclass
class Sample:
    """One decoded message, with explicit physical units in ``values`` keys.

    ``complete`` describes decoding completeness, not sensor validity or fix
    quality. Check status fields and ``issues`` before using measurements.
    ``received_time_ns`` is supplied by the transport; a Decoder never invents
    host time. ``raw`` preserves a serial frame/sentence including its checksum,
    the CAN data payload (identifier and flags are in ``metadata``), or the
    measurement register block for Modbus (not an RTU frame).
    Measurement properties read ``values`` directly and return ``None`` for
    unavailable fields or non-finite components; no measurements are cached.
    """

    type: str
    values: dict[str, Any]
    raw: bytes
    received_time_ns: int | None = None
    complete: bool = True
    issues: tuple[str, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def _vector(self, name: str) -> tuple[float | None, ...] | None:
        value = self.values.get(name)
        return None if value is None else tuple(_finite_number(item) for item in value)

    @property
    def acceleration_m_s2(self) -> Vector3 | None:
        """Acceleration along the three device-configured body axes, in m/s²."""
        return cast(Vector3 | None, self._vector("acceleration_m_s2"))

    @property
    def angular_velocity_rad_s(self) -> Vector3 | None:
        """Angular velocity along the three body axes, in rad/s."""
        return cast(Vector3 | None, self._vector("angular_velocity_rad_s"))

    @property
    def magnetic_field_t(self) -> Vector3 | None:
        """Magnetic field along the three body axes, in tesla."""
        return cast(Vector3 | None, self._vector("magnetic_field_t"))

    @property
    def euler_rad(self) -> Vector3 | None:
        """Roll, pitch and yaw, in radians; preserve the device Euler convention."""
        return cast(Vector3 | None, self._vector("euler_rad"))

    @property
    def quaternion_wxyz(self) -> Vector4 | None:
        """Body-to-navigation quaternion in WXYZ order."""
        return cast(Vector4 | None, self._vector("quaternion_wxyz"))

    @property
    def roll_rad(self) -> float | None:
        """INS roll, in radians."""
        return _finite_number(self.values.get("roll_rad"))

    @property
    def pitch_rad(self) -> float | None:
        """INS pitch, in radians."""
        return _finite_number(self.values.get("pitch_rad"))

    @property
    def heading_rad(self) -> float | None:
        """INS heading, in radians; distinct from Euler yaw and course over ground."""
        return _finite_number(self.values.get("heading_rad"))

    @property
    def latitude_deg(self) -> float | None:
        """Latitude, in degrees; inspect position status before using it."""
        return _finite_number(self.values.get("latitude_deg"))

    @property
    def longitude_deg(self) -> float | None:
        """Longitude, in degrees; inspect position status before using it."""
        return _finite_number(self.values.get("longitude_deg"))

    @property
    def altitude_msl_m(self) -> float | None:
        """Altitude above mean sea level, in metres."""
        return _finite_number(self.values.get("altitude_msl_m"))

    @property
    def geoid_separation_m(self) -> float | None:
        """Geoid separation, in metres; distinct from altitude."""
        return _finite_number(self.values.get("geoid_separation_m"))

    @property
    def velocity_enu_m_s(self) -> Vector3 | None:
        """East, north and up velocity, in m/s."""
        return cast(Vector3 | None, self._vector("velocity_enu_m_s"))

    @property
    def heave_surge_sway_m(self) -> Vector3 | None:
        """MRU displacement in heave, surge, sway order, in metres."""
        return cast(Vector3 | None, self._vector("heave_surge_sway_m"))

    @property
    def heave_surge_sway_hz(self) -> Vector3 | None:
        """MRU frequency in heave, surge, sway order, in hertz."""
        return cast(Vector3 | None, self._vector("heave_surge_sway_hz"))

    @property
    def pressure_pa(self) -> float | None:
        """Pressure, in pascals."""
        return _finite_number(self.values.get("pressure_pa"))

    @property
    def temperature_c(self) -> float | None:
        """Temperature, in degrees Celsius."""
        return _finite_number(self.values.get("temperature_c"))

    @property
    def inclination_rad(self) -> Vector2 | None:
        """Two independent inclination angles, in radians; not Euler roll/pitch."""
        return cast(Vector2 | None, self._vector("inclination_rad"))

    def to_dict(self, *, include_raw: bool = False) -> dict[str, Any]:
        """Export a flat record suitable for ``json.dumps(allow_nan=False)``."""
        result = _json_value(self.values)
        result.update(
            type=self.type,
            complete=self.complete,
            issues=list(self.issues),
            metadata=_json_value(self.metadata),
        )
        if self.received_time_ns is not None:
            result["received_time_ns"] = self.received_time_ns
        if include_raw:
            result["raw_hex"] = self.raw.hex()
        return result


@dataclass
class DeviceInfo:
    """Identity reported by a device; unavailable fields remain ``None``."""

    product_name: str | None = None
    firmware_version: str | None = None
    bootloader_version: str | None = None
    serial_number: str | None = None
    build: str | None = None
    raw_response: str = ""

    def to_dict(self) -> dict[str, Any]:
        return _json_value(vars(self))


@dataclass
class CommandResult:
    """ASCII command reply; acknowledgement does not imply readback verification."""

    command: str
    text: str
    acknowledged: bool

    def to_dict(self) -> dict[str, Any]:
        return _json_value(vars(self))
