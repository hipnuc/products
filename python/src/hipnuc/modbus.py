"""Synchronous HiPNUC Modbus RTU access through PyModbus 3.15.

Register addresses are zero based. One :class:`ModbusBus` owns one serial
port and serializes complete operations for all its devices. Only FC03 and
FC06 are used; neither broadcasts nor Modbus TCP are provided.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException, ModbusIOException

from ._connection import SERIAL_OPEN_ERRORS, open_error
from .decoder import GRAVITY, status_flags
from .errors import DeviceError, ResponseTimeout, TransportError, VerificationError
from .models import DeviceInfo, Sample

# Public IMU/AHRS manual, Modbus RTU baudrate register codes (code = index).
BAUDRATES = (4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 256000)
_MAGNETIC_COUNTS_PER_UT = 32.768


@dataclass(frozen=True)
class WriteResult:
    """An echo acknowledges receipt; only a readback verifies a value/state."""

    address: int
    value: int
    acknowledged: bool
    verified: bool = False
    readback: int | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _integer(value: int, minimum: int, maximum: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    return value


def _duration(value: float, name: str) -> float:
    if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _words_bytes(registers: list[int]) -> bytes:
    return struct.pack(f">{len(registers)}H", *registers)


def _version(value: int) -> str | None:
    return f"{value // 100}.{value // 10 % 10}.{value % 10}" if value else None


class ModbusBus:
    """One synchronous RTU master port shared by unicast devices.

    The port is opened lazily or by entering a context manager. Requests are
    never retried automatically: replaying a write may repeat a device action.
    ``handle_local_echo`` is for USB/RS-485 adapters that echo transmitted bytes.
    ``timeout`` is processing/host-wait margin in seconds; each transaction adds
    its request and response wire time plus a 3.5-character turnaround.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.5,
        *,
        handle_local_echo: bool = False,
    ) -> None:
        if not isinstance(port, str) or not port:
            raise ValueError("port must be a nonempty serial port name")
        self.port = port
        self.baudrate = _integer(baudrate, 1, 10_000_000, "baudrate")
        self.timeout = _duration(timeout, "timeout")
        self.handle_local_echo = handle_local_echo
        self._lock = threading.RLock()
        self._client = self._make_client()

    def _make_client(self) -> ModbusSerialClient:
        return ModbusSerialClient(
            self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
            retries=0,
            handle_local_echo=self.handle_local_echo,
        )

    def open(self) -> ModbusBus:
        """Open the shared port, returning self; raise TransportError on failure."""
        with self._lock:
            try:
                if not self._client.connect():
                    # PyModbus can return False after logging the OS exception.
                    raise open_error(self.port)
            except BaseException as exc:
                # __exit__ is not called if __enter__/open fails partway through.
                try:
                    self._client.close()
                except OSError:
                    pass
                if isinstance(exc, SERIAL_OPEN_ERRORS + (ModbusException,)):
                    raise open_error(self.port, exc) from exc
                raise
        return self

    def close(self) -> None:
        """Close the shared port; bound devices can reopen it on their next request."""
        with self._lock:
            self._client.close()

    def __enter__(self) -> ModbusBus:
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    def device(self, device_id: int = 80) -> ModbusDevice:
        """Bind a unicast node; no probing, broadcasting or configuration occurs."""
        return ModbusDevice(self, device_id)

    def reconfigure(self, baudrate: int) -> None:
        """Change only the master's baudrate and reopen the shared port."""
        baudrate = _integer(baudrate, 1, 10_000_000, "baudrate")
        with self._lock:
            self.close()
            self.baudrate = baudrate
            self._client = self._make_client()
            self.open()

    def _request(self, device_id: int, function: int, address: int, argument: int) -> Any:
        # The caller holds the bus lock for both this request and its readback.
        try:
            response_bytes = 5 + 2 * argument if function == 3 else 8
            budget = self.timeout + (8 + response_bytes + 3.5) * 10 / self.baudrate
            # PyModbus 3.15 keeps separate CommParams copies: the serial client
            # uses one for recv(), and the transaction manager another for its
            # complete-response deadline. Update both for EVERY request.
            self._client.comm_params.timeout_connect = budget
            self._client.transaction.comm_params.timeout_connect = budget
            self.open()
            self._client.socket.timeout = budget
            self._client.socket.write_timeout = budget
            if function == 3:
                response = self._client.read_holding_registers(
                    address,
                    count=argument,
                    device_id=device_id,
                )
            else:
                response = self._client.write_register(address, argument, device_id=device_id)
        except (TimeoutError, ModbusIOException) as exc:
            if isinstance(exc, TimeoutError) or "no response received" in str(exc).lower():
                raise ResponseTimeout(
                    f"No Modbus response from node {device_id}: {exc}. Check the node ID, "
                    "baudrate and RS-485 wiring (A/B)."
                ) from exc
            raise TransportError(f"Invalid Modbus response from node {device_id}: {exc}") from exc
        except (OSError, ModbusException) as exc:
            raise TransportError(
                f"Modbus communication with node {device_id} failed: {exc}"
            ) from exc
        if response is None:
            raise ResponseTimeout(f"No Modbus response from node {device_id}")
        if response.isError():
            raise DeviceError(
                f"Node {device_id} rejected FC{function:02d} at 0x{address:04X}",
                code=response.exception_code,
                response=str(response),
            )
        if response.dev_id != device_id or response.function_code != function:
            raise VerificationError("Modbus response does not match the requested node/function")
        return response


class ModbusDevice:
    """One HiPNUC device on a :class:`ModbusBus`."""

    def __init__(self, bus: ModbusBus, device_id: int = 80) -> None:
        self.bus = bus
        self.device_id = _integer(device_id, 1, 247, "device_id")
        self._info: DeviceInfo | None = None

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        """Read 1..125 raw unsigned 16-bit holding registers using FC03."""
        address = _integer(address, 0, 0xFFFF, "address")
        count = _integer(count, 1, 125, "count")
        if address + count > 0x10000:
            raise ValueError("register range exceeds 0xFFFF")
        with self.bus._lock:
            response = self.bus._request(self.device_id, 3, address, count)
            if len(response.registers) != count:
                raise VerificationError(
                    f"FC03 returned {len(response.registers)} registers; expected {count}"
                )
            return list(response.registers)

    def _write_ack(self, address: int, value: int) -> WriteResult:
        response = self.bus._request(self.device_id, 6, address, value)
        if response.address != address or response.registers != [value]:
            raise VerificationError("FC06 echo does not match the requested address/value")
        return WriteResult(address, value, True)

    def write_register(self, address: int, value: int, *, verify: bool = True) -> WriteResult:
        """Write one raw u16 using FC06, optionally verifying a readable register.

        This low-level API does not save, reboot, or update the device binding.
        Use ``set_id`` for node changes and ``set_baudrate`` for baud changes.
        For write-only controls explicitly pass ``verify=False``.
        """
        address = _integer(address, 0, 0xFFFF, "address")
        value = _integer(value, 0, 0xFFFF, "value")
        with self.bus._lock:
            try:
                result = self._write_ack(address, value)
            except ResponseTimeout:
                if not verify:
                    raise
                result = WriteResult(address, value, False)
            if not verify:
                return result
            actual = self.read_registers(address)[0]
            if actual != value:
                raise VerificationError(
                    f"Write to 0x{address:04X} was echoed but read back {actual}, expected {value}"
                )
            return WriteResult(address, value, result.acknowledged, True, actual)

    def read_info(self) -> DeviceInfo:
        """Read the documented 19-register identity block; cache its identity."""
        with self.bus._lock:
            raw = _words_bytes(self.read_registers(0x70, 19))
            product_name = raw[:16].split(b"\0", 1)[0].decode("ascii", errors="replace").strip()
            serial = raw[30:38]
            self._info = DeviceInfo(
                product_name=product_name or None,
                firmware_version=_version(int.from_bytes(raw[16:18], "big")),
                bootloader_version=_version(int.from_bytes(raw[18:20], "big")),
                serial_number=serial.hex().upper() if any(serial) else None,
                build=None,
                raw_response=raw.hex(),
            )
            return self._info

    def read_status(self) -> dict[str, Any]:
        """Read main status (with flag names), calibration status and progress."""
        with self.bus._lock:
            status, calibration, progress = self.read_registers(0x09, 3)
            return {
                "main_status": status,
                "status_flags": status_flags(status),
                "calibration_status": calibration,
                "calibration_progress_percent": progress,
            }

    def read_sample(self, *, include_status: bool = True, include_mru: bool = False) -> Sample:
        """Read the public IMU/AHRS block in SI units (temperature in degrees C).

        Register and status reads are not promised to be an atomic firmware
        sample. CPUTIME is boot-relative, never UTC. ``raw`` contains big-endian
        register bytes, not an RTU ADU.
        """
        with self.bus._lock:
            info = self._info or self.read_info()
            count = 32 if include_mru else 26
            raw = _words_bytes(self.read_registers(0x34, count))
            received = time.time_ns()
            vectors = struct.unpack_from(">9h", raw)
            uptime_ms = struct.unpack_from(">I", raw, 48)[0]
            values: dict[str, Any] = {
                "acceleration_m_s2": tuple(v * GRAVITY / 2048.0 for v in vectors[:3]),
                "angular_velocity_rad_s": tuple(math.radians(v / 16.384) for v in vectors[3:6]),
                "magnetic_field_t": tuple(v / _MAGNETIC_COUNTS_PER_UT * 1e-6 for v in vectors[6:9]),
                "euler_rad": tuple(
                    math.radians(v / 1000.0) for v in struct.unpack_from(">3i", raw, 18)
                ),
                "temperature_c": struct.unpack_from(">h", raw, 30)[0] / 100.0,
                "pressure_pa": struct.unpack_from(">i", raw, 32)[0] / 100.0,
                "quaternion_wxyz": tuple(v / 10000.0 for v in struct.unpack_from(">4h", raw, 36)),
                "inclination_rad": tuple(
                    math.radians(v * 0.011) for v in struct.unpack_from(">2h", raw, 44)
                ),
                "device_time_us": uptime_ms * 1000,
                "device_time_s": uptime_ms / 1000.0,
            }
            if include_mru:
                displacement = struct.unpack_from(">3h", raw, 52)
                frequency = struct.unpack_from(">3h", raw, 58)
                values["heave_surge_sway_m"] = tuple(value / 100.0 for value in displacement)
                values["heave_surge_sway_hz"] = tuple(value / 100.0 for value in frequency)
            if include_status:
                values.update(self.read_status())
            return Sample(
                type="MODBUS",
                values=values,
                raw=raw,
                received_time_ns=received,
                complete=True,
                issues=(),
                metadata={
                    "protocol": "modbus_rtu",
                    "device_id": self.device_id,
                    "firmware_version": info.firmware_version,
                    "register_start": 0x34,
                    "register_count": count,
                    "raw_format": "registers_be",
                    "register_snapshot": "not_guaranteed",
                    "body_frame": "device_configured_axes",
                    "navigation_frame": "device_configured",
                    "quaternion_order": "wxyz",
                    "quaternion_direction": "body_to_navigation",
                    "euler_convention": "device_configured",
                    "euler_components": ["roll", "pitch", "yaw"],
                    "magnetic_counts_per_microtesla": _MAGNETIC_COUNTS_PER_UT,
                    "device_time": {
                        "source": "CPUTIME",
                        "epoch": "boot",
                        "unit": "us",
                        "wrap_ms": 2**32,
                    },
                },
            )

    def save_config(self) -> WriteResult:
        """Save once for older firmware; newer firmware accepts this as a no-op."""
        return self.write_register(0x00, 0x00, verify=False)

    def set_id(self, device_id: int, *, save: bool = False) -> WriteResult:
        """Write a new node ID, rebind this object and read the ID back there.

        The target address must be free on the bus; nothing is scanned.
        """
        device_id = _integer(device_id, 1, 247, "device_id")
        with self.bus._lock:
            previous = self.device_id
            acknowledged = True
            try:
                self._write_ack(0x05, device_id)
            except ResponseTimeout:
                acknowledged = False
            self.device_id = device_id
            try:
                actual = self.read_registers(0x05)[0]
            except (TransportError, DeviceError):
                self.device_id = previous
                raise
            if actual != device_id:
                self.device_id = previous
                raise VerificationError(f"Node ID read back {actual}, expected {device_id}")
            if save:
                self.save_config()
            return WriteResult(0x05, device_id, acknowledged, True, actual)

    def set_baudrate(
        self,
        baudrate: int,
        *,
        reboot: bool = False,
        save: bool = False,
        timeout: float = 5.0,
    ) -> WriteResult:
        """Write the baudrate code; the device applies it after a reboot.

        ``save=True`` saves after the write; ``reboot=True`` reboots, switches
        the host port to the new speed and waits for the identity block.
        """
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate not in BAUDRATES:
            raise ValueError(f"Unsupported public Modbus baudrate: {baudrate}")
        _duration(timeout, "timeout")
        with self.bus._lock:
            result = self.write_register(0x04, BAUDRATES.index(baudrate))
            if save:
                self.save_config()
            if reboot:
                self.reboot(timeout=timeout, baudrate=baudrate)
            return result

    def reboot(
        self, *, timeout: float = 5.0, save: bool = False, baudrate: int | None = None
    ) -> DeviceInfo:
        """Save if requested, send reset once and wait until the identity block reads.

        Pass ``baudrate`` when a new device speed takes effect after the reset;
        the shared bus is switched to it before waiting.
        """
        _duration(timeout, "timeout")
        with self.bus._lock:
            if save:
                self.save_config()
            try:
                self._write_ack(0x00, 0xFF)
            except ResponseTimeout:
                pass  # Some builds reset before the echo is sent.
            time.sleep(0.05)
            if baudrate is not None and baudrate != self.bus.baudrate:
                self.bus.reconfigure(baudrate)
            deadline = time.monotonic() + timeout
            last_error: Exception | None = None
            while True:
                try:
                    return self.read_info()
                except (TransportError, ResponseTimeout) as exc:
                    last_error = exc
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ResponseTimeout(
                        f"Node {self.device_id} did not answer within {timeout:g}s after reboot "
                        f"at {self.bus.baudrate} baud"
                    ) from last_error
                time.sleep(min(0.05, remaining))
