"""CAN wire examples use independent product-unit expectations, not C output."""

from collections import deque
from datetime import datetime, timezone
import json
import math
import struct
import subprocess
import sys

import pytest

from hipnuc.can import decode_message, make_trigger, read_register, write_register
from hipnuc.errors import DeviceError, ResponseTimeout, TransportError, VerificationError
import hipnuc.can as api

can = pytest.importorskip("can")


def message(pgn, data, node=0xE8, **kwargs):
    return can.Message(arbitration_id=0x0C000000 | (pgn << 8) | node, data=data, **kwargs)


@pytest.mark.parametrize(
    "pgn,raw,key,expected",
    [
        (0xFF34, bytes.fromhex("00 08 00 F8 00 04 00 00"), "acceleration_m_s2", (9.8, -9.8, 4.9)),
        (
            0xFF37,
            bytes.fromhex("00 40 00 C0 00 00 00 00"),
            "angular_velocity_rad_s",
            (math.radians(1000), -math.radians(1000), 0),
        ),
        (
            0xFF3A,
            bytes.fromhex("00 40 00 C0 00 00 00 00"),
            "magnetic_field_t",
            (0.0005, -0.0005, 0),
        ),
        (0xFF46, struct.pack("<4h", 10000, 0, -5000, 0), "quaternion_wxyz", (1, 0, -0.5, 0)),
        (
            0xFF4A,
            struct.pack("<2i", 90000, -45000),
            "inclination_rad",
            (math.pi / 2, -math.pi / 4),
        ),
        (0xFF26, struct.pack("<4h", 300, -400, 50, 500), "velocity_enu_m_s", (3, -4, 0.5)),
    ],
)
def test_classic_physical_units(pgn, raw, key, expected):
    sample = decode_message(message(pgn, raw, timestamp=123.25))
    assert sample.values[key] == pytest.approx(expected)
    assert sample.values["node_id"] == 232  # Full address, never & 0x7F.
    assert sample.raw == raw
    assert sample.received_time_ns == 123_250_000_000
    assert sample.metadata["pgn"] == pgn
    assert sample.metadata["protocol"] == "j1939"
    assert sample.complete


def test_partial_euler_fields_never_merge_or_relabel_heading():
    rp = decode_message(message(0xFF3D, struct.pack("<2i", 90000, -45000)))
    assert rp.euler_rad == (math.pi / 2, -math.pi / 4, None)
    assert rp.heading_rad is None
    heading = decode_message(message(0xFF41, struct.pack("<i", 270000)))
    assert heading.heading_rad == math.pi * 1.5
    assert heading.euler_rad is None
    yaw = decode_message(message(0xFF41, struct.pack("<2i", 270000, 90000)))
    assert yaw.euler_rad == (None, None, math.pi / 2)
    assert yaw.roll_rad is None
    assert yaw.heading_rad == math.pi * 1.5


def test_ins_position_and_raw_gnss_quality_remain_independent():
    position = decode_message(message(0xFF10, struct.pack("<2i", 312500000, 1215000000)))
    assert (position.latitude_deg, position.longitude_deg) == (31.25, 121.5)
    assert position.altitude_msl_m is None
    assert "position_quality" not in position.values
    assert position.metadata["position_source"] == "ins"
    altitude = decode_message(message(0xFF14, struct.pack("<ihh", 12345, -125, 250)))
    assert altitude.latitude_deg is None
    assert altitude.altitude_msl_m == 123.45
    assert altitude.geoid_separation_m == -1.25
    assert altitude.values["differential_age_s"] == 2.5
    quality = decode_message(message(0xFF18, bytes([4, 5, 18, 16, 3, 0, 0, 0])))
    assert quality.values == {
        "node_id": 232,
        "position_quality": 4,
        "heading_quality": 5,
        "position_satellites": 18,
        "heading_satellites": 16,
        "ins_status": 3,
        "ins_status_name": "navigating",
    }
    assert quality.metadata["quality_source"] == "raw_gnss"


def test_temperature_reserved_bytes_are_not_pressure():
    sample = decode_message(message(0xFF43, struct.pack("<hhi", -1234, 0, 999)))
    assert sample.temperature_c == -12.34
    assert sample.pressure_pa is None


def test_classic_utc_is_calendar_time_without_invented_host_stamp():
    sample = decode_message(message(0xFF2F, struct.pack("<6BH", 24, 2, 29, 12, 30, 59, 999)))
    assert sample.values["utc"] == datetime(2024, 2, 29, 12, 30, 59, 999000, timezone.utc)
    assert sample.received_time_ns is None
    assert sample.to_dict()["utc"] == "2024-02-29T12:30:59.999000Z"


@pytest.mark.parametrize(
    "parts",
    [
        (0, 0, 0, 12, 1, 1, 500),
        (25, 2, 29, 0, 0, 0, 0),
        (24, 2, 29, 0, 0, 60, 0),
        (24, 2, 29, 0, 0, 1, 1000),
    ],
)
def test_invalid_utc_does_not_invent_date_or_carry_milliseconds(parts):
    sample = decode_message(message(0xFF2F, struct.pack("<6BH", *parts)))
    assert sample.values["utc"] is None
    assert "invalid_utc" in sample.issues


FD_FIELDS = {
    0: struct.pack("<3f", 1, 2, -3),
    1: struct.pack("<3f", 0.5, 1, 2),
    2: struct.pack("<3f", 100, 200, -300),
    3: struct.pack("<3f", 90, -45, 180),
    4: struct.pack("<4f", 1, 0, 0, 0),
    5: struct.pack("<Q", 0x100000001),
    6: struct.pack("<5BHx", 24, 2, 29, 1, 2, 30500),
    8: struct.pack("<f", 25.5),
}


def fd_frame(bitmap, status=0):
    raw = struct.pack("<IHBB", bitmap, status, 3, 254)
    raw += b"".join(data for bit, data in FD_FIELDS.items() if bitmap & (1 << bit))
    padded = next(size for size in (12, 16, 20, 24, 32, 48, 64) if size >= len(raw))
    return message(0xFF5B, raw.ljust(padded, b"\x00"), is_fd=True, bitrate_switch=True)


FD_BITMAPS = [
    bitmap
    for bitmap in range(1, 0x180)
    if not bitmap & 0x80
    and 8 + sum(len(data) for bit, data in FD_FIELDS.items() if bitmap & (1 << bit)) <= 64
]


@pytest.mark.parametrize("bitmap", FD_BITMAPS)
def test_canfd83_all_supported_fitting_bitmaps(bitmap):
    sample = decode_message(fd_frame(bitmap))
    assert sample.type == "CANFD83"
    assert sample.metadata["sequence"] == 254
    assert sample.values["node_id"] == 232
    expected = {
        0: ("acceleration_m_s2", [1, 2, -3]),
        1: ("angular_velocity_rad_s", [0.5, 1, 2]),
        2: ("magnetic_field_t", [0.0001, 0.0002, -0.0003]),
        3: ("euler_rad", [math.pi / 2, -math.pi / 4, math.pi]),
        4: ("quaternion_wxyz", [1, 0, 0, 0]),
        5: ("device_time_us", 0x100000001),
        6: ("utc", datetime(2024, 2, 29, 1, 2, 30, 500000, timezone.utc)),
        8: ("temperature_c", 25.5),
    }
    for bit, (key, value) in expected.items():
        if bitmap & (1 << bit):
            assert sample.values[key] == (value if bit == 6 else pytest.approx(value))
        else:
            assert key not in sample.values


def test_canfd83_status_and_nonfinite_components():
    sample = decode_message(fd_frame(1 << 6, status=(1 << 7) | (1 << 11)))
    assert sample.values["utc"] is None
    assert sample.values["status_flags"] == ["ATT_CONV", "UTC_UNSYNC"]
    assert sample.issues == ("utc_unsynchronized",)
    raw = struct.pack("<IHBB3f", 1, 0, 0, 1, math.nan, math.inf, -2)
    sample = decode_message(message(0xFF5B, raw, is_fd=True))
    assert sample.acceleration_m_s2 == (None, None, -2)
    assert len(sample.issues) == 2
    json.dumps(sample.to_dict(include_raw=True), allow_nan=False)


@pytest.mark.parametrize(
    "frame",
    [
        message(0xFF34, bytes(5)),
        message(0xFF34, bytes(9), is_fd=True),
        message(0xFF34, bytes(8), dlc=7),
        message(0xFF34, bytes(8), is_remote_frame=True),
        message(0xFF34, bytes(8), is_error_frame=True),
        message(0xFF5B, struct.pack("<IHBBI", 1 << 8, 0, 0, 0, 0)),
        message(0xFF5B, struct.pack("<IHBBI", 0, 0, 0, 0, 0), is_fd=True),
        message(0xFF5B, struct.pack("<IHBBI", 1 << 7, 0, 0, 0, 0), is_fd=True),
        message(0xFF5B, struct.pack("<IHBBI", 1, 0, 0, 0, 0), is_fd=True),
        message(0xFF5B, struct.pack("<IHBBI", 1 << 8, 0, 0, 0, 0) + b"x", is_fd=True),
        message(0xFF5B, struct.pack("<IHBB", 0x17F, 0, 0, 0) + bytes(56), is_fd=True),
    ],
)
def test_known_invalid_frame_is_distinct_from_unrelated(frame):
    with pytest.raises(ValueError):
        decode_message(frame)


@pytest.mark.parametrize(
    "frame",
    [
        can.Message(arbitration_id=0x123, data=b"", is_extended_id=False),
        message(0xFE01, bytes(3), is_error_frame=True),
        message(0xEF55, bytes(8)),
        message(0x1FF34, bytes(8)),
    ],
)
def test_unrelated_frames_return_none(frame):
    assert decode_message(frame) is None


class FakeBus:
    def __init__(self, replies=()):
        self.replies = deque(replies)
        self.sent = []
        self.waits = []

    def send(self, request, timeout=None):
        self.sent.append((request, timeout))

    def recv(self, timeout=None):
        self.waits.append(timeout)
        return self.replies.popleft() if self.replies else None


def reply(address=0x1234, command=3, value=0x12345678, node=0xE8, status=0, **kwargs):
    return can.Message(
        arbitration_id=0x0CEF5500 | node,
        data=struct.pack("<HBBI", address, command, status, value),
        **kwargs,
    )


def test_register_read_exact_wire_and_full_reply_match():
    other_destination = reply()
    other_destination.arbitration_id = 0x0CEF56E8
    other_priority = reply()
    other_priority.arbitration_id = 0x18EF55E8
    bus = FakeBus(
        [
            reply(node=0x68),
            reply(address=0x1235),
            reply(command=6),
            other_destination,
            other_priority,
            reply(is_rx=False),
            reply(is_fd=True),
            reply(is_error_frame=True),
            reply(is_remote_frame=True),
            reply(dlc=7),
            reply(),
        ]
    )
    assert read_register(bus, 232, 0x1234) == 0x12345678
    request, timeout = bus.sent[0]
    assert request.arbitration_id == 0x0CEFE855
    assert bytes(request.data) == bytes.fromhex("34 12 03 00 01 00 00 00")
    assert request.is_extended_id and not request.is_fd
    assert timeout == 2
    assert len(bus.sent) == 1


def test_register_write_returns_none_after_matching_echo():
    bus = FakeBus([reply(command=6, value=0xAABBCCDD)])
    assert write_register(bus, 232, 0x1234, 0xAABBCCDD) is None
    assert bytes(bus.sent[0][0].data) == bytes.fromhex("34 12 06 00 DD CC BB AA")


def test_already_queued_timestamped_reply_is_not_this_request(monkeypatch):
    monkeypatch.setattr(api.time, "time", lambda: 100.0)
    bus = FakeBus([reply(value=1, timestamp=99), reply(value=2, timestamp=100.1)])
    assert read_register(bus, 232, 0x1234) == 2


def test_register_rejection_and_wrong_echo_are_not_success():
    bus = FakeBus([reply(status=7)])
    with pytest.raises(DeviceError) as failure:
        read_register(bus, 232, 0x1234)
    assert failure.value.code == 7
    assert failure.value.response == bytes(reply(status=7).data).hex()
    with pytest.raises(VerificationError):
        write_register(FakeBus([reply(command=6, value=1)]), 232, 0x1234, 2)


@pytest.mark.parametrize("write", [False, True])
def test_silent_rejection_or_lost_ack_is_timeout_without_retry(write):
    bus = FakeBus()
    with pytest.raises(ResponseTimeout, match="may not support"):
        if write:
            write_register(bus, 232, 0x1234, 1)
        else:
            read_register(bus, 232, 0x1234)
    assert len(bus.sent) == 1


def test_unrelated_traffic_does_not_restart_deadline(monkeypatch):
    now = 0.0
    monkeypatch.setattr(api.time, "monotonic", lambda: now)
    bus = FakeBus()

    def traffic(timeout=None):
        nonlocal now
        bus.waits.append(timeout)
        now += 0.75
        return reply(node=1)

    bus.recv = traffic
    with pytest.raises(ResponseTimeout):
        read_register(bus, 232, 0x1234, timeout=2)
    assert bus.waits == [2, 1.25, 0.5]
    assert len(bus.sent) == 1


def test_late_matching_response_does_not_extend_total_timeout(monkeypatch):
    now = 0.0
    monkeypatch.setattr(api.time, "monotonic", lambda: now)
    bus = FakeBus()

    def delayed_response(timeout=None):
        nonlocal now
        now = 3
        return reply()

    bus.recv = delayed_response
    with pytest.raises(ResponseTimeout):
        read_register(bus, 232, 0x1234, timeout=2)


@pytest.mark.parametrize("operation", ["send", "recv"])
def test_can_transport_errors_preserve_cause(operation):
    bus = FakeBus()
    original = can.CanOperationError("adapter removed")

    def fail(*args, **kwargs):
        raise original

    setattr(bus, operation, fail)
    with pytest.raises(TransportError) as failure:
        read_register(bus, 232, 0x1234)
    assert failure.value.__cause__ is original


@pytest.mark.parametrize("node", [-1, 0x55, 254, 255, 256, True, 2.5])
def test_invalid_target_never_sends(node):
    bus = FakeBus()
    with pytest.raises(ValueError):
        read_register(bus, node, 0x1234)
    assert not bus.sent


@pytest.mark.parametrize("timeout", [0, -1, math.nan, math.inf])
def test_invalid_timeout_never_sends(timeout):
    bus = FakeBus()
    with pytest.raises(ValueError):
        read_register(bus, 232, 0x1234, timeout=timeout)
    assert not bus.sent


def test_trigger_is_plain_message_without_transaction():
    request = make_trigger(232, 0xFF34)
    assert request.arbitration_id == 0x0CEFE855
    assert bytes(request.data) == bytes.fromhex("96 00 06 00 34 FF 00 00")
    with pytest.raises(ValueError):
        make_trigger(232, 0xEF00)


def test_real_python_can_virtual_transport(monkeypatch):
    with can.Bus(interface="virtual", channel="hipnuc-reg-test") as host:
        with can.Bus(interface="virtual", channel="hipnuc-reg-test") as device:
            host_send = host.send

            def exchange(request, timeout=None):
                host_send(request, timeout=timeout)
                received = device.recv(timeout=0.1)
                assert received.arbitration_id == 0x0CEFE855
                assert bytes(received.data) == bytes.fromhex("34 12 03 00 01 00 00 00")
                device.send(reply(value=42))

            monkeypatch.setattr(host, "send", exchange)
            assert read_register(host, 232, 0x1234, timeout=0.5) == 42


def test_importing_can_api_does_not_require_optional_dependency():
    script = """
import sys
class NoCan:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'can':
            raise ModuleNotFoundError('can intentionally unavailable')
sys.meta_path.insert(0, NoCan())
import hipnuc
from hipnuc.can import decode_message
assert 'can' not in sys.modules
"""
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
