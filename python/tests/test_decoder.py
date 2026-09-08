"""Pure byte-stream tests using an independent CRC and hand-computed vectors."""

from __future__ import annotations

import binascii
from datetime import date, datetime, time, timezone
import json
import logging
import math
import struct

import pytest

from hipnuc import Decoder
from hipnuc.models import CommandResult, DeviceInfo, Sample


# Synthetic protocol vectors. Expected values were computed by hand from the
# published field layouts, independently of the decoder.
VECTORS = {
    "hi91_si": {
        "protocol": "HI91",
        "complete": True,
        "raw_hex": (
            "5aa54c0050959108081980e6c54740e201000000803f000000bf0000803e000034430000b4c200003442"
            "000020410000a0c10000f041000020410000a0c10000f0410000803f000000000000000000000000"
        ),
        "expected": {
            "acceleration_m_s2": [9.8, -4.9, 2.45],
            "angular_velocity_rad_s": [3.141592653589793, -1.5707963267948966, 0.7853981633974483],
            "device_time_ms": 123456,
            "euler_rad": [0.17453292519943295, -0.3490658503988659, 0.5235987755982988],
            "magnetic_field_t": [1e-05, -2e-05, 3e-05],
            "main_status": 2056,
            "pressure_pa": 101325,
            "quaternion_wxyz": [1, 0, 0, 0],
            "temperature_c": 25,
        },
    },
    "hi81_si": {
        "protocol": "HI81",
        "complete": True,
        "raw_hex": (
            "5aa56800062181000803600915cd5b07ffffe8030cfefa00640038ff0008640038ff2c01d007d204fb1a"
            "09060c22d5dde80330f82823102700000000000087aed442874cc31040e201000f0804160412022efb01"
            "7b0038fe1503c800d4fe900191aabbccddeeff0083deadbeef91"
        ),
        "expected": {
            "acceleration_m_s2": [0.48828, -0.97656, 9.9999744],
            "altitude_msl_m": 123.456,
            "angular_velocity_rad_s": [1, -0.5, 0.25],
            "geoid_separation_m": -12.34,
            "gps_time_of_week_ms": 123456789,
            "gps_week": 2400,
            "heading_rad": 1.5707963267948966,
            "ins_status": 3,
            "latitude_deg": 28.1234567,
            "longitude_deg": 112.1234567,
            "magnetic_field_t": [3.0517e-06, -6.1034e-06, 9.1551e-06],
            "main_status": 2048,
            "odometer_speed_m_s": 12.34,
            "pitch_rad": -0.3490658503988659,
            "pressure_pa": 102000,
            "quaternion_wxyz": [1, 0, 0, 0],
            "roll_rad": 0.17453292519943295,
            "temperature_c": -5,
            "utc": "2026-09-06T12:34:56.789000Z",
            "velocity_enu_m_s": [1.23, -4.56, 7.89],
        },
    },
    "hi83_current": {
        "protocol": "HI83",
        "complete": True,
        "raw_hex": (
            "5aa5ec0034e483000003ffff0fc00ae81c410ae89cc0000000000000803f000000bf0000803e00002041"
            "0000a0c10000f041000020410000a0c10000f0410000803f000000000000000000000000b2a9d1c66200"
            "00001a09060c22d5dd0080e6c5470000c4410000a0400000c0c00000f0410000803f0000004000004040"
            "cdcccc3dcdcc4c3e9a99993e0000803f0000004000004040cdcccc3dcdcc4cbe9a99993e373eeeb6e607"
            "5c40daf8b8db9a1f3c4077be9f1a2fdd5e4004160412a4704541a47045c1000000400800000066666666"
            "66065c409a99999999193c400000000000c05e40000080400000a0400000c040"
        ),
        "expected": {
            "acceleration_m_s2": [9.80665, -4.903325, 0],
            "altitude_msl_m": 123.456,
            "angular_velocity_rad_s": [1, -0.5, 0.25],
            "data_bitmap": 3222274047,
            "device_time_us": 424242424242,
            "gnss_velocity_enu_m_s": [4, 5, 6],
            "inclination_rad": [0.08726646259971647, -0.10471975511965978],
            "inclination_yaw_rad": 0.5235987755982988,
            "ins_status": 3,
            "latitude_deg": 28.1234567,
            "longitude_deg": 112.1234567,
            "magnetic_field_t": [1e-05, -2e-05, 3e-05],
            "main_status": 0,
            "node_id": 8,
            "pressure_pa": 101325,
            "quaternion_wxyz": [1, 0, 0, 0],
            "temperature_c": 24.5,
            "utc": "2026-09-06T12:34:56.789000Z",
            "velocity_enu_m_s": [1, 2, 3],
        },
    },
    "hi83_unknown_prefix": {
        "protocol": "HI83",
        "complete": False,
        "raw_hex": (
            "5aa53400a9c083000000010010400000803f000000400000404091756e6b6e6f776e0000000000005c40"
            "0000000000003c400000000000005940"
        ),
        "expected": {"acceleration_m_s2": [1, 2, 3], "data_bitmap": 1074790401, "main_status": 0},
    },
}
GGA = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"
RMC = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n"


def vector(name="hi91_si"):
    return bytes.fromhex(VECTORS[name]["raw_hex"])


def frame(payload):
    """An independent CRC oracle; never call the production CRC function."""
    header = b"\x5a\xa5" + struct.pack("<H", len(payload))
    return header + struct.pack("<H", binascii.crc_hqx(header + payload, 0)) + payload


def nmea(body):
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return f"${body}*{checksum:02X}\r\n".encode("ascii")


def assert_values(actual, expected):
    for key, value in expected.items():
        if isinstance(value, (int, float, list)):
            assert actual[key] == pytest.approx(value, rel=1e-6, abs=1e-12), key
        else:
            assert actual[key] == value, key


@pytest.mark.parametrize("entry", VECTORS.values(), ids=list(VECTORS))
def test_fixed_vectors_and_independent_crc(entry):
    raw = bytes.fromhex(entry["raw_hex"])
    assert len(raw) == int.from_bytes(raw[2:4], "little") + 6
    assert binascii.crc_hqx(raw[:4] + raw[6:], 0) == int.from_bytes(raw[4:6], "little")
    samples = Decoder().feed(raw)
    if not entry["complete"]:
        assert samples == []
        return
    assert len(samples) == 1
    sample = samples[0]
    assert sample.type == entry["protocol"]
    assert sample.raw == raw
    assert sample.complete is entry["complete"]
    assert sample.received_time_ns is None
    assert_values(sample.to_dict(), entry["expected"])
    json.dumps(sample.to_dict(include_raw=True), allow_nan=False)


@pytest.mark.parametrize("name", list(VECTORS))
def test_every_binary_split_point(name):
    raw = vector(name)
    expected = [sample.to_dict() for sample in Decoder().feed(raw)]
    for split in range(len(raw) + 1):
        decoder = Decoder()
        samples = decoder.feed(raw[:split]) + decoder.feed(raw[split:])
        assert [sample.to_dict() for sample in samples] == expected, split
        assert decoder.buffered_bytes == 0


def test_unsupported_sentence_keeps_following_binary_sample():
    raw = nmea("GPZDA,123456.789,06,09,2026,00,00")
    decoder = Decoder()

    samples = decoder.feed(raw + vector())

    assert [sample.type for sample in samples] == ["ZDA", "HI91"]
    assert samples[0].complete is False
    assert samples[0].issues == ("unsupported_nmea_sentence",)
    assert samples[0].raw == raw
    assert samples[1].to_dict() == Decoder().feed(vector())[0].to_dict()
    assert decoder.buffered_bytes == 0


@pytest.mark.parametrize("chunk_size", [1, 2, 7, 64, 1024])
def test_mixed_stream_keeps_sample_order_and_ascii_responses(chunk_size):
    raw = b"LOG VERSION\r\n" + GGA + vector() + RMC + vector("hi83_current") + b"OK\r\n"
    decoder = Decoder()
    samples = []
    for offset in range(0, len(raw), chunk_size):
        samples.extend(decoder.feed(raw[offset : offset + chunk_size]))
    assert [sample.type for sample in samples] == ["GGA", "HI91", "RMC", "HI83"]
    assert decoder.drain_lines() == ["LOG VERSION", "OK"]
    assert decoder.drain_lines() == []
    assert decoder.statistics["samples"] == 4


def test_corrupt_crc_resynchronizes_without_emitting_embedded_ascii_or_nmea():
    bad = bytearray(frame(b"\x99\r\nOK\r\n" + GGA))
    bad[4] ^= 0x80
    decoder = Decoder()
    samples = decoder.feed(bytes(bad) + vector())
    assert [sample.type for sample in samples] == ["HI91"]
    assert decoder.drain_lines() == []
    assert decoder.statistics["crc_errors"] == 1


def test_valid_binary_payload_cannot_emit_ascii_or_nmea():
    decoder = Decoder()
    samples = decoder.feed(frame(b"\x99\r\nOK\r\n" + GGA))
    assert samples == []
    assert decoder.statistics["malformed_packets"] == 1
    assert decoder.drain_lines() == []


def test_nmea_checksum_reestablishes_boundary_after_damaged_binary():
    for raw in (b"\x5a\xa5\xff\xff\x00\x00\r\nOK\r\n", vector()[:4] + b"\x00\x00" + vector()[6:]):
        decoder = Decoder()
        samples = decoder.feed(raw + GGA + b"OK\r\n")
        assert [sample.type for sample in samples] == ["GGA"]
        assert decoder.drain_lines() == ["OK"]


def test_corrupted_short_length_cannot_turn_residual_payload_into_ack():
    raw = b"\x5a\xa5\x01\x00\x00\x00\x91\r\nOK\r\n"
    decoder = Decoder()
    assert decoder.feed(raw) == []
    assert decoder.drain_lines() == []
    decoder.reset()
    decoder.feed(b"OK\r\n")
    assert decoder.drain_lines() == ["OK"]


@pytest.mark.parametrize("length", [0, 507, 65535])
def test_invalid_length_recovers_at_next_binary_frame(length):
    decoder = Decoder()
    corrupt = b"\x5a\xa5" + struct.pack("<H", length) + b"\x00\x00\r\nOK\r\n"
    assert [sample.type for sample in decoder.feed(corrupt + vector())] == ["HI91"]
    assert decoder.drain_lines() == []
    assert decoder.statistics["length_errors"] == 1


def test_incomplete_plausible_length_recovers_binary_suffix_at_eof():
    decoder = Decoder()
    corrupt = b"\x5a\xa5\xf4\x01\x00\x00\x99\r\nOK\r\n"
    assert decoder.feed(corrupt + vector()) == []
    assert [sample.type for sample in decoder.finish()] == ["HI91"]
    assert decoder.buffered_bytes == 0
    assert decoder.drain_lines() == []


def test_deleted_byte_does_not_lose_following_frame():
    decoder = Decoder()
    damaged = vector()[:20] + vector()[21:]
    samples = decoder.feed(damaged + vector("hi81_si"))
    assert [sample.type for sample in samples] == ["HI81"]
    assert decoder.statistics["crc_errors"] >= 1


def test_garbage_and_overlong_lines_have_bounded_storage():
    decoder = Decoder(max_line_size=80, max_pending_lines=2)
    assert decoder.feed(b"\xff" * 1000000) == []
    assert decoder.buffered_bytes <= decoder.max_line_size + decoder.max_payload_size + 6
    assert decoder.feed(b"$" + b"A" * 100000) == []
    assert decoder.buffered_bytes <= decoder.max_line_size + decoder.max_payload_size + 6
    assert [sample.type for sample in decoder.feed(vector())] == ["HI91"]
    decoder.feed(b"\nA\nB\nC\n")
    assert decoder.drain_lines() == ["B", "C"]
    assert decoder.statistics["lines_dropped"] >= 1


def test_nmea_without_newline_resynchronizes_on_binary_or_new_sentence():
    decoder = Decoder()
    assert [
        sample.type for sample in decoder.feed(b"$GPGGA,broken" + vector() + b"$broken" + GGA)
    ] == ["HI91", "GGA"]
    assert decoder.drain_lines() == []


@pytest.mark.parametrize("split_sync", [False, True])
def test_overlong_nmea_preserves_split_binary_sync(split_sync):
    decoder = Decoder()
    raw = vector()
    prefix = b"$" + b"A" * (decoder.max_line_size - 2)
    if split_sync:
        assert decoder.feed(prefix + raw[:1]) == []
        samples = decoder.feed(raw[1:])
    else:
        samples = decoder.feed(prefix + raw)
    assert [sample.raw for sample in samples] == [raw]
    assert decoder.statistics["nmea_errors"] == 1
    assert decoder.buffered_bytes == 0
    assert decoder.drain_lines() == []


def test_reset_discards_partial_frame_and_response_lines():
    decoder = Decoder()
    decoder.feed(b"OK\n" + vector()[:20])
    before = decoder.statistics["bytes_received"]
    decoder.reset()
    assert decoder.buffered_bytes == 0
    assert decoder.drain_lines() == []
    assert decoder.statistics["bytes_received"] == before
    assert len(decoder.feed(vector())) == 1


def test_drain_lines_can_discard_stale_half_line_without_resetting_frame():
    decoder = Decoder()
    decoder.feed(b"old\nO")
    assert decoder.drain_lines(discard_partial=True) == ["old"]
    decoder.feed(b"K\n")
    assert decoder.drain_lines() == ["K"]
    decoder.feed(vector()[:20])
    decoder.drain_lines(discard_partial=True)
    assert [sample.type for sample in decoder.feed(vector()[20:])] == ["HI91"]
    decoder.feed(GGA[:20])
    decoder.drain_lines(discard_partial=True)
    assert [sample.type for sample in decoder.feed(GGA[20:])] == ["GGA"]


def test_multiple_payloads_are_rejected_and_next_frame_recovers():
    decoder = Decoder()
    samples = decoder.feed(frame(vector()[6:] + vector("hi81_si")[6:]))
    assert samples == []
    assert decoder.feed(frame(vector()[6:] + b"\x81\x00")) == []
    assert decoder.statistics["malformed_packets"] == 2
    assert [sample.type for sample in decoder.feed(vector())] == ["HI91"]


def test_hi81_reserved_bytes_pressure_and_heading_semantics():
    sample = Decoder().feed(vector("hi81_si"))[0]
    assert sample.values["pressure_pa"] == 102000
    assert sample.values["heading_rad"] == pytest.approx(math.pi / 2)
    assert "euler_rad" not in sample.values
    assert not any(key.startswith("gnss_") for key in sample.values)
    assert sample.metadata["reserved_tail_hex"] == "91aabbccddeeff0083deadbeef91"
    assert sample.metadata["heading_reference"] == "north_clockwise"
    assert sample.metadata["navigation_frame"] == "device_configured"


@pytest.mark.parametrize(
    "parts,issue",
    [
        (b"\x00" * 7, "utc_unavailable"),
        (struct.pack("<5BH", 26, 2, 30, 12, 1, 1000), "invalid_utc"),
        (struct.pack("<5BH", 26, 9, 6, 12, 1, 60000), "invalid_utc"),
    ],
)
def test_hi81_invalid_utc_never_invents_a_date(parts, issue):
    payload = bytearray(vector("hi81_si")[6:])
    payload[35:42] = parts
    sample = Decoder().feed(frame(payload))[0]
    assert sample.values["utc"] is None
    assert issue in sample.issues


def test_hi83_time_and_unknown_layout_rejection():
    current = Decoder().feed(vector("hi83_current"))[0]
    assert current.values["device_time_s"] == pytest.approx(424242.424242)
    assert current.metadata["device_time_reference"] == "local_counter"
    decoder = Decoder()
    assert decoder.feed(vector("hi83_unknown_prefix")) == []
    assert decoder.statistics["malformed_packets"] == 1
    assert [sample.type for sample in decoder.feed(vector())] == ["HI91"]


def test_hi83_legacy_millisecond_layout_is_rejected():
    payload = struct.pack("<BHBI3fI", 0x83, 0, 0, (1 << 0) | (1 << 5), 1, 2, 3, 123456)
    decoder = Decoder()
    assert decoder.feed(frame(payload)) == []
    assert decoder.statistics["malformed_packets"] == 1


def test_hi83_internal_bits_reject_the_frame():
    payload = struct.pack(
        "<BHBI3fQ", 0x83, 0, 0, (1 << 0) | (1 << 5) | (1 << 25), 1, 2, 3, 123456
    ) + bytes(64)
    decoder = Decoder()
    assert decoder.feed(frame(payload)) == []
    assert decoder.statistics["malformed_packets"] == 1


@pytest.mark.parametrize(
    "bit,size",
    [
        (0, 12),
        (1, 12),
        (2, 12),
        (3, 12),
        (4, 16),
        (5, 8),
        (6, 8),
        (7, 4),
        (8, 4),
        (9, 12),
        (10, 12),
        (11, 12),
        (12, 12),
        (13, 12),
        (14, 24),
        (15, 4),
        (16, 4),
        (17, 4),
        (18, 4),
        (19, 4),
        (30, 24),
        (31, 12),
    ],
)
def test_hi83_individual_bitmap_fields_and_truncation(bit, size):
    payload = struct.pack("<BHBI", 0x83, 0, 0, 1 << bit) + bytes(size)
    decoder = Decoder()
    sample = decoder.feed(frame(payload))[0]
    assert sample.complete
    assert decoder.feed(frame(payload[:-1])) == []
    assert decoder.statistics["malformed_packets"] == 1


def test_hi83_zero_bitmap_and_bitmap_change_do_not_retain_previous_fields():
    decoder = Decoder()
    full = decoder.feed(vector("hi83_current"))[0]
    empty = decoder.feed(frame(struct.pack("<BHBI", 0x83, 0, 0, 0)))[0]
    assert full.values["device_time_us"] == 424242424242
    assert empty.values == {
        "main_status": 0,
        "status_flags": [],
        "ins_status": 0,
        "ins_status_name": "invalid",
        "data_bitmap": 0,
    }


def test_hi83_unsynchronized_utc_is_unavailable():
    payload = struct.pack("<BHBI5BHB", 0x83, 1 << 11, 0, 1 << 6, 26, 9, 6, 12, 34, 56789, 0)
    sample = Decoder().feed(frame(payload))[0]
    assert sample.values["utc"] is None
    assert "utc_unsynchronized" in sample.issues


def test_nmea_known_sentences_and_json_dates():
    samples = Decoder().feed(GGA + RMC)
    gga, rmc = [sample.to_dict() for sample in samples]
    assert gga["latitude_deg"] == pytest.approx(48.1173)
    assert gga["longitude_deg"] == pytest.approx(11.5166666667)
    assert gga["utc_time"] == "12:35:19Z"
    assert "utc" not in gga
    assert gga["altitude_msl_m"] == 545.4
    assert gga["fix_valid"] is True
    assert rmc["utc"] == "1994-03-23T12:35:19Z"
    assert rmc["speed_over_ground_m_s"] == pytest.approx(22.4 * 1852 / 3600)
    json.dumps([gga, rmc], allow_nan=False)


def test_nmea_void_fix_retains_diagnostic_coordinates_and_marks_invalid():
    for body in (
        "GPGGA,123519,4807.038,N,01131.000,E,0,00,0.9,545.4,M,46.9,M,,",
        "GPRMC,123519,V,4807.038,N,01131.000,E,0,0,060926,,",
    ):
        sample = Decoder().feed(nmea(body))[0]
        assert sample.values["fix_valid"] is False
        assert sample.values["latitude_deg"] == pytest.approx(48.1173)
        assert "gnss_fix_invalid" in sample.issues


def test_nmea_empty_coordinates_stay_unavailable_and_hemisphere_is_respected():
    sample = Decoder().feed(nmea("GPGGA,,,,,,0,00,,,,,,,"))[0]
    assert sample.values["latitude_deg"] is None
    assert sample.values["longitude_deg"] is None
    assert sample.values["utc_time"] is None
    sample = Decoder().feed(nmea("GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,"))[
        0
    ]
    assert sample.values["latitude_deg"] < 0
    assert sample.values["longitude_deg"] < 0


@pytest.mark.parametrize(
    "raw",
    [
        GGA.replace(b"*47", b"*00"),
        GGA.replace(b"*47", b""),
        b"$GPGGA,123*GG\r\n",
        nmea("GPGGA,123"),
        b"$GPGGA,\xff*00\r\n",
    ],
)
def test_bad_nmea_is_rejected_and_next_frame_survives(raw):
    decoder = Decoder()
    assert [sample.type for sample in decoder.feed(raw + vector())] == ["HI91"]
    assert decoder.statistics["nmea_errors"] == 1
    assert decoder.drain_lines() == []


def test_invalid_nmea_calendar_and_coordinates_are_not_fabricated():
    sample = Decoder().feed(nmea("GPRMC,250000,A,4867.038,N,18131.000,E,0,0,310226,,"))[0]
    assert sample.values["utc"] is None
    assert sample.values["latitude_deg"] is None
    assert sample.values["longitude_deg"] is None
    assert {"invalid_utc_time", "invalid_utc_date", "invalid_latitude", "invalid_longitude"} <= set(
        sample.issues
    )


def test_unsupported_checksum_valid_nmea_is_explicitly_partial():
    sample = Decoder().feed(nmea("GPXYZ,1,2,3"))[0]
    assert sample.type == "XYZ"
    assert sample.complete is False
    assert sample.values["fields"] == ["1", "2", "3"]
    assert sample.issues == ("unsupported_nmea_sentence",)


@pytest.mark.parametrize(
    "options",
    [
        {"max_payload_size": 0},
        {"max_payload_size": 65536},
        {"max_line_size": 0},
        {"max_pending_lines": 0},
    ],
)
def test_decoder_resource_limits_must_be_positive(options):
    with pytest.raises(ValueError):
        Decoder(**options)


def test_nonfinite_binary_values_are_explicitly_unavailable_in_strict_json():
    payload = bytearray(vector()[6:])
    struct.pack_into("<3f", payload, 12, math.nan, math.inf, -math.inf)
    sample = Decoder().feed(frame(payload))[0]
    assert sample.values["acceleration_m_s2"] == [None, None, None]
    assert len([issue for issue in sample.issues if issue.startswith("non_finite:")]) == 3
    json.dumps(sample.to_dict(), allow_nan=False)


def test_models_json_contract_handles_nested_time_and_nonfinite_values():
    sample = Sample(
        "test",
        {
            "value": math.nan,
            "nested": [math.inf, date(2026, 9, 6), time(12, 0, tzinfo=timezone.utc)],
        },
        b"\x00\xff",
        received_time_ns=123,
        metadata={"when": datetime(2026, 9, 6, tzinfo=timezone.utc)},
    )
    record = sample.to_dict(include_raw=True)
    assert record["value"] is None
    assert record["nested"] == [None, "2026-09-06", "12:00:00Z"]
    assert record["metadata"]["when"] == "2026-09-06T00:00:00Z"
    assert record["raw_hex"] == "00ff"
    assert "raw_hex" not in sample.to_dict()
    assert record["received_time_ns"] == 123
    json.dumps(record, allow_nan=False)
    assert (
        DeviceInfo(product_name="HI229", raw_response="VERSION").to_dict()["product_name"]
        == "HI229"
    )
    assert CommandResult("PING", "OK", True).to_dict()["verified"] is None


def test_decoder_does_not_configure_logging_or_accept_text():
    root = logging.getLogger()
    before = list(root.handlers), root.level
    Decoder().feed(vector())
    assert (list(root.handlers), root.level) == before
    with pytest.raises(TypeError):
        Decoder().feed("not bytes")


def test_main_status_flags_and_ins_status_names():
    payload = bytearray(76)
    payload[0] = 0x91
    struct.pack_into("<H", payload, 1, (1 << 7) | (1 << 4) | (1 << 11))
    sample = Decoder().feed(frame(bytes(payload)))[0]
    assert sample.values["status_flags"] == ["MAG_DIST", "ATT_CONV", "UTC_UNSYNC"]
    assert sample.metadata["device_time_reference"] == "local_counter"
    ins = struct.pack("<BHBI", 0x83, 0, 6, 0)
    sample = Decoder().feed(frame(ins))[0]
    assert sample.values["ins_status_name"] == "dead_reckoning"
    assert sample.values["status_flags"] == []
