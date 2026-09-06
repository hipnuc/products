"""Pure byte-stream tests using independent CRC and public golden vectors."""

from __future__ import annotations

import binascii
from datetime import date, datetime, time, timezone
import json
import logging
import math
from pathlib import Path
import struct

import pytest

from hipnuc import Decoder
from hipnuc.models import CommandResult, DeviceInfo, Sample


FIXTURES = json.loads(
    (Path(__file__).parent / "fixtures" / "hipnuc_protocol.json").read_text(encoding="utf-8")
)["frames"]
VECTORS = {entry["name"]: entry for entry in FIXTURES}
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


@pytest.mark.parametrize("entry", FIXTURES, ids=lambda item: item["name"])
def test_fixed_vectors_and_independent_crc(entry):
    raw = bytes.fromhex(entry["raw_hex"])
    assert len(raw) == entry["payload_length"] + 6
    assert binascii.crc_hqx(raw[:4] + raw[6:], 0) == int.from_bytes(raw[4:6], "little")
    samples = Decoder().feed(raw)
    assert len(samples) == 1
    sample = samples[0]
    assert sample.type == entry["protocol"]
    assert sample.raw == raw
    assert sample.complete is entry["complete"]
    assert sample.received_time_ns is None
    assert_values(sample.to_dict(), entry["expected"])
    json.dumps(sample.to_dict(include_raw=True), allow_nan=False)


@pytest.mark.parametrize("name", [entry["name"] for entry in FIXTURES])
def test_every_binary_split_point(name):
    raw = vector(name)
    expected = Decoder().feed(raw)[0].to_dict()
    for split in range(len(raw) + 1):
        decoder = Decoder()
        samples = decoder.feed(raw[:split]) + decoder.feed(raw[split:])
        assert [sample.to_dict() for sample in samples] == [expected], split
        assert decoder.buffered_bytes == 0


@pytest.mark.parametrize("field", ["day", "month", "year"])
@pytest.mark.parametrize("value", ["999999999999999999999999", "0"])
def test_invalid_zda_date_keeps_following_binary_sample(field, value):
    parts = {"day": "06", "month": "09", "year": "2026"}
    parts[field] = value
    raw = nmea(f"GPZDA,123456.789,{parts['day']},{parts['month']},{parts['year']},00,00")
    decoder = Decoder()

    samples = decoder.feed(raw + vector())

    assert [sample.type for sample in samples] == ["ZDA", "HI91"]
    assert samples[0].values["utc"] is None
    assert samples[0].issues == ("invalid_utc_date",)
    assert samples[0].raw == raw
    assert samples[1].to_dict() == Decoder().feed(vector())[0].to_dict()
    assert decoder.buffered_bytes == 0


@pytest.mark.parametrize("chunk_size", [1, 2, 7, 64, 1024])
def test_mixed_stream_keeps_sample_order_and_ascii_responses(chunk_size):
    raw = b"LOG VERSION\r\n" + GGA + vector() + RMC + vector("hi83_legacy_ms") + b"OK\r\n"
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
    assert [sample.type for sample in samples] == ["UNKNOWN"]
    assert samples[0].complete is False
    assert samples[0].issues == ("unknown_payload_tag:0x99",)
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


def test_multiple_known_payloads_and_atomic_truncation():
    decoder = Decoder()
    samples = decoder.feed(frame(vector()[6:] + vector("hi81_si")[6:]))
    assert [sample.type for sample in samples] == ["HI91", "HI81"]
    assert [sample.metadata["payload_offset"] for sample in samples] == [0, 76]
    assert decoder.feed(frame(vector()[6:] + b"\x81\x00")) == []
    assert decoder.statistics["malformed_packets"] == 1


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


def test_hi83_time_layouts_and_unknown_bits_preserve_only_reliable_prefix():
    current = Decoder().feed(vector("hi83_current"))[0]
    legacy = Decoder().feed(vector("hi83_legacy_ms"))[0]
    assert current.metadata["hi83_time_layout"] == "uint64_us"
    assert current.values["device_time_s"] == pytest.approx(424242.424242)
    assert legacy.metadata["hi83_time_layout"] == "legacy_uint32_ms"
    assert legacy.values["device_time_s"] == pytest.approx(123.456)
    assert legacy.values["pressure_pa"] == 101325
    partial = Decoder().feed(vector("hi83_unknown_prefix"))[0]
    assert not partial.complete
    assert "gnss_longitude_deg" not in partial.values
    assert partial.metadata["undecoded_payload_offset"] == 20
    assert partial.metadata["undecoded_payload_hex"].startswith("91756e6b6e6f776e")


def test_hi83_unknown_bitmap_does_not_guess_legacy_time_width():
    payload = (
        struct.pack("<BHBI3fI", 0x83, 0, 0, (1 << 0) | (1 << 5) | (1 << 20), 1, 2, 3, 123456)
        + b"unknown"
    )
    sample = Decoder().feed(frame(payload))[0]
    assert sample.values["acceleration_m_s2"] == [1, 2, 3]
    assert not any(key.startswith("device_time") for key in sample.values)
    assert sample.complete is False
    assert "ambiguous_hi83_time_layout" in sample.issues


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
    assert empty.values == {"main_status": 0, "status_ext": 0, "data_bitmap": 0}


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


def test_sxt_explicit_units_and_json_datetime():
    sample = Decoder().feed(
        nmea(
            "GNSXT,20260906123456.789,112.1,28.1,123.4,90,10,90,12.3,-20,4,4,22,18,180,-90,45,1,2,3,3,1"
        )
    )[0]
    assert sample.type == "SXT"
    assert sample.to_dict()["utc"] == "2026-09-06T12:34:56.789000Z"
    assert sample.values["angular_velocity_rad_s"] == pytest.approx(
        [math.pi, -math.pi / 2, math.pi / 4]
    )
    assert sample.values["heading_rad"] == pytest.approx(math.pi / 2)
    assert sample.values["velocity_enu_m_s"] == [1, 2, 3]
    json.dumps(sample.to_dict(), allow_nan=False)


@pytest.mark.parametrize(
    "body,kind,expected",
    [
        (
            "GPVTG,90,T,80,M,10,N,18.52,K,A",
            "VTG",
            {"course_over_ground_rad": math.pi / 2, "speed_over_ground_m_s": 1852 / 360},
        ),
        (
            "GPGSA,A,3,04,05,,,,,,,,,,,1.8,1.0,1.5",
            "GSA",
            {"fix_type": 3, "pdop": 1.8, "hdop": 1.0, "vdop": 1.5},
        ),
        (
            "GPGSV,1,1,01,04,45,180,42,1",
            "GSV",
            {"message_count": 1, "message_number": 1, "satellites_in_view": 1, "signal_id": 1},
        ),
        ("GPZDA,123456.789,06,09,2026,00,00", "ZDA", {"utc": "2026-09-06T12:34:56.789000Z"}),
    ],
)
def test_additional_standard_nmea_sentences(body, kind, expected):
    sample = Decoder().feed(nmea(body))[0]
    assert sample.type == kind
    assert_values(sample.to_dict(), expected)
    if kind == "GSV":
        assert sample.values["satellites"] == [
            {"id": "04", "elevation_rad": math.pi / 4, "azimuth_rad": math.pi, "snr_db": 42}
        ]


def test_truncated_gsv_satellite_group_is_rejected():
    decoder = Decoder()
    assert decoder.feed(nmea("GPGSV,1,1,01,04,45")) == []
    assert decoder.statistics["nmea_errors"] == 1


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
