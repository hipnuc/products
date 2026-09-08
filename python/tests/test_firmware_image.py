"""Independent Intel HEX record cases; no firmware or hardware required."""

import pytest

from hipnuc._firmware_image import load_image


def record(kind, address=0, data=b""):
    body = bytes((len(data), address >> 8, address & 255, kind)) + data
    return ":" + (body + bytes((-sum(body) & 255,))).hex().upper()


def write_hex(tmp_path, records):
    path = tmp_path / "image.hex"
    path.write_text("\n".join(records), encoding="ascii")
    return path


def test_extended_linear_address_gaps_and_start_records(tmp_path):
    path = write_hex(
        tmp_path,
        [
            ":020000040800F2",
            ":024000000102BB",
            record(0, 0x4004, b"\x03"),
            record(5, data=b"\x08\x00\x40\x01"),
            ":00000001FF",
            "",
        ],
    )
    image = load_image(path)
    assert image.address == 0x08004000
    assert image.data == b"\x01\x02\xff\xff\x03"


def test_extended_segment_address(tmp_path):
    image = load_image(
        write_hex(
            tmp_path,
            [
                record(2, data=b"\x10\x00"),
                record(0, 0x20, b"X"),
                record(1),
            ],
        )
    )
    assert image.address == 0x10020
    assert image.data == b"X"


@pytest.mark.parametrize(
    "records,reason",
    [
        ([":014000005866", record(1)], "checksum"),
        ([":024000005867", record(1)], "length"),
        ([record(0, 0x4000, b"XX")], "end-of-file"),
        ([record(1)], "no data"),
        ([record(0, 0x4000, b"XX"), record(0, 0x4001, b"Y"), record(1)], "ascending"),
        ([record(0, 0x4000, b"XX"), record(1), record(0, 0x4002, b"Y")], "after end"),
        ([record(1, data=b"X")], "malformed"),
        ([record(2, 1, b"\x00\x00")], "extended"),
        ([record(4, data=b"X")], "extended"),
        ([record(5, data=b"X")], "start address"),
        ([record(6)], "unsupported"),
        ([": 00000001FF"], "Invalid"),
        ([record(4, data=b"\xff\xff"), record(0, 0xFFFF, b"XY")], "overflow"),
        ([record(0, 0, b"X"), record(4, data=b"\x04\x00"), record(0, 0, b"Y")], "64 MiB"),
    ],
)
def test_malformed_hex(tmp_path, records, reason):
    with pytest.raises(ValueError, match=reason):
        load_image(write_hex(tmp_path, records))


def test_binary_requires_explicit_selection_and_nonempty_file(tmp_path):
    path = tmp_path / "image.bin"
    path.write_bytes(b"data")
    assert load_image(path, raw_binary=True).data == b"data"
    with pytest.raises(ValueError):
        load_image(path)
    path.write_bytes(b"")
    with pytest.raises(ValueError):
        load_image(path, raw_binary=True)


def test_binary_span_is_bounded(tmp_path, monkeypatch):
    monkeypatch.setattr("hipnuc._firmware_image._MAX_IMAGE_SIZE", 4)
    path = tmp_path / "image.bin"
    path.write_bytes(b"12345")
    with pytest.raises(ValueError):
        load_image(path, raw_binary=True)
