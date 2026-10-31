"""Read an application image before opening a device or erasing flash."""

from binascii import crc32
from dataclasses import dataclass
from pathlib import Path
import struct

_MAX_IMAGE_SIZE = 64 * 1024 * 1024
_MARKER = 0x0FFEB6B6
_MARKER_SCAN = 100 * 1024  # The bootloader looks for the marker this far.
_HEADER_MAGIC = 0xFEEDA5A5
_META_MAGIC = 0x41504D44
_DEBUG_CRC = 0x55AA1234


@dataclass(frozen=True)
class FirmwareImage:
    """``version`` and ``build`` are ``None`` when the image does not record them."""

    address: int
    data: bytes
    version: str | None = None
    build: str | None = None
    debug: bool = False


def load_image(path: str | Path) -> FirmwareImage:
    """Load a HiPNUC application from Intel HEX and check its header and CRC."""
    image = read_hex(path)
    data = image.data
    for marker in range(0, min(len(data), _MARKER_SCAN) - 7, 4):
        if struct.unpack_from("<I", data, marker)[0] == _MARKER:
            break
    else:
        raise ValueError("Not a HiPNUC application image: image marker not found")
    header = struct.unpack_from("<I", data, marker + 4)[0] - image.address
    if not 0 <= header <= len(data) - 24 or struct.unpack_from("<I", data, header)[0] != (
        _HEADER_MAGIC
    ):
        raise ValueError("Not a HiPNUC application image: invalid image header")
    legacy_version, length, checksum = struct.unpack_from("<III", data, header + 8)
    debug = checksum == _DEBUG_CRC
    if not debug:
        if not header + 20 <= length <= len(data):
            raise ValueError("Image header length does not match the file")
        # The checksum covers the image except its own four bytes.
        if crc32(data[header + 20 : length], crc32(data[: header + 16])) != checksum:
            raise ValueError("Image CRC mismatch: the file is damaged or incomplete")
    version = build = None
    meta = header + 24
    if len(data) >= meta + 24 and struct.unpack_from("<I", data, meta)[0] == _META_MAGIC:
        raw, date = struct.unpack_from("<II", data, meta + 8)
        version = _version(raw)
        if date:
            build = f"20{date // 10000:02d}-{date // 100 % 100:02d}-{date % 100:02d}"
    elif 0 < legacy_version <= 999:
        version = _version(legacy_version)
    return FirmwareImage(image.address, data, version, build, debug)


def _version(raw: int) -> str | None:
    return f"{raw // 100}.{raw // 10 % 10}.{raw % 10}" if raw else None


def read_hex(path: str | Path) -> FirmwareImage:
    """Parse strict Intel HEX into one span, filling gaps with FF."""
    origin = None
    upper = 0
    data = bytearray()
    ended = False
    try:
        text = Path(path).read_bytes().decode("ascii")
    except UnicodeDecodeError:
        raise ValueError(f"{Path(path).name} is not an Intel HEX file") from None
    for line_number, line in enumerate(text.split("\n"), 1):
        line = line.rstrip("\r\n")
        if not line:
            continue
        error = f"Invalid Intel HEX at line {line_number}"
        if ended:
            raise ValueError(f"{error}: data after end-of-file record")
        if not line.startswith(":") or len(line) > 521:
            raise ValueError(error)
        # bytes.fromhex alone also accepts embedded whitespace.
        digits = line[1:]
        if len(digits) % 2 or any(c not in "0123456789abcdefABCDEF" for c in digits):
            raise ValueError(error)
        record = bytes.fromhex(digits)
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"{error}: record length mismatch")
        if sum(record) & 0xFF:
            raise ValueError(f"{error}: checksum mismatch")
        count, kind = record[0], record[3]
        address = int.from_bytes(record[1:3], "big")
        payload = record[4:-1]
        if kind == 0:
            if not count:
                continue
            absolute = upper + address
            if absolute + count > 0x100000000:
                raise ValueError(f"{error}: address overflow")
            if origin is None:
                origin = absolute
            offset = absolute - origin
            if offset < len(data):
                raise ValueError(f"{error}: records must be ascending and not overlap")
            if offset + count > _MAX_IMAGE_SIZE:
                raise ValueError("Firmware address span exceeds 64 MiB")
            data.extend(b"\xff" * (offset - len(data)))
            data.extend(payload)
        elif kind == 1:
            if count or address:
                raise ValueError(f"{error}: malformed end-of-file record")
            ended = True
        elif kind in (2, 4):
            if count != 2 or address:
                raise ValueError(f"{error}: malformed extended address")
            upper = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind in (3, 5):
            if count != 4 or address:
                raise ValueError(f"{error}: malformed start address")
            # The bootloader starts the application using its vector table.
        else:
            raise ValueError(f"{error}: unsupported record type {kind:02X}")
    if not ended:
        raise ValueError("Intel HEX is missing its end-of-file record")
    if origin is None or not data:
        raise ValueError("Firmware contains no data")
    return FirmwareImage(origin, bytes(data))
