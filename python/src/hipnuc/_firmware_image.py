"""Read an application image before opening a device or erasing flash."""

from dataclasses import dataclass
from pathlib import Path

_MAX_IMAGE_SIZE = 64 * 1024 * 1024


@dataclass(frozen=True)
class FirmwareImage:
    address: int
    data: bytes


def load_image(path: str | Path, *, raw_binary: bool = False) -> FirmwareImage:
    """Load strict Intel HEX, or explicitly requested binary; fill HEX gaps with FF."""
    path = Path(path)
    if raw_binary:
        with path.open("rb") as source:
            data = source.read(_MAX_IMAGE_SIZE + 1)
        if not data or len(data) > _MAX_IMAGE_SIZE:
            raise ValueError("Firmware must contain 1 byte to 64 MiB of data")
        return FirmwareImage(0, data)

    origin = None
    upper = 0
    data = bytearray()
    ended = False
    with path.open("r", encoding="ascii") as source:
        for line_number, line in enumerate(source, 1):
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
