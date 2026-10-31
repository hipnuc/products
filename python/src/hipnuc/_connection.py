"""Small connection helpers shared by discovery, the CLI and Modbus."""

from __future__ import annotations

from collections import Counter
import errno
import os
import re
import sys

import serial

from .errors import TransportError

# pySerial can raise native backend errors; unrelated programming errors must
# remain visible instead of becoming misleading port-access diagnostics.
SERIAL_OPEN_ERRORS = (OSError, ValueError, NotImplementedError)
try:
    import termios
except ImportError:
    pass
else:
    SERIAL_OPEN_ERRORS += (termios.error,)


def reset_line_coding(connection: serial.Serial) -> None:
    """Send the line settings to the adapter, as Windows does on every open.

    Linux cdc_acm sends them only when they change. A CH347 reopened at the
    same baudrate then keeps the input queued while the port was closed and
    delivers everything seconds late for the whole session, or holds back one
    byte until more data follows. Switching away and back sends them; input
    that arrived before is stale and is discarded.
    """
    if os.name == "posix":
        rate = connection.baudrate
        connection.baudrate = 9600 if rate != 9600 else 19200
        connection.baudrate = rate
        connection.reset_input_buffer()


def is_usb_port(port: object) -> bool:
    """Recognize USB metadata, with conventional Linux/macOS name fallbacks."""
    if getattr(port, "vid", None) is not None or getattr(port, "pid", None) is not None:
        return True
    if re.search(r"\bUSB\b", getattr(port, "hwid", "") or "", re.IGNORECASE):
        return True
    return bool(
        re.fullmatch(
            r"/dev/(?:tty(?:USB|ACM)\d+|(?:cu|tty)\.usb(?:serial|modem)[^/]*)",
            getattr(port, "device", ""),
        )
    )


def _error_kind(error: BaseException | str) -> str:
    """Prefer numeric errors, including wrapped pySerial/termios exceptions."""
    messages = []
    seen = set()
    while error is not None and id(error) not in seen:
        seen.add(id(error))
        messages.append(str(error))
        code = getattr(error, "errno", None)
        if code is None and isinstance(error, BaseException) and error.args:
            code = error.args[0] if isinstance(error.args[0], int) else None
        if code is None:
            match = re.search(r"\bErrno\s+(\d+)\b", str(error), re.IGNORECASE)
            code = int(match[1]) if match else None
        winerror = getattr(error, "winerror", None)
        if winerror is None:
            match = re.search(r"\bWinError\s+(\d+)\b", str(error), re.IGNORECASE)
            winerror = int(match[1]) if match else None
        if code in (errno.EACCES, errno.EPERM) or winerror == 5:
            return "permission"
        # pySerial reports another process's exclusive lock as EAGAIN.
        if code in (errno.EBUSY, errno.EAGAIN) or winerror == 32:
            return "busy"
        if code in (errno.ENOENT, errno.ENODEV) or winerror in (2, 3):
            return "missing"
        if code == errno.EINVAL or winerror == 87:
            return "baudrate"
        error = getattr(error, "__cause__", None) or getattr(error, "__context__", None)
    text = " ".join(messages).lower()
    if "permission denied" in text or "access denied" in text or "access is denied" in text:
        return "permission"
    if "busy" in text or "already open" in text:
        return "busy"
    if "no such file" in text or "filenotfounderror" in text:
        return "missing"
    if "baud" in text and ("not supported" in text or "unsupported" in text):
        return "baudrate"
    return "other"


def is_baudrate_error(error: BaseException) -> bool:
    """True for an invalid serial setting; another candidate rate may work."""
    return _error_kind(error) == "baudrate"


def _permission_hint() -> str:
    if sys.platform.startswith("linux"):
        return (
            "Check serial-port permissions (usually dialout on Ubuntu/Debian); "
            "log in again after changing group membership."
        )
    return "Check access permissions and close CHCenter or other programs using the port."


def open_error(port: str, error: BaseException | None = None) -> TransportError:
    """Preserve the reported failure and add guidance without guessing a driver."""
    kind = _error_kind(error) if error is not None else "other"
    if kind == "permission":
        hint = _permission_hint()
    elif kind == "busy":
        hint = "Close CHCenter or other programs using this port."
        if sys.platform.startswith("linux"):
            hint += (
                " Just after plugging in, ModemManager may hold it for a few seconds; "
                "wait and retry, or check with: sudo fuser -v " + port
            )
    elif kind == "missing":
        hint = "The port does not exist; check its name (hihost list --all)."
    elif kind == "baudrate":
        hint = "The adapter or driver rejected the serial settings; try a supported baudrate."
    else:
        hint = "Check the port name, access permissions and other programs using it."
    detail = f": {error}." if error is not None else "."
    return TransportError(f"Cannot open {port}{detail} {hint}")


def discovery_error_summary(errors: dict[str, str]) -> str:
    """Summarize failed ports; the discovery result retains their full errors."""
    if not errors:
        return (
            "No USB serial ports found. Check the USB connection (including VM passthrough). "
            "Specify any other port, such as an on-board UART, by name (-p in the CLI)."
        )
    counts = Counter(_error_kind(error) for error in errors.values())
    labels = {
        "permission": "permission denied",
        "busy": "in use",
        "missing": "missing/disconnected",
        "baudrate": "unsupported serial settings",
        "other": "no usable response",
    }
    summary = ", ".join(f"{count} {labels[kind]}" for kind, count in counts.items())
    protocol = (
        " Protocol settings cannot be checked until the port opens."
        if all(kind in ("permission", "busy", "missing") for kind in counts)
        else ""
    )
    return f"{len(errors)} ports checked: {summary}.{protocol}"
