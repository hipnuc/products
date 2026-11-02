"""Housekeeping after a verified serial update."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import logging
import threading

from .errors import HipnucError, ResponseTimeout

_prompt = logging.getLogger("hipnuc.update")  # The CLI shows this logger's warnings.
_RESTART_S = 10.0  # As long as the wait for the updated application.
_local = threading.local()


class _Hush(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:
        return not getattr(_local, "active", False)


# Logger filters apply whatever level an application sets on this logger.
logging.getLogger("hipnuc.serial_device").addFilter(_Hush())


@contextmanager
def _quiet() -> Iterator[None]:
    """Keep this housekeeping out of the SDK's debug output, in this thread only."""
    _local.active = True
    try:
        yield
    finally:
        _local.active = False


def _cleared(text: str) -> bool:
    """Whether the reported triple is all zero."""
    tokens = text.split()
    for index, token in enumerate(tokens):
        if token.startswith("GYRB="):
            rest = token[len("GYRB=") :]
            values = (([rest] if rest else []) + tokens[index + 1 :])[:3]
            try:
                return len(values) == 3 and not any(float(value) for value in values)
            except ValueError:
                return False
    return False


def finish(port: str, baudrate: int) -> None:
    """Run once after a verified update.

    Quiet unless it needs the device kept still, or a restart it started does
    not return; that raises ResponseTimeout, since the device is then no longer
    known to run.
    """
    from .serial_device import SerialDevice

    restarted = False
    with _quiet():
        try:
            with SerialDevice(port, baudrate, timeout=1.0) as device:
                if not _cleared(device.command("LOG IMUCONFIG", timeout=1.0).text):
                    return
                _prompt.warning("Completing the update; keep the module still...")
                device.command("FCAL WB AUTO 2", timeout=6.0)
                restarted = True  # Applied at the next start.
                device.reboot(recovery_timeout=_RESTART_S)
        except (HipnucError, OSError, ValueError) as error:
            if restarted:
                raise ResponseTimeout(
                    f"The device did not answer on {port} after the update; reconnect and "
                    "read its identity to confirm the new firmware"
                ) from error
