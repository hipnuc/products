"""Exercise pySerial's real POSIX backend with a virtual serial peer."""

import os
import select
import threading
import time

import pytest

from hipnuc import ResponseTimeout, SerialDevice, TransportError
from test_serial_device import frame

pytestmark = pytest.mark.skipif(os.name != "posix", reason="POSIX pseudoterminal backend")


def test_real_posix_port_mixed_stream_and_fragmented_command():
    master, slave = os.openpty()
    stop = threading.Event()
    errors = []

    def peer():
        pending = bytearray()
        try:
            while not stop.is_set():
                ready, _, _ = select.select([master], [], [], 0.05)
                if not ready:
                    continue
                pending.extend(os.read(master, 4096))
                if b"LOG VERSION\r\n" in pending:
                    os.write(master, frame() + b"PNAME=HI14\r\nAPP_VER=172\r\nUUID=PTY01\r\nO")
                    time.sleep(0.02)
                    os.write(master, b"K\r\n")
                    pending.clear()
        except OSError as exc:
            if not stop.is_set():
                errors.append(exc)

    thread = threading.Thread(target=peer, daemon=True)
    thread.start()
    try:
        seen, raw = [], []
        with SerialDevice(
            os.ttyname(slave), 115200, timeout=1, raw_sink=raw.append, sample_sink=seen.append
        ) as device:
            info = device.read_info()
            assert info.product_name == "HI14"
            assert info.firmware_version == "1.7.2"
            assert info.serial_number == "PTY01"
            assert device.read().type == "HI91"
            assert len(seen) == 1
            assert b"".join(raw).startswith(frame())
            with pytest.raises(ResponseTimeout):
                device.read(0.02)
        assert not device.is_open
        with pytest.raises(TransportError):
            device.read()
    finally:
        stop.set()
        thread.join(timeout=2)
        os.close(master)
        os.close(slave)
    assert not thread.is_alive()
    assert not errors
