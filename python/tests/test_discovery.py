"""Discovery uses real framing and fake serial peers, never local hardware."""

from collections import deque
import logging
from types import SimpleNamespace

import pytest

from hipnuc import (
    DeviceInfo,
    DiscoveryResult,
    DiscoveredDevice,
    ResponseTimeout,
    SerialDevice,
    TransportError,
    discover,
)
import hipnuc.serial_device as implementation
from test_serial_device import Clock, FakeSerial, IDENTITY, frame


@pytest.fixture
def clock(monkeypatch):
    value = Clock()
    monkeypatch.setattr(implementation.time, "monotonic", value.monotonic)
    monkeypatch.setattr(implementation.time, "sleep", value.sleep)
    monkeypatch.setattr(implementation.list_ports, "comports", lambda: [])
    return value


def test_construction_and_explicit_connection_do_not_discover(monkeypatch):
    calls = []
    monkeypatch.setattr(
        implementation,
        "discover",
        lambda *a, **k: pytest.fail("explicit connection must not discover"),
    )
    monkeypatch.setattr(
        implementation.serial, "Serial", lambda *a, **k: calls.append((a, k)) or FakeSerial(*a, **k)
    )
    unopened = SerialDevice()
    device = SerialDevice("Exact/Case", 9600)
    assert calls == [] and unopened.port is None and unopened.baudrate is None
    assert not unopened.is_open
    with device:
        assert device.is_open
    assert calls[0][0] == ("Exact/Case", 9600)


def test_library_discovery_uses_standard_logging_without_printing(
    monkeypatch, clock, caplog, capsys
):
    monkeypatch.setattr(
        implementation.serial,
        "Serial",
        lambda *a, **k: FakeSerial(*a, responses={"LOG VERSION": IDENTITY}, **k),
    )
    with caplog.at_level(logging.INFO, logger="hipnuc.serial_device"):
        result = discover(["FAKE"], baudrates=(115200,))
    assert result.devices[0].info.product_name == "HI14"
    assert "Checking FAKE at 115200 baud" in caplog.text
    assert "found HI14" in caplog.text
    assert capsys.readouterr() == ("", "")


@pytest.mark.parametrize("port,baudrate", [(None, None), ("Exact/Case", None), (None, 9600)])
def test_automatic_open_fills_only_missing_arguments(monkeypatch, port, baudrate):
    info = DeviceInfo(product_name="HI14", firmware_version="1.7.2")
    result = DiscoveryResult([DiscoveredDevice("Exact/Case", 9600, info)], {}, True)
    calls = []
    monkeypatch.setattr(implementation, "discover", lambda *a, **k: calls.append((a, k)) or result)
    monkeypatch.setattr(implementation.serial, "Serial", FakeSerial)
    with SerialDevice(port, baudrate, scan_timeout=4) as device:
        assert (device.port, device.baudrate) == ("Exact/Case", 9600)
        assert device.info is info and device.discovery_result is result
    assert calls[0][0] == (([port] if port else None),)
    assert calls[0][1]["baudrates"] == ((baudrate,) if baudrate else implementation.BAUDRATES)
    assert calls[0][1]["scan_timeout"] == 4


@pytest.mark.parametrize(
    "result,reason",
    [
        (DiscoveryResult([], {}, True), "No serial ports found"),
        (DiscoveryResult([], {"COM3": "Access denied"}, True), "Access denied"),
        (
            DiscoveryResult(
                [DiscoveredDevice("COM3", 9600), DiscoveredDevice("COM4", 115200)], {}, True
            ),
            "Multiple.*COM3.*COM4",
        ),
        (DiscoveryResult([DiscoveredDevice("COM3", 9600)], {}, False), "incomplete.*COM3"),
    ],
)
def test_ambiguous_missing_or_incomplete_results_never_open(monkeypatch, result, reason):
    monkeypatch.setattr(implementation, "discover", lambda *a, **k: result)
    monkeypatch.setattr(
        implementation.serial, "Serial", lambda *a, **k: pytest.fail("must not open")
    )
    with pytest.raises(TransportError, match=reason):
        SerialDevice().open()


def test_find_real_identity_at_later_baud_and_close_all_handles(monkeypatch, clock):
    opened = []

    def factory(name, baud, **kwargs):
        port = FakeSerial(name, baud, responses={"LOG VERSION": IDENTITY if baud == 9600 else b""})
        if baud == 115200:
            # USB buffers can retain a valid frame from the previous host rate.
            port.pending.append(frame())
        opened.append(port)
        return port

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    result = discover(["Exact/Case"], baudrates=(115200, 9600), timeout=2)
    assert result.complete and result.errors == {}
    assert result.devices[0].baudrate == 9600
    assert result.devices[0].info.product_name == "HI14"
    assert all(not port.is_open for port in opened)
    assert opened[0].writes == [b"LOG VERSION\r\n"] * 2
    assert opened[1].writes == [b"LOG VERSION\r\n"]


@pytest.mark.parametrize("garbage", [b"garbage", b"\x00noise"])
def test_wrong_baud_residual_line_is_cleared_by_first_query_then_identity_retry_succeeds(
    monkeypatch, clock, garbage
):
    class LineBufferedPeer:
        """Model line parsing; wrong-baud bytes are representative corruption."""

        def __init__(self):
            self.line = bytearray()
            self.executed = []
            self.opened = []

        def receive(self, data, port):
            for byte in data:
                if byte > 127 or len(self.line) >= 127:
                    self.line.clear()
                elif byte in (10, 13):
                    command = bytes(self.line).split(b"\0", 1)[0].strip()
                    self.line.clear()
                    if command.upper() == b"LOG VERSION":
                        self.executed.append(command)
                        port.pending.append(IDENTITY)
                    # Empty and unknown command lines receive no response.
                else:
                    self.line.append(byte)

        def open(self, name, baud, **kwargs):
            peer = self

            class LineBufferedSerial(FakeSerial):
                @property
                def in_waiting(self):
                    return super().in_waiting or (len(frame()) if self.baudrate == 921600 else 0)

                def read(self, size):
                    if not self.pending and self.baudrate == 921600:
                        self.pending.append(frame())
                    clock.sleep(0.001)
                    return super().read(size)

                def write(self, data):
                    self.writes.append(data)
                    peer.receive(data if self.baudrate == 921600 else garbage, self)
                    return len(data)

            port = LineBufferedSerial(name, baud, **kwargs)
            self.opened.append(port)
            return port

    peer = LineBufferedPeer()
    monkeypatch.setattr(implementation.serial, "Serial", peer.open)
    result = discover(["FAKE"], baudrates=(115200, 921600, 460800), timeout=0.1)
    assert result.complete and result.errors == {}
    assert len(result.devices) == 1
    device = result.devices[0]
    assert device.baudrate == 921600
    assert device.protocol == "hipnuc_binary"
    assert device.info.product_name == "HI14"
    assert device.identity_error is None
    assert [port.baudrate for port in peer.opened] == [115200, 921600]
    assert peer.opened[0].writes == [b"LOG VERSION\r\n"]
    assert peer.opened[1].writes == [b"LOG VERSION\r\n"] * 2
    assert peer.executed == [b"LOG VERSION"]
    assert all(not port.is_open for port in peer.opened)


def test_late_fragmented_identity_has_default_two_second_window(monkeypatch, clock):
    class LateReply(FakeSerial):
        def write(self, data):
            self.writes.append(data)
            self.events = deque([(clock.now + 0.9, IDENTITY[:-4]), (clock.now + 1.1, b"OK\r\n")])
            return len(data)

        @property
        def in_waiting(self):
            while getattr(self, "events", None) and self.events[0][0] <= clock.now:
                self.pending.append(self.events.popleft()[1])
            return super().in_waiting

    monkeypatch.setattr(implementation.serial, "Serial", LateReply)
    result = discover(["COM3"], baudrates=(9600,))
    assert result.devices[0].info.firmware_version == "1.7.2"
    assert 1.1 < clock.now < 2.1


def test_valid_binary_stream_is_readable_without_identity(monkeypatch, clock):
    port = FakeSerial()
    port.pending.append(frame() * 2)
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    result = discover(["COM3"], baudrates=(9600,))
    found = result.devices[0]
    assert result.complete and result.errors == {}
    assert found.info is None and found.protocol == "hipnuc_binary"
    assert "No OK reply" in found.identity_error
    assert port.writes == [b"LOG VERSION\r\n"] * 2
    assert not port.is_open


@pytest.mark.parametrize(
    "data", [b"noise", b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n"]
)
def test_noise_and_generic_nmea_do_not_identify_brand(monkeypatch, clock, data):
    port = FakeSerial()
    port.pending.append(data)
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    result = discover(["COM3"], baudrates=(9600,), timeout=0.1)
    assert not result.devices
    assert "Received bytes" in result.errors["COM3"]
    assert port.writes == [b"LOG VERSION\r\n"]
    assert not port.is_open


def test_silent_peer_does_not_retry_identity(monkeypatch, clock):
    port = FakeSerial()
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    result = discover(["FAKE"], baudrates=(921600,), timeout=0.1)
    assert result.complete and result.devices == []
    assert "No bytes received" in result.errors["FAKE"]
    assert port.writes == [b"LOG VERSION\r\n"]
    assert not port.is_open


@pytest.mark.parametrize(
    "response,reason",
    [(b"ERROR: denied\r\n", "denied"), (b"OK\r\n", "no usable device identity")],
)
def test_device_error_is_not_retried_even_with_valid_binary(monkeypatch, clock, response, reason):
    port = FakeSerial(responses={"LOG VERSION": response})
    port.pending.append(frame())
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    result = discover(["FAKE"], baudrates=(921600,), timeout=0.1)
    assert result.complete and result.errors == {}
    assert result.devices[0].info is None
    assert reason in result.devices[0].identity_error
    assert port.writes == [b"LOG VERSION\r\n"]
    assert not port.is_open


@pytest.mark.parametrize("budget,attempts", [(0.14, 1), (0.2, 2)])
def test_identity_retry_uses_only_remaining_scan_budget(monkeypatch, clock, budget, attempts):
    class DelayedRetry(FakeSerial):
        def __init__(self):
            super().__init__()
            self.pending.append(frame())
            self.reply_at = None

        def write(self, data):
            self.writes.append(data)
            if len(self.writes) == 2:
                self.reply_at = clock.now + 0.08
            return len(data)

        @property
        def in_waiting(self):
            if self.reply_at is not None and clock.now >= self.reply_at:
                self.pending.append(IDENTITY)
                self.reply_at = None
            return super().in_waiting

    port = DelayedRetry()
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    result = discover(["FAKE"], baudrates=(921600,), timeout=0.1, scan_timeout=budget)
    assert not result.complete
    assert result.devices[0].protocol == "hipnuc_binary"
    assert result.devices[0].info is None
    assert "FAKE" in result.errors
    assert port.writes == [b"LOG VERSION\r\n"] * attempts
    assert clock.now == pytest.approx(budget)
    assert not port.is_open


def test_ctrl_c_during_identity_retry_closes_the_same_port(monkeypatch, clock):
    class InterruptedRetry(FakeSerial):
        def write(self, data):
            self.writes.append(data)
            if len(self.writes) == 2:
                raise KeyboardInterrupt
            return len(data)

    port = InterruptedRetry()
    port.pending.append(frame())
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    with pytest.raises(KeyboardInterrupt):
        discover(["FAKE"], baudrates=(921600,), timeout=0.1)
    assert port.writes == [b"LOG VERSION\r\n"] * 2
    assert not port.is_open


def test_scan_budget_and_incomplete_state_include_found_candidates(monkeypatch, clock):
    opened = []

    def factory(name, baud, **kwargs):
        port = FakeSerial(name, baud, responses={"LOG VERSION": IDENTITY if name == "A" else b""})
        opened.append(port)
        return port

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    result = discover(["A", "B", "C"], baudrates=(9600,), scan_timeout=0.2)
    assert not result.complete and result.devices[0].port == "A"
    assert "B" in result.errors
    assert clock.now == pytest.approx(0.2)
    assert all(not port.is_open for port in opened)


def test_occupied_port_is_reported_once_not_retried_at_each_baud(monkeypatch, clock):
    attempts = []

    def factory(*args, **kwargs):
        attempts.append(args)
        raise implementation.serial.SerialException("Access denied: port is already open")

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    result = discover(["COM3"])
    assert len(attempts) == 1 and result.complete
    assert "Access denied" in result.errors["COM3"]


def test_discovery_ctrl_c_closes_the_port(monkeypatch, clock):
    port = FakeSerial()

    def interrupted(_data):
        raise KeyboardInterrupt

    port.write = interrupted
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    with pytest.raises(KeyboardInterrupt):
        discover(["COM3"], baudrates=(9600,))
    assert not port.is_open


def test_all_enumerated_ports_are_checked_before_auto_selection(monkeypatch, clock):
    monkeypatch.setattr(
        implementation.list_ports,
        "comports",
        lambda: [SimpleNamespace(device=p) for p in ("A", "B")],
    )
    monkeypatch.setattr(
        implementation.serial,
        "Serial",
        lambda *a, **k: FakeSerial(*a, responses={"LOG VERSION": IDENTITY}, **k),
    )
    result = discover(baudrates=(115200,))
    assert result.complete and [device.port for device in result.devices] == ["A", "B"]


def test_iter_samples_reports_idle_and_distinguishes_bytes_without_frames(monkeypatch, clock):
    port = FakeSerial()
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    with SerialDevice("COM3", 9600, timeout=0.01) as device:
        port.pending.append(frame())
        samples = device.iter_samples()
        assert next(samples).type == "HI91"
        with pytest.raises(ResponseTimeout, match="No bytes received"):
            next(samples)
        port.pending.append(b"not a measurement")
        with pytest.raises(ResponseTimeout, match="bytes but no valid measurement"):
            device.read()


def test_discovery_send_time_is_included_in_scan_budget(monkeypatch, clock):
    class SlowWrite(FakeSerial):
        def write(self, data):
            clock.sleep(0.04)
            return super().write(data)

    monkeypatch.setattr(implementation.serial, "Serial", SlowWrite)
    result = discover(["COM3"], baudrates=(9600, 115200), scan_timeout=0.1)
    assert not result.complete
    assert clock.now == pytest.approx(0.1)
