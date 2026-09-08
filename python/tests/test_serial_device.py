"""Serial transaction tests use real framing over a deterministic fake port."""

import binascii
from collections import deque
import struct
import time

import pytest

from hipnuc import SerialDevice, discover
from hipnuc.errors import DeviceError, ResponseTimeout, TransportError, VerificationError
import hipnuc.serial_device as implementation


def frame(tag=0x91):
    body = bytes([tag]) + bytes(75)
    header = b"\x5a\xa5" + struct.pack("<H", len(body))
    return header + struct.pack("<H", binascii.crc_hqx(header + body, 0)) + body


IDENTITY = b"PNAME=HI14\r\nAPP_VER=172\r\nBL_VER=108\r\nUUID=TEST1234\r\nOK\r\n"


class FakeSerial:
    def __init__(self, *args, responses=None, **kwargs):
        self.is_open = True
        self.baudrate = args[1] if len(args) > 1 else kwargs.get("baudrate", 115200)
        self.timeout = 0.001
        self.write_timeout = 1
        self.pending = deque()
        self.writes = []
        self.responses = responses or {}

    @property
    def in_waiting(self):
        return len(self.pending[0]) if self.pending else 0

    def read(self, size):
        if not self.pending:
            return b""
        chunk = self.pending.popleft()
        if len(chunk) > size:
            self.pending.appendleft(chunk[size:])
        return chunk[:size]

    def write(self, value):
        self.writes.append(value)
        response = self.responses.get(value.decode().strip(), [])
        if callable(response):
            response = response(self)
        if isinstance(response, bytes):
            response = [response]
        self.pending.extend(response)
        return len(value)

    def close(self):
        self.is_open = False


@pytest.fixture
def fake(monkeypatch):
    port = FakeSerial()
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: port)
    return port


def test_interleaved_fragmented_ack_keeps_every_measurement(fake):
    seen, raw = [], []
    fake.responses["LOG VERSION"] = [frame(), b"PNAME=HI14\r\nAPP_VER=172\n", b"O", b"K\r\n"]
    with SerialDevice(
        "FAKE", 115200, timeout=0.02, sample_sink=seen.append, raw_sink=raw.append
    ) as dev:
        info = dev.read_info()
        assert info.product_name == "HI14"
        assert info.firmware_version == "1.7.2"
        sample = dev.read()
        assert sample is seen[0]
        assert sample.received_time_ns > 0
        assert b"".join(raw).startswith(frame())
    assert not fake.is_open
    assert fake.writes == [b"LOG VERSION\r\n"]


def test_commands_and_reads_do_not_reconfigure_open_transport(monkeypatch):
    class FixedSettingsSerial(FakeSerial):
        def __init__(self, *args, **kwargs):
            assert kwargs["timeout"] == 0
            assert kwargs["write_timeout"] == 0.2
            super().__init__(*args, **kwargs)
            self.locked = True
            self.pending.append(frame())
            self.responses["LOG VERSION"] = [IDENTITY[:-4], b"O", b"K\r\n"]

        def __setattr__(self, name, value):
            if name in {"timeout", "write_timeout"} and getattr(self, "locked", False):
                raise AssertionError("open transport settings must remain fixed")
            super().__setattr__(name, value)

    monkeypatch.setattr(implementation.serial, "Serial", FixedSettingsSerial)
    seen = []
    with SerialDevice("FAKE", 115200, timeout=0.2, sample_sink=seen.append) as dev:
        assert dev.command("LOG VERSION", timeout=0.03).acknowledged
        assert dev.read().type == "HI91"
        assert len(seen) == 1
        with pytest.raises(ResponseTimeout, match="No bytes received"):
            dev.read(0.002)


def test_error_does_not_disable_output_and_context_closes(fake):
    fake.responses["BAD"] = b"ERROR: Invalid value\r\n"
    with pytest.raises(DeviceError, match="Invalid value"):
        with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
            dev.command("BAD")
    assert not fake.is_open
    assert fake.writes == [b"BAD\r\n"]


@pytest.mark.parametrize("reply", [b"NOTOK\n", b"BOOK=123\n", b""])
def test_substring_is_not_ack(fake, reply):
    fake.responses["TEST"] = reply
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        with pytest.raises(ResponseTimeout):
            dev.command("TEST")


def test_bad_crc_binary_containing_ok_never_acknowledges(fake):
    packet = bytearray(frame())
    packet[18:22] = b"\nOK\n"
    fake.responses["TEST"] = bytes(packet)
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        with pytest.raises(ResponseTimeout):
            dev.command("TEST")


@pytest.mark.parametrize("command", ["LOG HI91 ONMARK ONCE", "LOG COM1 HI91 ONMARK ONCE"])
def test_once_data_completes_without_ack(fake, command):
    fake.responses[command] = frame()
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        result = dev.command(command)
        assert result.verified is True
        assert result.acknowledged is False
        assert dev.read().type == "HI91"


def test_old_partial_ack_cannot_complete_new_transaction(fake):
    fake.pending.extend([b"O", b"K\n"])
    fake.responses["BAD"] = b"ERROR: rejected\n"
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        with pytest.raises(DeviceError, match="rejected"):
            dev.command("BAD")


def test_error_in_same_chunk_is_not_hidden_by_ok(fake):
    fake.responses["BAD"] = b"OK\nERROR: rejected\n"
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        with pytest.raises(DeviceError, match="rejected"):
            dev.command("BAD")


def test_read_idle_and_closed_are_errors(fake):
    dev = SerialDevice("FAKE", 115200, timeout=0.005)
    with pytest.raises(TransportError):
        dev.read()
    with dev:
        with pytest.raises(ResponseTimeout, match="No bytes received"):
            dev.read()
    with pytest.raises(TransportError):
        dev.command("LOG VERSION")


def test_queue_overflow_is_counted_but_sink_receives_all(fake):
    seen = []
    with SerialDevice("FAKE", 115200, sample_sink=seen.append, queue_size=2) as dev:
        fake.pending.append(frame() * 5)
        assert dev.read() is not None
        assert len(seen) == 5
        assert dev.dropped_samples == 3


def test_raw_configuration_readback_and_save_share_one_session(fake):
    fake.responses.update(
        {
            "CONFIG IMU COORD 2": b"OK\n",
            "LOG USRCONFIG": b"COORD=2\nOK\n",
            "SAVECONFIG": b"OK\n",
        }
    )
    with SerialDevice("FAKE", 115200) as dev:
        assert dev.command("CONFIG IMU COORD 2").acknowledged
        assert "COORD=2" in dev.command("LOG USRCONFIG").text
        assert dev.save_config().acknowledged
    assert fake.writes == [b"CONFIG IMU COORD 2\r\n", b"LOG USRCONFIG\r\n", b"SAVECONFIG\r\n"]


@pytest.mark.parametrize("timeout", [0, -1, float("nan"), float("inf")])
@pytest.mark.parametrize("operation", ["baud", "reboot"])
def test_invalid_recovery_timeout_is_rejected_before_device_io(
    fake, monkeypatch, timeout, operation
):
    def fail(*args, **kwargs):
        raise AssertionError("invalid recovery timeout must not send a command")

    with SerialDevice("FAKE", 115200) as dev:
        monkeypatch.setattr(dev, "command", fail)
        with pytest.raises(ValueError, match="timeout must be finite and positive"):
            if operation == "baud":
                dev.set_baudrate(921600, recovery_timeout=timeout, save=True)
            else:
                dev.reboot(recovery_timeout=timeout)
    assert fake.writes == []


@pytest.mark.parametrize("device_port", [None, "COM2"])
def test_baud_change_without_ack_recovers_same_device(fake, device_port):
    device_baud = [115200]
    fake.responses["LOG VERSION"] = lambda port: IDENTITY if port.baudrate == device_baud[0] else []

    def change(_port):
        device_baud[0] = 921600
        return []

    target = f"{device_port} " if device_port else ""
    fake.responses[f"SERIALCONFIG {target}921600"] = change
    fake.responses["SAVECONFIG"] = b"OK\n"
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        info = dev.set_baudrate(921600, device_port=device_port, recovery_timeout=0.2)
        assert info.serial_number == "TEST1234"
        assert dev.baudrate == 921600
        assert b"SAVECONFIG\r\n" not in fake.writes


def test_probe_uses_exact_explicit_linux_paths_and_closes(monkeypatch):
    ports = []

    def factory(name, baud, **kwargs):
        p = FakeSerial(name, baud, responses={"LOG VERSION": IDENTITY})
        ports.append((name, p))
        return p

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    results = discover(["/dev/serial/by-id/MixedCase"], baudrates=(115200,), timeout=0.01)
    assert results.devices[0].port == "/dev/serial/by-id/MixedCase"
    assert results.complete
    assert not ports[0][1].is_open


def test_discovery_rejects_ok_without_identity(fake):
    fake.responses["LOG VERSION"] = b"OK\n"
    assert discover(["FAKE"], baudrates=(115200,), timeout=0.01).devices == []


def test_write_failure_is_transport_error_and_port_closes(fake, monkeypatch):
    def fail(_data):
        raise implementation.serial.SerialException("unplugged")

    monkeypatch.setattr(fake, "write", fail)
    with pytest.raises(TransportError, match="unplugged"):
        with SerialDevice("FAKE", 115200) as dev:
            dev.command("LOG VERSION")
    assert not fake.is_open


@pytest.mark.parametrize("reply", [b"OK\n", b"PNAME=HI14\nOK\n", b"APP_VER=172\nOK\n"])
def test_read_info_requires_product_and_version_or_serial(fake, reply):
    fake.responses["LOG VERSION"] = reply
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        with pytest.raises(VerificationError, match="usable device identity"):
            dev.read_info()
        assert dev.info is None


def test_read_info_accepts_unknown_version_with_identity(fake):
    fake.responses["LOG VERSION"] = b"PNAME=FUTURE_PRODUCT\nAPP_VER=999\nOK\n"
    with SerialDevice("FAKE", 115200, timeout=0.01) as dev:
        assert dev.read_info().firmware_version == "9.9.9"


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


def test_idle_reads_respect_deadlines_without_blocking_transport(monkeypatch):
    clock = Clock()
    sleeps = []

    def sleep(duration):
        sleeps.append(duration)
        clock.sleep(duration)

    class IdleSerial(FakeSerial):
        def read(self, size):
            raise AssertionError("no transport read is needed without queued bytes")

    monkeypatch.setattr(implementation.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(implementation.time, "sleep", sleep)
    monkeypatch.setattr(implementation.serial, "Serial", IdleSerial)
    with SerialDevice("FAKE", 115200) as dev:
        assert dev._pump(0) == []
        assert sleeps == []
        with pytest.raises(ResponseTimeout, match="No bytes received"):
            dev.read(0.0025)
        assert clock.now == pytest.approx(0.0025)
        assert sleeps == pytest.approx([0.001, 0.001, 0.0005])
        with pytest.raises(ResponseTimeout):
            dev.command("NO_RESPONSE", timeout=0.0025)
        # First command adds its bounded startup observation before response wait.
        assert clock.now == pytest.approx(0.0075)


@pytest.mark.parametrize("baudrate,ready_at", [(115200, 0.014), (4800, 0.2)])
def test_first_command_waits_for_clean_stream_boundary_and_preserves_samples(
    monkeypatch, baudrate, ready_at
):
    clock = Clock()
    written_at = []
    damaged = bytearray(frame())
    damaged[4] ^= 1

    class StartingStream(FakeSerial):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, responses={"LOG VERSION": IDENTITY}, **kwargs)
            self.events = deque(
                [
                    (0.003, bytes(damaged)),
                    (ready_at / 2, frame() + frame()[:8]),
                    (ready_at, frame()[8:]),
                ]
            )

        @property
        def in_waiting(self):
            while self.events and self.events[0][0] <= clock.now:
                self.pending.append(self.events.popleft()[1])
            return super().in_waiting

        def write(self, data):
            written_at.append(clock.now)
            assert not self.events, "do not interrupt an incoming partial frame"
            return super().write(data)

    monkeypatch.setattr(implementation.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(implementation.time, "sleep", clock.sleep)
    monkeypatch.setattr(implementation.serial, "Serial", StartingStream)
    seen, raw = [], []
    with SerialDevice(
        "FAKE", baudrate, timeout=0.5, sample_sink=seen.append, raw_sink=raw.append
    ) as dev:
        assert dev.read_info().product_name == "HI14"
        assert written_at[0] >= ready_at
        assert len(seen) == 2
        assert dev.read() is seen[0]
        assert dev.read() is seen[1]
        assert b"".join(raw).startswith(bytes(damaged) + frame() * 2)
        assert dev.decoder.statistics["crc_errors"] == 1


class DelayedAckSerial(FakeSerial):
    """Schedule old firmware's second OK after its 5-ms handler delay."""

    def __init__(self, clock, baudrate):
        super().__init__("FAKE", baudrate)
        self.clock = clock
        self.events = []

    def _release_events(self):
        while self.events and self.events[0][0] <= self.clock.now:
            _, data = self.events.pop(0)
            self.pending.append(data)

    @property
    def in_waiting(self):
        self._release_events()
        return super().in_waiting

    def read(self, size):
        self._release_events()
        data = super().read(size)
        self.clock.sleep(len(data) * 10 / self.baudrate if data else min(self.timeout, 0.001))
        return data

    def write(self, value):
        self.writes.append(value)
        if value.startswith(b"SERIALCONFIG"):
            self.events.append((self.clock.now, b"OK\r\n"))
            char_time = 10 / self.baudrate
            second_at = self.clock.now + 4 * char_time + 0.005
            self.events.extend(
                (second_at + i * char_time, bytes([byte])) for i, byte in enumerate(b"OK\r\n")
            )
        else:
            # The old OK would arrive before this new command's failure.
            self.events.append((self.clock.now + 0.02, b"ERR\r\n"))
        self.events.sort(key=lambda item: item[0])
        return len(value)


def test_continuous_binary_does_not_extend_ack_cleanup(monkeypatch):
    clock = Clock()

    class StreamingSerial(FakeSerial):
        @property
        def in_waiting(self):
            return super().in_waiting or len(frame())

        def read(self, size):
            if not self.pending:
                self.pending.append(frame())
            data = super().read(size)
            clock.sleep(0.001)
            return data

    fake = StreamingSerial(responses={"LOG ENABLE": b"OK\n"})
    seen = []
    monkeypatch.setattr(implementation.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: fake)
    with SerialDevice("FAKE", 115200, sample_sink=seen.append) as dev:
        before = clock.now
        assert dev.command("LOG ENABLE").acknowledged
        assert clock.now - before < 0.03
        assert seen
        assert len(seen) == dev.decoder.statistics["samples"]


def test_reboot_does_not_query_identity_in_old_five_ms_reset_window(fake, monkeypatch):
    # Older Windows interpreters have a monotonic clock coarser than 5 ms.
    # Verify protocol ordering with deterministic time, not OS tick timing.
    clock = Clock()
    monkeypatch.setattr(implementation.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(implementation.time, "sleep", clock.sleep)
    reset_at = []
    queries = []

    def identify(_port):
        if reset_at:
            queries.append(time.monotonic() - reset_at[0])
        return IDENTITY

    def reset(_port):
        reset_at.append(time.monotonic())
        return b"OK\n"

    fake.responses.update({"LOG VERSION": identify, "REBOOT": reset})
    with SerialDevice("FAKE", 115200, timeout=0.02) as dev:
        assert dev.reboot(recovery_timeout=0.1).serial_number == "TEST1234"
    assert queries and min(queries) >= 0.005
    assert fake.writes.count(b"REBOOT\r\n") == 1
    assert b"SAVECONFIG\r\n" not in fake.writes


class DisconnectOnReset(FakeSerial):
    def __init__(self):
        super().__init__(responses={"LOG VERSION": IDENTITY})
        self.disconnected = False

    @property
    def in_waiting(self):
        if self.disconnected:
            raise implementation.serial.SerialException("USB handle disconnected")
        return super().in_waiting

    def write(self, value):
        if value == b"REBOOT\r\n":
            self.writes.append(value)
            self.disconnected = True
            return len(value)
        return super().write(value)


def test_native_usb_reboot_closes_stale_handle_and_reopens_same_port(monkeypatch):
    original = DisconnectOnReset()
    returned = FakeSerial(responses={"LOG VERSION": IDENTITY})
    opened = []

    def factory(name, baud, **kwargs):
        opened.append((name, baud))
        if len(opened) == 1:
            return original
        if len(opened) == 2:
            raise implementation.serial.SerialException("USB not enumerated yet")
        return returned

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    with SerialDevice("/dev/serial/by-id/Device", 115200, timeout=0.02) as dev:
        assert dev.reboot(recovery_timeout=0.2).serial_number == "TEST1234"
        assert dev.is_open
    assert not original.is_open and not returned.is_open
    assert opened == [("/dev/serial/by-id/Device", 115200)] * 3
    assert original.writes.count(b"REBOOT\r\n") == 1
    assert b"REBOOT\r\n" not in returned.writes


def test_native_usb_missing_port_leaves_closed_handle_and_clear_error(monkeypatch):
    original = DisconnectOnReset()
    attempts = []

    def factory(*args, **kwargs):
        attempts.append(args)
        if len(attempts) == 1:
            return original
        raise implementation.serial.SerialException("port no longer exists")

    monkeypatch.setattr(implementation.serial, "Serial", factory)
    with SerialDevice("COM_TEST", 115200, timeout=0.02) as dev:
        with pytest.raises(ResponseTimeout, match="did not answer"):
            dev.reboot(recovery_timeout=0.005)
        assert not dev.is_open


def test_disconnected_device_reboot_reports_timeout_without_sending_reset(monkeypatch):
    original = DisconnectOnReset()
    monkeypatch.setattr(implementation.serial, "Serial", lambda *a, **k: original)
    with SerialDevice("COM_TEST", 115200, timeout=0.02) as dev:
        dev.read_info()
        original.disconnected = True
        with pytest.raises(ResponseTimeout, match="did not answer"):
            dev.reboot(recovery_timeout=0.005)
    assert b"REBOOT\r\n" not in original.writes
