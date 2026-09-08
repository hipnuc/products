"""Run the four small examples with deterministic devices, never physical ports."""

import binascii
import builtins
import importlib.util
import json
import logging
import math
from pathlib import Path
import runpy
import signal
import struct

import pytest

import hipnuc
from hipnuc import DeviceError, Recorder, ResponseTimeout, TransportError, modbus, serial_device
from test_decoder import GGA
from test_modbus import FakeServer
from test_serial_device import Clock, FakeSerial, IDENTITY, frame


EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
NAMES = ("read_samples", "record_samples", "send_commands", "modbus_multinode")


@pytest.fixture(autouse=True)
def no_physical_ports(monkeypatch):
    def fail(*args, **kwargs):
        raise AssertionError("Example tests must not access physical ports")

    monkeypatch.setattr(serial_device.serial, "Serial", fail)
    monkeypatch.setattr(serial_device.list_ports, "comports", fail)
    monkeypatch.setattr(modbus, "ModbusSerialClient", fail)


def load_example(name):
    spec = importlib.util.spec_from_file_location(f"example_test_{name}", EXAMPLES / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def use_serial(example, port, monkeypatch):
    monkeypatch.setattr(serial_device.serial, "Serial", lambda *args, **kwargs: port)
    monkeypatch.setattr(example, "PORT", "FAKE")
    monkeypatch.setattr(example, "BAUDRATE", 115200)
    device_class = example.SerialDevice

    def connect(*args, **kwargs):
        kwargs["timeout"] = 0.02
        return device_class(*args, **kwargs)

    monkeypatch.setattr(example, "SerialDevice", connect)


def json_records(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


@pytest.mark.parametrize("name", NAMES)
def test_import_defines_editable_constants_without_device_or_file_effects(
    name, monkeypatch, tmp_path
):
    def fail(*args, **kwargs):
        raise AssertionError("Importing an example must not construct devices or recorders")

    monkeypatch.setattr(hipnuc, "SerialDevice", fail)
    monkeypatch.setattr(hipnuc, "ModbusBus", fail)
    monkeypatch.setattr(hipnuc, "Recorder", fail)
    monkeypatch.setattr(logging, "basicConfig", fail)
    monkeypatch.setattr(signal, "signal", fail)
    original_open = builtins.open

    def no_writes(file, mode="r", *args, **kwargs):
        assert not any(flag in mode for flag in "wax+"), "Import must not create output files"
        return original_open(file, mode, *args, **kwargs)

    monkeypatch.setattr(builtins, "open", no_writes)
    monkeypatch.chdir(tmp_path)
    example = load_example(name)
    assert callable(example.main)
    if name == "modbus_multinode":
        assert example.PORT == "COM3"
        assert example.BAUDRATE == 115200
        assert example.NODE_IDS == [80, 81]
        assert example.INTERVAL_S == 0.1
    else:
        assert example.PORT is None
        assert example.BAUDRATE is None
    if name == "record_samples":
        assert example.JSONL_PATH == "samples.jsonl"
        assert example.RAW_PATH is None
    if name == "send_commands":
        assert example.COMMANDS == ["LOG VERSION", "LOG COMCONFIG"]
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("name", NAMES)
def test_direct_execution_maps_keyboard_interrupt_to_130(name, monkeypatch, tmp_path):
    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(hipnuc, "SerialDevice", interrupt)
    monkeypatch.setattr(hipnuc, "ModbusBus", interrupt)
    monkeypatch.setattr(hipnuc, "Recorder", interrupt)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(SystemExit) as caught:
        runpy.run_path(str(EXAMPLES / f"{name}.py"), run_name="__main__")
    assert caught.value.code == 130


def test_read_example_prints_typed_si_values(monkeypatch, capsys):
    class InterruptedSerial(FakeSerial):
        @property
        def in_waiting(self):
            if not self.pending:
                raise KeyboardInterrupt
            return super().in_waiting

    packet = bytearray(frame())
    struct.pack_into("<3f", packet, 6 + 12, 1, 0, 0)
    struct.pack_into("<3f", packet, 6 + 24, 180, 0, 0)
    struct.pack_into("<H", packet, 4, binascii.crc_hqx(packet[:4] + packet[6:], 0))
    port = InterruptedSerial()
    port.pending.append(bytes(packet))
    example = load_example("read_samples")
    use_serial(example, port, monkeypatch)
    with pytest.raises(KeyboardInterrupt):
        example.main()
    output = capsys.readouterr().out
    assert "HI91" in output
    assert "Connected to FAKE at 115200 baud." in output
    assert "m/s" in output
    assert "rad/s" in output
    assert "acceleration (m/s^2): (9.8, 0.0, 0.0)" in output
    assert str(math.pi) in output
    assert port.writes == []
    assert not port.is_open


def test_command_example_preserves_order_without_automatic_identity_or_save(monkeypatch, capsys):
    port = FakeSerial(
        responses={"LOG VERSION": IDENTITY, "LOG COMCONFIG": b"UART1_BAUD=115200\r\nOK\r\n"}
    )
    example = load_example("send_commands")
    use_serial(example, port, monkeypatch)
    example.main()
    assert port.writes == [b"LOG VERSION\r\n", b"LOG COMCONFIG\r\n"]
    output = capsys.readouterr().out
    assert "LOG VERSION" in output
    assert "Connected to FAKE at 115200 baud." in output
    assert "PNAME=HI14" in output
    assert "LOG COMCONFIG" in output
    assert "UART1_BAUD=115200" in output
    assert not port.is_open


def test_command_example_stops_at_first_failure(monkeypatch):
    port = FakeSerial(responses={"LOG VERSION": b"ERROR: rejected\r\n"})
    example = load_example("send_commands")
    use_serial(example, port, monkeypatch)
    with pytest.raises(DeviceError, match="rejected"):
        example.main()
    assert port.writes == [b"LOG VERSION\r\n"]
    assert not port.is_open


@pytest.mark.parametrize("ending", ["interrupt", "disconnect", "idle"])
def test_record_example_preserves_original_chunks_and_closes_on_exit(ending, monkeypatch, tmp_path):
    events = []

    class EndingSerial(FakeSerial):
        @property
        def in_waiting(self):
            if not self.pending:
                if ending == "interrupt":
                    raise KeyboardInterrupt
                if ending == "disconnect":
                    raise serial_device.serial.SerialException("Device disconnected")
            return super().in_waiting

        def close(self):
            events.append("serial closed")
            super().close()

    class ObservedRecorder(Recorder):
        def close(self):
            events.append("recorder closed")
            super().close()

    shared_frame = frame() * 2
    chunks = [b"\x00noise\r\n" + shared_frame[:13], shared_frame[13:70], shared_frame[70:] + GGA]
    port = EndingSerial()
    port.pending.extend(chunks)
    example = load_example("record_samples")
    use_serial(example, port, monkeypatch)
    parsed, raw = tmp_path / "samples.jsonl", tmp_path / "serial.bin"
    monkeypatch.setattr(example, "JSONL_PATH", parsed)
    monkeypatch.setattr(example, "RAW_PATH", raw)
    monkeypatch.setattr(example, "Recorder", ObservedRecorder)
    error = {"interrupt": KeyboardInterrupt, "disconnect": TransportError, "idle": ResponseTimeout}
    with pytest.raises(error[ending]):
        example.main()
    assert raw.read_bytes() == b"".join(chunks)
    assert [sample["type"] for sample in json_records(parsed)] == ["HI91", "HI91", "GGA"]
    assert events[-2:] == ["serial closed", "recorder closed"]
    assert not port.is_open


@pytest.mark.parametrize("raw_only", [False, True])
def test_record_example_supports_either_format_without_callback_changes(
    raw_only, monkeypatch, tmp_path
):
    class InterruptedSerial(FakeSerial):
        @property
        def in_waiting(self):
            if not self.pending:
                raise KeyboardInterrupt
            return super().in_waiting

    port = InterruptedSerial()
    port.pending.append(frame())
    example = load_example("record_samples")
    use_serial(example, port, monkeypatch)
    parsed, raw = tmp_path / "samples.jsonl", tmp_path / "serial.bin"
    monkeypatch.setattr(example, "JSONL_PATH", None if raw_only else parsed)
    monkeypatch.setattr(example, "RAW_PATH", raw if raw_only else None)
    with pytest.raises(KeyboardInterrupt):
        example.main()
    if raw_only:
        assert raw.read_bytes() == frame()
        assert not parsed.exists()
    else:
        assert len(json_records(parsed)) == 1
        assert not raw.exists()
    assert not port.is_open


@pytest.mark.parametrize("disk_failure", [False, True])
def test_record_sigint_finishes_batch_unless_writing_fails(monkeypatch, tmp_path, disk_failure):
    original_handler = signal.getsignal(signal.SIGINT)
    recorders = []

    class InterruptingRecorder(Recorder):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            recorders.append(self)

        def write(self, sample):
            super().write(sample)
            if self.samples_written == 1:
                signal.raise_signal(signal.SIGINT)
                if disk_failure:
                    raise OSError("Disk full")

    chunk = frame() * 5
    port = FakeSerial()
    port.pending.append(chunk)
    example = load_example("record_samples")
    use_serial(example, port, monkeypatch)
    parsed, raw = tmp_path / "samples.jsonl", tmp_path / "serial.bin"
    monkeypatch.setattr(example, "JSONL_PATH", parsed)
    monkeypatch.setattr(example, "RAW_PATH", raw)
    monkeypatch.setattr(example, "Recorder", InterruptingRecorder)
    with pytest.raises(OSError if disk_failure else KeyboardInterrupt):
        example.main()
    assert len(json_records(parsed)) == (1 if disk_failure else 5)
    assert raw.read_bytes() == chunk
    assert not port.is_open
    assert recorders[0]._closed
    assert signal.getsignal(signal.SIGINT) is original_handler


@pytest.mark.parametrize("disconnect", [False, True])
def test_record_sigint_while_idle_is_bounded_and_preserves_disconnect(
    monkeypatch, tmp_path, disconnect
):
    original_handler = signal.getsignal(signal.SIGINT)
    clock = Clock()
    monkeypatch.setattr(serial_device.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(serial_device.time, "sleep", clock.sleep)

    class IdleSerial(FakeSerial):
        interrupted = False

        @property
        def in_waiting(self):
            if not self.interrupted:
                self.interrupted = True
                signal.raise_signal(signal.SIGINT)
                if disconnect:
                    raise serial_device.serial.SerialException("Disconnected during stop")
            return 0

    port = IdleSerial()
    example = load_example("record_samples")
    use_serial(example, port, monkeypatch)
    parsed = tmp_path / "samples.jsonl"
    monkeypatch.setattr(example, "JSONL_PATH", parsed)
    with pytest.raises(TransportError if disconnect else KeyboardInterrupt):
        example.main()
    assert clock.monotonic() <= 0.021
    assert parsed.read_bytes() == b""
    assert not port.is_open
    assert signal.getsignal(signal.SIGINT) is original_handler


def test_record_discovery_keeps_original_interrupt_handler(monkeypatch, tmp_path):
    original_handler = signal.getsignal(signal.SIGINT)
    example = load_example("record_samples")
    parsed = tmp_path / "samples.jsonl"
    monkeypatch.setattr(example, "JSONL_PATH", parsed)

    def interrupt_discovery(*args, **kwargs):
        assert signal.getsignal(signal.SIGINT) is original_handler
        signal.raise_signal(signal.SIGINT)

    monkeypatch.setattr(serial_device, "discover", interrupt_discovery)
    with pytest.raises(KeyboardInterrupt):
        example.main()
    assert signal.getsignal(signal.SIGINT) is original_handler
    assert parsed.read_bytes() == b""


@pytest.mark.parametrize("name", ["read_samples", "record_samples", "send_commands"])
def test_serial_examples_enable_progress_only_when_run(name, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(logging, "basicConfig", lambda **kwargs: calls.append(kwargs))
    example = load_example(name)
    assert calls == []
    monkeypatch.chdir(tmp_path)

    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(example, "SerialDevice", interrupt)
    with pytest.raises(KeyboardInterrupt):
        example.main()
    assert calls == [{"level": logging.INFO, "format": "%(message)s"}]


def test_modbus_example_uses_one_bus_for_both_stations_and_preserves_metadata(monkeypatch, capsys):
    server = FakeServer()
    server.add_node(81)
    monkeypatch.setattr(modbus, "ModbusSerialClient", server.factory)
    example = load_example("modbus_multinode")
    monkeypatch.setattr(example, "PORT", "FAKE")

    def interrupt_after_round(seconds):
        assert seconds == example.INTERVAL_S
        raise KeyboardInterrupt

    monkeypatch.setattr(example.time, "sleep", interrupt_after_round)
    with pytest.raises(KeyboardInterrupt):
        example.main()
    samples = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert [sample["metadata"]["device_id"] for sample in samples] == [80, 81]
    assert all(sample["metadata"]["protocol"] == "modbus_rtu" for sample in samples)
    assert len(server.clients) == 1
    assert server.clients[0].port == "FAKE"
    assert server.clients[0].closed


def test_modbus_example_closes_bus_if_second_station_fails(monkeypatch, capsys):
    server = FakeServer()
    monkeypatch.setattr(modbus, "ModbusSerialClient", server.factory)
    example = load_example("modbus_multinode")
    monkeypatch.setattr(example, "PORT", "FAKE")
    with pytest.raises(ResponseTimeout):
        example.main()
    samples = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert [sample["metadata"]["device_id"] for sample in samples] == [80]
    assert len(server.clients) == 1
    assert server.clients[0].closed
