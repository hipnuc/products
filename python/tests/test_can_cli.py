"""CAN CLI tests use simulated buses and never access a physical interface."""

from contextlib import contextmanager
from dataclasses import dataclass
import json
import signal
import subprocess
import sys

from click.testing import CliRunner
import pytest

from hipnuc import cli
from hipnuc.errors import TransportError


@pytest.mark.parametrize(
    "args",
    [
        ["can"],
        ["can", "--help"],
        ["can", "read", "--help"],
        ["can", "reg", "read", "--help"],
        ["can", "reg", "write", "--help"],
        ["can", "update", "--help"],
        ["update", "--help"],
    ],
)
def test_help_without_device_or_optional_dependency(args):
    result = CliRunner().invoke(cli.main, args)
    assert result.exit_code == 0, result.output
    assert "Usage:" in result.stdout
    assert result.stderr == ""


def test_can_connection_failure_creates_no_recording(monkeypatch, tmp_path):
    @contextmanager
    def fail(*args, **kwargs):
        raise TransportError("can0 is unavailable")
        yield

    monkeypatch.setattr(cli, "_can_connection", fail)
    path = tmp_path / "samples.jsonl"
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "can0", "--record", str(path)])
    assert result.exit_code == 1
    assert "can0 is unavailable" in result.stderr
    assert not path.exists()


@pytest.fixture
def fake_bus(monkeypatch):
    can = pytest.importorskip("can")

    class Bus:
        def __init__(self):
            self.messages = []
            self.closed = False
            self.received = 0

        def recv(self, timeout):
            self.received += 1
            return self.messages.pop(0) if self.messages else None

    bus = Bus()

    @contextmanager
    def connect(*args, **kwargs):
        try:
            yield bus
        finally:
            bus.closed = True

    monkeypatch.setattr(cli, "_can_connection", connect)
    # Different stations at one instant must remain distinct in both display and recording.
    bus.messages = [
        can.Message(
            arbitration_id=0x0CFF3400 | node,
            data=b"\x00\x08\x00\x00\x00\x00\x00\x00",
            is_extended_id=True,
            timestamp=1.0,
        )
        for node in (8, 9)
    ]
    return bus


def test_can_jsonl_records_all_stations(monkeypatch, tmp_path, fake_bus):
    path = tmp_path / "samples.jsonl"
    result = CliRunner().invoke(
        cli.main, ["can", "read", "-i", "can0", "--count", "2", "--record", str(path), "--jsonl"]
    )
    assert result.exit_code == 0, result.output
    output = [json.loads(line) for line in result.stdout.splitlines()]
    saved = [json.loads(line) for line in path.read_text().splitlines()]
    assert output == saved
    assert [sample["node_id"] for sample in saved] == [8, 9]
    assert all(sample["acceleration_m_s2"] == [9.8, 0.0, 0.0] for sample in saved)
    assert fake_bus.closed


def test_can_existing_file_stops_before_receiving(tmp_path, fake_bus):
    path = tmp_path / "samples.jsonl"
    path.write_text("keep")
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "can0", "--record", str(path)])
    assert result.exit_code == 1
    assert path.read_text() == "keep"
    assert fake_bus.received == 0
    assert fake_bus.closed


def test_can_display_does_not_hide_a_second_station(fake_bus):
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "can0", "--count", "2"])
    assert result.exit_code == 0, result.output
    assert "id=8" in result.stdout
    assert "id=9" in result.stdout


@pytest.mark.parametrize(
    "args",
    [
        ["update", "image.hex"],
        ["update", "image.hex", "-p", "COM3"],
        ["can", "update", "image.hex", "-i", "can0"],
        ["can", "reg", "read", "0", "-i", "can0"],
        ["can", "reg", "write", "0", "1", "-i", "can0", "--id", "255"],
    ],
)
def test_writes_and_updates_require_explicit_target(args):
    result = CliRunner().invoke(cli.main, args)
    assert result.exit_code == 2


def test_serial_update_reports_transfer_without_claiming_application_start(monkeypatch, tmp_path):
    from hipnuc import update

    @dataclass
    class Result:
        bytes_written: int = 12
        transfer_acknowledged: bool = True
        start_requested: bool = True
        start_acknowledged: bool = True
        application_verified: bool = False

    def run(path, *, port, baudrate, progress):
        assert port == "COM3" and baudrate == 115200
        progress(0, 12)
        progress(12, 12)
        return Result()

    monkeypatch.setattr(update, "update_serial", run)
    path = tmp_path / "firmware.hex"
    path.write_text("validated by library")
    result = CliRunner().invoke(
        cli.main, ["update", str(path), "-p", "COM3", "-b", "115200", "--json"]
    )
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["application_verified"] is False
    assert "100%" in result.stderr


def test_base_install_and_help_do_not_need_python_can():
    code = """
import importlib.abc
import sys
class NoCan(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "can":
            raise ModuleNotFoundError("No module named 'can'", name="can")
sys.meta_path.insert(0, NoCan())
import hipnuc
assert hipnuc.Decoder and hipnuc.update_serial
from hipnuc.cli import main
main(["can", "read", "-i", "can0"])
"""
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, encoding="utf-8"
    )
    assert result.returncode == 1
    assert 'python -m pip install ".[can]"' in result.stderr
    assert "Traceback" not in result.stderr


def test_missing_socketcan_interface_reports_name_and_next_step(monkeypatch):
    can = pytest.importorskip("can")
    monkeypatch.setattr(cli.sys, "platform", "linux")

    def fail(**kwargs):
        assert kwargs["channel"] == "missing0"
        assert kwargs["ignore_config"] is True
        raise OSError(19, "No such device")

    monkeypatch.setattr(can, "Bus", fail)
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "missing0"])
    assert result.exit_code == 1
    assert "Cannot open missing0" in result.stderr
    assert "ip -details link show missing0" in result.stderr


@pytest.mark.parametrize("invalid", [False, True])
def test_can_timeout_and_finite_zero_sample_failure(monkeypatch, fake_bus, invalid):
    now = [0.0]
    monkeypatch.setattr(cli.time, "monotonic", lambda: now[0])
    fake_bus.messages = []

    def receive(timeout):
        now[0] += timeout
        if invalid:
            from can import Message

            return Message(arbitration_id=0x0CFF3408, data=b"\x00", is_extended_id=True)
        return None

    fake_bus.recv = receive
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "can0", "--timeout", "0.2"])
    assert result.exit_code == 1
    assert ("No valid HiPNUC samples" if invalid else "No CAN frames") in result.stderr
    assert fake_bus.closed

    now[0] = 0
    result = CliRunner().invoke(cli.main, ["can", "read", "-i", "can0", "--duration", "0.1"])
    assert result.exit_code == 1
    assert "No HiPNUC CAN samples collected" in result.stderr


def test_can_interrupt_finishes_the_received_frame(monkeypatch, fake_bus, tmp_path):
    receive = fake_bus.recv

    def interrupted(timeout):
        message = receive(timeout)
        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        return message

    fake_bus.recv = interrupted
    path = tmp_path / "samples.jsonl"
    result = CliRunner().invoke(
        cli.main, ["can", "read", "-i", "can0", "--record", str(path), "--jsonl"]
    )
    assert result.exit_code == 130, result.output
    assert len(path.read_text().splitlines()) == 1
    assert json.loads(result.stdout) == json.loads(path.read_text())
    assert fake_bus.closed


@pytest.mark.parametrize("during_close", [False, True])
def test_can_disk_failure_is_reported_and_closes_bus(monkeypatch, fake_bus, tmp_path, during_close):
    def failure(*args):
        raise OSError("disk full")

    if during_close:
        original_close = cli.Recorder.close

        def close(recording):
            original_close(recording)
            failure()

        monkeypatch.setattr(cli.Recorder, "close", close)
    else:
        monkeypatch.setattr(cli.Recorder, "write", failure)
    result = CliRunner().invoke(
        cli.main,
        ["can", "read", "-i", "can0", "--record", str(tmp_path / "out.jsonl"), "--count", "1"],
    )
    assert result.exit_code == 1
    assert "disk full" in result.stderr
    assert fake_bus.closed


def test_can_close_failure_wins_over_ctrl_c(monkeypatch, fake_bus, tmp_path):
    original_close = cli.Recorder.close
    receive = fake_bus.recv

    def interrupted(timeout):
        message = receive(timeout)
        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
        return message

    def fail_close(recording):
        original_close(recording)
        raise OSError("final flush failed")

    fake_bus.recv = interrupted
    monkeypatch.setattr(cli.Recorder, "close", fail_close)
    result = CliRunner().invoke(
        cli.main, ["can", "read", "-i", "can0", "--record", str(tmp_path / "out.jsonl")]
    )
    assert result.exit_code == 1
    assert "final flush failed" in result.stderr
    assert fake_bus.closed
