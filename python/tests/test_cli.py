"""Customer CLI contracts exercised without opening physical serial ports."""

import binascii
import inspect
import json
import logging
import math
import os
import signal
import struct
import subprocess
import sys
from types import SimpleNamespace

from click.testing import CliRunner
import pytest

from hipnuc import cli, modbus, serial_device
from hipnuc.cli import main
from test_decoder import GGA, frame as protocol_frame
from test_serial_device import Clock, FakeSerial, IDENTITY, frame


CONNECTION = ["-p", "FAKE", "-b", "115200"]


def test_redirected_cli_uses_utf8_with_windows_legacy_encoding():
    code = """
from types import SimpleNamespace
from hipnuc import cli
cli.list_ports.comports = lambda: [SimpleNamespace(
    device='COM3', description='USB \\u4e32\\u884c\\u8bbe\\u5907',
    manufacturer=None, serial_number=None, vid=None, pid=None)]
cli.main(['list', '--json'])
"""
    result = subprocess.run(
        [sys.executable, "-c", code],
        env={**os.environ, "PYTHONIOENCODING": "cp1252"},
        capture_output=True,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout.decode("utf-8"))[0]["description"] == "USB 串行设备"


@pytest.fixture(autouse=True)
def no_physical_ports(monkeypatch):
    def fail(*args, **kwargs):
        raise AssertionError("CLI tests must not open physical ports")

    monkeypatch.setattr(serial_device.serial, "Serial", fail)
    monkeypatch.setattr(modbus, "ModbusSerialClient", fail)
    monkeypatch.setattr(cli.list_ports, "comports", lambda: [])


@pytest.fixture
def fake(monkeypatch):
    port = FakeSerial(responses={"LOG VERSION": IDENTITY})
    monkeypatch.setattr(serial_device.serial, "Serial", lambda *args, **kwargs: port)
    return port


def invoke(args):
    options = (
        {"mix_stderr": False} if "mix_stderr" in inspect.signature(CliRunner).parameters else {}
    )
    return CliRunner(**options).invoke(main, args)


def records(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def measurement_frame():
    packet = bytearray(frame())
    struct.pack_into("<3f", packet, 6 + 12, 1, 0, 0)
    struct.pack_into("<3f", packet, 6 + 24, 180, 0, 0)
    struct.pack_into("<3f", packet, 6 + 48, 90, 0, 0)
    struct.pack_into("<H", packet, 4, binascii.crc_hqx(packet[:4] + packet[6:], 0))
    return bytes(packet)


@pytest.mark.parametrize(
    "args",
    [
        [],
        ["--help"],
        ["help"],
        ["read", "--help"],
        ["scan", "--help"],
        ["command", "--help"],
        ["modbus", "--help"],
        ["modbus", "read", "--help"],
        ["modbus", "write-register", "--help"],
    ],
)
def test_help_does_not_open_ports(args):
    result = invoke(args)
    assert result.exit_code == 0, result.output
    assert "Usage:" in result.stdout
    assert result.stderr == ""


def test_list_empty_explains_human_output_and_preserves_json():
    result = invoke(["list"])
    assert result.exit_code == 0
    assert "No serial ports found" in result.stdout + result.stderr
    result = invoke(["list", "--json"])
    assert result.exit_code == 0
    assert json.loads(result.stdout) == []


@pytest.mark.parametrize("baud_args", [[], ["-b", "115200"]])
@pytest.mark.parametrize(
    "args",
    [
        ["command", "LOG VERSION"],
        ["command", "--file", "commands.txt"],
        ["baudrate", "9600"],
        ["reboot"],
    ],
)
def test_target_commands_require_port_before_any_discovery(monkeypatch, tmp_path, args, baud_args):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "commands.txt").write_text("LOG VERSION\n", encoding="utf-8")

    def reject(*args, **kwargs):
        raise AssertionError("Missing target must not enumerate or open ports")

    monkeypatch.setattr(cli.list_ports, "comports", reject)
    result = invoke([*args, *baud_args])
    assert result.exit_code == 2, result.output
    assert "Missing option" in result.stderr and "--port" in result.stderr
    assert "Searching" not in result.stderr
    assert result.stdout == ""
    assert invoke([*args, "--help"]).exit_code == 0


@pytest.mark.parametrize("args", [["command", "LOG VERSION"], ["command", "SAVECONFIG"]])
@pytest.mark.parametrize("baud_args", [[], ["-b", "115200"]])
def test_target_connection_only_searches_selected_port(monkeypatch, args, baud_args):
    opened = []

    def reject():
        raise AssertionError("An explicit port must not enumerate other ports")

    def connect(name, baud, **kwargs):
        assert name == "TARGET"
        opened.append((name, baud))
        return FakeSerial(name, baud, responses={"LOG VERSION": IDENTITY, "SAVECONFIG": b"OK\n"})

    monkeypatch.setattr(cli.list_ports, "comports", reject)
    monkeypatch.setattr(serial_device.serial, "Serial", connect)
    result = invoke([*args, "-p", "TARGET", *baud_args])
    assert result.exit_code == 0, result.output
    if baud_args:
        assert opened == [("TARGET", 115200)]
        assert "Checking" not in result.stderr
        assert "Detecting" not in result.stderr
    else:
        assert opened == [("TARGET", 115200), ("TARGET", 115200)]
        assert "Detecting baudrate on TARGET" in result.stderr


@pytest.mark.parametrize("args", [["info"], ["read", "--duration", "0.01"]])
def test_read_only_commands_still_discover_without_connection_parameters(monkeypatch, args):
    monkeypatch.setattr(cli.list_ports, "comports", lambda: [SimpleNamespace(device="FAKE")])

    def connect(name, baud, **kwargs):
        port = FakeSerial(
            name, baud, responses={"LOG VERSION": IDENTITY, "LOG USRCONFIG": b"COORD=2\nOK\n"}
        )
        port.pending.append(frame())
        return port

    monkeypatch.setattr(serial_device.serial, "Serial", connect)
    result = invoke(args)
    assert result.exit_code == 0, result.output
    assert "Checking FAKE at 115200 baud" in result.stderr
    assert "Connected to FAKE" in result.stderr


@pytest.mark.parametrize(
    "args",
    [
        [*CONNECTION, "read"],
        ["modbus", *CONNECTION, "registers"],
        ["modbus", *CONNECTION, "read"],
    ],
)
def test_connection_flags_belong_after_final_subcommand(args):
    result = invoke(args)
    assert result.exit_code == 2
    assert "No such option" in result.stderr
    assert result.stdout == ""


@pytest.mark.parametrize("as_json", [False, True])
def test_info_accepts_consistent_connection_flags(fake, as_json):
    result = invoke(["info", *CONNECTION, *(["--json"] if as_json else [])])
    assert result.exit_code == 0, result.output
    assert "HI14" in result.stdout
    assert "1.7.2" in result.stdout
    if as_json:
        assert isinstance(json.loads(result.stdout), dict)
    assert b"LOG VERSION\r\n" in fake.writes
    assert not fake.is_open


def test_scan_uses_same_singular_port_and_baudrate_flags(fake):
    result = invoke(["scan", *CONNECTION, "--json"])
    assert result.exit_code == 0, result.output
    report = json.loads(result.stdout)
    assert report["complete"] is True
    assert report["errors"] == {}
    assert len(report["devices"]) == 1
    assert report["devices"][0]["port"] == "FAKE"
    assert report["devices"][0]["baudrate"] == 115200
    assert report["devices"][0]["info"]["product_name"] == "HI14"
    assert not fake.is_open


def test_scan_preserves_port_open_failure(monkeypatch):
    def denied(*args, **kwargs):
        raise serial_device.serial.SerialException("Access is denied")

    monkeypatch.setattr(serial_device.serial, "Serial", denied)
    result = invoke(["scan", *CONNECTION, "--json"])
    assert result.exit_code == 1
    assert "FAKE" in result.stdout + result.stderr
    assert "Access is denied" in result.stdout + result.stderr


@pytest.mark.parametrize("command", ["scan", "info"])
def test_discovery_progress_precedes_port_open_and_keeps_json_clean(monkeypatch, command):
    logger = logging.getLogger("hipnuc.serial_device")
    original = (logger.level, logger.propagate, list(logger.handlers))
    root = logging.getLogger()
    root_original = (root.level, list(root.handlers))
    monkeypatch.setattr(cli.list_ports, "comports", lambda: [SimpleNamespace(device="FAKE")])

    def connect(*args, **kwargs):
        assert "Checking FAKE at 115200 baud" in sys.stderr.buffer.getvalue().decode("utf-8")
        return FakeSerial(*args, responses={"LOG VERSION": IDENTITY}, **kwargs)

    monkeypatch.setattr(serial_device.serial, "Serial", connect)
    result = invoke([command, "-b", "115200", "--json"])
    assert result.exit_code == 0, result.output
    assert isinstance(json.loads(result.stdout), dict)
    assert result.stderr.count("Checking FAKE at 115200 baud") == 1
    assert "found HI14" in result.stderr
    assert (logger.level, logger.propagate, logger.handlers) == original
    assert (root.level, root.handlers) == root_original


@pytest.mark.parametrize(
    "incoming,reason", [(b"", "no bytes received"), (b"noise\n", "no HiPNUC match")]
)
def test_failed_baud_progress_arrives_before_next_attempt(monkeypatch, incoming, reason):
    clock = Clock()
    monkeypatch.setattr(serial_device.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(serial_device.time, "sleep", clock.sleep)
    monkeypatch.setattr(cli, "BAUDRATES", (115200, 921600))

    def connect(name, baud, **kwargs):
        if baud == 921600:
            assert reason in sys.stderr.buffer.getvalue().decode("utf-8")
        port = FakeSerial(
            name, baud, responses={"LOG VERSION": IDENTITY if baud == 921600 else b""}
        )
        if incoming:
            port.pending.append(incoming)
        return port

    monkeypatch.setattr(serial_device.serial, "Serial", connect)
    result = invoke(["scan", "-p", "FAKE", "--timeout", "0.01", "--json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["devices"][0]["baudrate"] == 921600
    assert result.stderr.count("Checking FAKE") == 2


def test_discovery_explains_identity_retry_without_printing_measurements(fake, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(serial_device.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(serial_device.time, "sleep", clock.sleep)
    fake.pending.append(frame())
    fake.responses["LOG VERSION"] = lambda port: IDENTITY if len(port.writes) == 2 else []
    result = invoke(["scan", *CONNECTION, "--timeout", "0.01", "--json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["devices"][0]["info"]["product_name"] == "HI14"
    assert "valid binary data, identity timed out" in result.stderr
    assert result.stderr.count("retrying once") == 1
    assert "HI91" not in result.stderr


@pytest.mark.parametrize(
    "error,exit_code",
    [(serial_device.serial.SerialException("busy"), 1), (KeyboardInterrupt(), 130)],
)
def test_discovery_logging_is_restored_on_failure(monkeypatch, error, exit_code):
    logger = logging.getLogger("hipnuc.serial_device")
    original = (logger.level, logger.propagate, list(logger.handlers))

    def fail(*args, **kwargs):
        raise error

    monkeypatch.setattr(serial_device.serial, "Serial", fail)
    result = invoke(["scan", *CONNECTION])
    assert result.exit_code == exit_code
    assert (logger.level, logger.propagate, logger.handlers) == original


def test_command_failure_is_nonzero_and_stderr(fake):
    fake.responses["BAD"] = b"ERROR: invalid\n"
    result = invoke(["command", *CONNECTION, "BAD"])
    assert result.exit_code == 1
    assert "invalid" in result.stderr
    assert result.stdout == ""
    assert not fake.is_open


def test_command_response_timeout_is_visible_and_closes_port(fake):
    result = invoke(["command", *CONNECTION, "--timeout", "0.01", "NO_REPLY"])
    assert result.exit_code == 1
    assert result.stdout == ""
    assert "NO_REPLY" in result.stderr
    assert "Traceback" not in result.stderr
    assert not fake.is_open


def test_explicit_send_only_command_does_not_require_an_ack(fake):
    result = invoke(["command", *CONNECTION, "--response", "none", "--json", "NO_REPLY"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["acknowledged"] is False
    assert fake.writes == [b"NO_REPLY\r\n"]
    assert not fake.is_open


def test_command_file_stops_on_error_without_saving(fake, tmp_path):
    fake.responses.update({"FIRST": b"OK\n", "SECOND": b"ERROR\n"})
    path = tmp_path / "配置.txt"
    path.write_text("# Comment\nFIRST\nSECOND\nTHIRD\n", encoding="utf-8")
    result = invoke(["command", *CONNECTION, "--file", str(path), "--save"])
    assert result.exit_code == 1
    assert fake.writes == [b"FIRST\r\n", b"SECOND\r\n"]
    assert not fake.is_open


def test_command_file_json_failure_identifies_step_without_partial_json(fake, tmp_path):
    fake.responses.update({"FIRST": b"OK\n", "SECOND": b"ERROR: rejected\n"})
    path = tmp_path / "commands.txt"
    path.write_text("FIRST\nSECOND\nTHIRD\n", encoding="utf-8")
    result = invoke(["command", *CONNECTION, "--file", str(path), "--json", "--save", "--reboot"])
    assert result.exit_code == 1, result.output
    assert result.stdout == ""
    assert "Command 2/3 failed" in result.stderr
    assert "SECOND" in result.stderr
    assert fake.writes == [b"FIRST\r\n", b"SECOND\r\n"]
    assert not fake.is_open


def test_command_file_prints_reply_before_next_command(fake, tmp_path):
    def second(_port):
        assert "first reply" in sys.stdout.getvalue()
        return b"ERROR: stopped\n"

    fake.responses.update({"FIRST": b"first reply\nOK\n", "SECOND": second})
    path = tmp_path / "commands.txt"
    path.write_text("FIRST\nSECOND\nTHIRD\n", encoding="utf-8")
    result = invoke(["command", *CONNECTION, "--file", str(path), "--save", "--reboot"])
    assert result.exit_code == 1, result.output
    assert "first reply" in result.stdout
    assert "Command 2/3: SECOND" in result.stderr
    assert fake.writes == [b"FIRST\r\n", b"SECOND\r\n"]
    assert not fake.is_open


@pytest.mark.parametrize("save_reply,exit_code", [(b"OK\n", 0), (b"ERROR: save failed\n", 1)])
def test_command_save_then_reboot_and_recover_identity(fake, save_reply, exit_code):
    fake.responses.update(
        {"CONFIG IMU COORD 2": b"OK\n", "SAVECONFIG": save_reply, "REBOOT": b"OK\n"}
    )
    result = invoke(["command", "CONFIG IMU COORD 2", *CONNECTION, "--save", "--reboot", "--json"])
    assert result.exit_code == exit_code, result.output
    assert fake.writes[:2] == [b"CONFIG IMU COORD 2\r\n", b"SAVECONFIG\r\n"]
    if exit_code:
        assert b"REBOOT\r\n" not in fake.writes
        assert result.stdout == ""
    else:
        output = json.loads(result.stdout)
        assert output[0]["acknowledged"] and output[1]["acknowledged"]
        assert output[2]["product_name"] == "HI14"
        assert fake.writes.count(b"REBOOT\r\n") == 1
    assert not fake.is_open


@pytest.mark.parametrize("option", ["--save", "--reboot"])
@pytest.mark.parametrize("command", ["SERIALCONFIG 9600", "reboot", "FRESET"])
def test_raw_lifecycle_post_actions_are_rejected_before_connection(tmp_path, option, command):
    path = tmp_path / "commands.txt"
    path.write_text("CONFIG IMU COORD 2\n" + command, encoding="utf-8")
    result = invoke(["command", *CONNECTION, "--file", str(path), option])
    assert result.exit_code == 2, result.output
    assert "managed reconnection" in result.stderr


@pytest.mark.parametrize("option", ["--save", "--reboot"])
def test_send_only_cannot_trigger_post_actions(option):
    result = invoke(["command", "CONFIG IMU COORD 2", *CONNECTION, "--response", "none", option])
    assert result.exit_code == 2, result.output
    assert "require command responses" in result.stderr


def test_command_file_saves_once_at_end(fake, tmp_path):
    fake.responses.update({name: b"OK\n" for name in ("FIRST", "SECOND", "SAVECONFIG")})
    path = tmp_path / "commands.txt"
    path.write_text("FIRST\nSECOND\n", encoding="utf-8")
    result = invoke(["command", *CONNECTION, "--file", str(path), "--save"])
    assert result.exit_code == 0, result.output
    assert fake.writes == [b"FIRST\r\n", b"SECOND\r\n", b"SAVECONFIG\r\n"]


def test_command_and_file_are_mutually_exclusive(tmp_path):
    path = tmp_path / "commands.txt"
    path.write_text("FIRST\n", encoding="utf-8")
    result = invoke(["command", *CONNECTION, "FIRST", "--file", str(path)])
    assert result.exit_code == 2


def test_baudrate_distinguishes_connected_and_target_speed(fake):
    fake.responses["SERIALCONFIG 230400"] = b"OK\n"
    result = invoke(["baudrate", "230400", *CONNECTION, "--json"])
    assert result.exit_code == 0, result.output
    assert fake.writes[0] == b"LOG VERSION\r\n"
    assert b"SERIALCONFIG 230400\r\n" in fake.writes
    assert fake.baudrate == 230400
    assert not fake.is_open


def test_reboot_sends_reset_once_and_verifies_identity(fake):
    fake.responses["REBOOT"] = b"OK\n"
    result = invoke(["reboot", *CONNECTION, "--json"])
    assert result.exit_code == 0, result.output
    assert fake.writes.count(b"REBOOT\r\n") == 1
    assert "HI14" in result.stdout
    assert not fake.is_open


def test_read_default_is_human_and_recording_stays_si(fake, tmp_path):
    fake.pending.append(measurement_frame())
    path = tmp_path / "samples.jsonl"
    result = invoke(["read", *CONNECTION, "--duration", "0.02", "--record", str(path)])
    assert result.exit_code == 0, result.output
    assert "HI91" in result.stdout
    assert "deg/s" in result.stdout or "°/s" in result.stdout
    assert "m/s" in result.stdout
    assert "metadata" not in result.stdout
    assert not result.stdout.lstrip().startswith("{")
    assert "180.000" in result.stdout
    assert "90.000" in result.stdout
    sample = records(path)[0]
    assert sample["angular_velocity_rad_s"][0] == pytest.approx(math.pi)
    assert sample["euler_rad"][0] == pytest.approx(math.pi / 2)
    assert sample["acceleration_m_s2"][0] == pytest.approx(9.8)
    assert not fake.is_open


@pytest.mark.parametrize("second_angle", [-30.0, float("nan"), float("inf")])
def test_hi83_inclination_only_displays_degrees_and_handles_missing_values(
    fake, tmp_path, second_angle
):
    payload = struct.pack("<BHBI3f", 0x83, 0, 0, 1 << 9, 90, second_angle, 0)
    fake.pending.append(protocol_frame(payload))
    path = tmp_path / "inclination.jsonl"
    result = invoke(["read", *CONNECTION, "--duration", "0.02", "--record", str(path)])
    assert result.exit_code == 0, result.output
    assert "inclination=[90.000," in result.stdout
    assert "] °" in result.stdout
    assert "roll/pitch/yaw" not in result.stdout
    sample = records(path)[0]
    assert "euler_rad" not in sample
    assert sample["inclination_rad"][0] == pytest.approx(math.pi / 2)
    if math.isfinite(second_angle):
        assert "-30.000" in result.stdout
        assert sample["inclination_rad"][1] == pytest.approx(-math.pi / 6)
    else:
        assert "—" in result.stdout
        assert sample["inclination_rad"][1] is None
    assert not fake.is_open


def test_human_coordinates_preserve_seven_decimal_places(fake):
    fake.pending.append(GGA)
    result = invoke(["read", *CONNECTION, "--duration", "0.02"])
    assert result.exit_code == 0, result.output
    assert "latitude=48.1173000" in result.stdout
    assert "longitude=11.5166667" in result.stdout
    assert not fake.is_open


def test_human_throttling_is_per_type_and_preserves_recordings(fake, tmp_path):
    payload = (frame() + GGA) * 5
    fake.pending.append(payload)
    raw, parsed = tmp_path / "采集.bin", tmp_path / "采集.jsonl"
    result = invoke(
        [
            "read",
            *CONNECTION,
            "--duration",
            "0.02",
            "--display-rate",
            "1",
            "--record-raw",
            str(raw),
            "--record",
            str(parsed),
        ]
    )
    assert result.exit_code == 0, result.output
    assert result.stdout.count("HI91") == 1
    assert result.stdout.count("GGA") == 1
    assert len(records(parsed)) == 10
    assert raw.read_bytes() == payload
    assert not fake.is_open


def test_jsonl_stdout_is_complete_despite_human_display_limit(fake):
    fake.pending.append(frame() * 5)
    result = invoke(["read", *CONNECTION, "--duration", "0.02", "--display-rate", "1", "--jsonl"])
    assert result.exit_code == 0, result.output
    samples = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(samples) == 5
    assert all(sample["type"] == "HI91" for sample in samples)
    assert all("metadata" in sample for sample in samples)
    assert "sample" in result.stderr.lower()


def test_quiet_read_still_records_every_sample(fake, tmp_path):
    fake.pending.append(frame() * 5)
    path = tmp_path / "samples.jsonl"
    result = invoke(["read", *CONNECTION, "--duration", "0.02", "--quiet", "--record", str(path)])
    assert result.exit_code == 0, result.output
    assert result.stdout == ""
    assert len(records(path)) == 5


@pytest.mark.parametrize(
    "received,diagnostic", [(b"", "No bytes received"), (b"noise", "no valid measurement frames")]
)
def test_bounded_empty_read_fails_with_diagnostic_summary(fake, received, diagnostic):
    if received:
        fake.pending.append(received)
    result = invoke(["read", *CONNECTION, "--duration", "0.01"])
    assert result.exit_code == 1
    assert result.stdout == ""
    assert "sample" in result.stderr.lower()
    assert "0" in result.stderr
    assert diagnostic in result.stderr
    assert not fake.is_open


def test_existing_recording_is_not_silently_overwritten(tmp_path):
    path = tmp_path / "important.bin"
    path.write_bytes(b"original")
    result = invoke(["read", *CONNECTION, "--duration", "0.01", "--record-raw", str(path)])
    assert result.exit_code == 1
    assert "exist" in result.stderr.lower()
    assert path.read_bytes() == b"original"


def test_explicit_overwrite_replaces_recording(fake, tmp_path):
    path = tmp_path / "samples.jsonl"
    path.write_text("original", encoding="utf-8")
    fake.pending.append(frame())
    result = invoke(
        ["read", *CONNECTION, "--duration", "0.02", "--record", str(path), "--overwrite"]
    )
    assert result.exit_code == 0, result.output
    assert len(records(path)) == 1


@pytest.mark.parametrize("option", ["--duration", "--display-rate", "--timeout", "--scan-timeout"])
@pytest.mark.parametrize("value", ["nan", "inf", "-1"])
def test_invalid_read_options_do_not_open_ports_or_create_logs(tmp_path, option, value):
    raw, parsed = tmp_path / "capture.bin", tmp_path / "capture.jsonl"
    result = invoke(
        ["read", *CONNECTION, option, value, "--record-raw", str(raw), "--record", str(parsed)]
    )
    assert result.exit_code == 2, result.output
    assert option in result.stderr
    assert result.stdout == ""
    assert not raw.exists()
    assert not parsed.exists()


def test_ctrl_c_finishes_received_batch_then_returns_130(fake, monkeypatch, tmp_path):
    fake.pending.append(frame() * 5)
    original_echo = cli.click.echo
    interrupted = False

    def interrupt_on_sample(message=None, *args, **kwargs):
        nonlocal interrupted
        if not kwargs.get("err") and "HI91" in str(message) and not interrupted:
            interrupted = True
            signal.raise_signal(signal.SIGINT)
        return original_echo(message, *args, **kwargs)

    monkeypatch.setattr(cli.click, "echo", interrupt_on_sample)
    raw, parsed = tmp_path / "stop.bin", tmp_path / "stop.jsonl"
    result = invoke(
        [
            "read",
            *CONNECTION,
            "--duration",
            "0.1",
            "--jsonl",
            "--record-raw",
            str(raw),
            "--record",
            str(parsed),
        ]
    )
    assert interrupted
    assert result.exit_code == 130, result.exception
    assert len(records(parsed)) == 5
    assert raw.read_bytes() == frame() * 5
    assert not fake.is_open


def test_recording_write_failure_is_visible_and_closes_port(fake, monkeypatch, tmp_path):
    from hipnuc.recording import Recorder

    def disk_full(self, sample):
        raise OSError("disk full")

    monkeypatch.setattr(Recorder, "write", disk_full)
    fake.pending.append(frame())
    result = invoke(
        ["read", *CONNECTION, "--duration", "0.02", "--record", str(tmp_path / "out.jsonl")]
    )
    assert result.exit_code == 1
    assert "disk full" in result.stderr
    assert not fake.is_open
