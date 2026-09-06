"""Modbus CLI contracts use the register simulator, never a physical bus."""

import inspect
import json
import signal

from click.testing import CliRunner
import pytest

from hipnuc import cli, modbus, serial_device
from hipnuc.cli import main
from hipnuc.errors import ResponseTimeout
from test_modbus import FakeServer


CONNECTION = ["-p", "FAKE", "-b", "115200"]


@pytest.fixture(autouse=True)
def no_physical_ports(monkeypatch):
    def fail(*args, **kwargs):
        raise AssertionError("CLI tests must not open physical ports")

    monkeypatch.setattr(serial_device.serial, "Serial", fail)
    monkeypatch.setattr(modbus, "ModbusSerialClient", fail)
    monkeypatch.setattr(cli.list_ports, "comports", lambda: [])


@pytest.fixture
def server(monkeypatch):
    server = FakeServer()
    monkeypatch.setattr(modbus, "ModbusSerialClient", server.factory)
    return server


def invoke(args):
    options = (
        {"mix_stderr": False} if "mix_stderr" in inspect.signature(CliRunner).parameters else {}
    )
    return CliRunner(**options).invoke(main, ["modbus", *args])


def records(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_modbus_info_accepts_connection_options_after_leaf(server):
    result = invoke(["info", *CONNECTION, "--json"])
    assert result.exit_code == 0, result.output
    info = json.loads(result.stdout)
    assert info["product_name"] == "HI226"
    assert server.clients[0].options["baudrate"] == 115200
    assert server.clients[0].options["timeout"] == 2
    assert {call[1] for call in server.calls} == {80}
    assert all(client.closed for client in server.clients)


def test_modbus_read_count_and_recording_preserve_station(server, tmp_path):
    server.add_node(81)
    path = tmp_path / "站点81.jsonl"
    result = invoke(
        [
            "read",
            *CONNECTION,
            "--id",
            "81",
            "--count",
            "3",
            "--interval",
            "0",
            "--jsonl",
            "--display-rate",
            "1",
            "--record",
            str(path),
        ]
    )
    assert result.exit_code == 0, result.output
    samples = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(samples) == 3
    assert records(path) == samples
    assert all(sample["metadata"]["device_id"] == 81 for sample in samples)
    assert all("angular_velocity_rad_s" in sample for sample in samples)
    assert {call[1] for call in server.calls} == {81}
    assert all(client.closed for client in server.clients)
    assert "sample" in result.stderr.lower()


def test_modbus_read_runs_continuously_until_duration(server):
    result = invoke(["read", *CONNECTION, "--duration", "0.03", "--interval", "0", "--jsonl"])
    assert result.exit_code == 0, result.output
    samples = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(samples) > 1
    assert all(client.closed for client in server.clients)


def test_modbus_human_display_throttling_does_not_reduce_recording(server, tmp_path):
    path = tmp_path / "samples.jsonl"
    result = invoke(
        [
            "read",
            *CONNECTION,
            "--count",
            "5",
            "--interval",
            "0",
            "--display-rate",
            "1",
            "--record",
            str(path),
        ]
    )
    assert result.exit_code == 0, result.output
    assert len(records(path)) == 5
    assert len(result.stdout.strip().splitlines()) == 1
    assert "deg/s" in result.stdout or "°/s" in result.stdout
    assert "m/s" in result.stdout


def test_modbus_quiet_read_keeps_recording(server, tmp_path):
    path = tmp_path / "quiet.jsonl"
    result = invoke(
        [
            "read",
            *CONNECTION,
            "--count",
            "2",
            "--interval",
            "0",
            "--quiet",
            "--record",
            str(path),
        ]
    )
    assert result.exit_code == 0, result.output
    assert result.stdout == ""
    assert len(records(path)) == 2


def test_modbus_mru_is_visible_and_recorded_in_si_units(server, tmp_path):
    server.nodes[80].update(
        {
            address: value & 0xFFFF
            for address, value in enumerate([125, -250, 375, 50, 75, 125], 0x4E)
        }
    )
    path = tmp_path / "mru.jsonl"
    result = invoke(["read", *CONNECTION, "--count", "1", "--mru", "--record", str(path)])
    assert result.exit_code == 0, result.output
    sample = records(path)[0]
    assert sample["heave_surge_sway_m"] == pytest.approx([1.25, -2.5, 3.75])
    assert sample["heave_surge_sway_hz"] == pytest.approx([0.5, 0.75, 1.25])
    assert "heave/surge/sway=[1.250, -2.500, 3.750] m" in result.stdout
    assert "heave/surge/sway frequency=[0.500, 0.750, 1.250] Hz" in result.stdout
    assert ("read", 80, 0x34, 32) in server.calls
    assert all(client.closed for client in server.clients)


@pytest.mark.parametrize(
    ("args", "call"),
    [
        (["set-id", "81"], ("write", 80, 5, 81)),
        (["registers", "0x04", "1"], ("read", 80, 4, 1)),
        (["write-register", "0x12", "7"], ("write", 80, 0x12, 7)),
        (["baudrate", "230400"], ("write", 80, 4, 6)),
        (["write-register", "0", "0", "--no-verify"], ("write", 80, 0, 0)),
    ],
)
def test_modbus_operations_accept_connection_flags_at_final_subcommand(server, args, call):
    result = invoke([*args, *CONNECTION, "--json"])
    assert result.exit_code == 0, result.output
    json.loads(result.stdout)
    assert call in server.calls
    assert all(client.closed for client in server.clients)


def test_modbus_reboot_uses_the_selected_station(server):
    server.add_node(81)
    result = invoke(["reboot", *CONNECTION, "--id", "81", "--json"])
    assert result.exit_code == 0, result.output
    assert ("write", 81, 0, 0xFF) in server.calls
    assert {call[1] for call in server.calls} == {81}
    assert all(client.closed for client in server.clients)


def test_register_write_verifies_then_saves_then_reboots(server):
    result = invoke(["write-register", "6", "7", *CONNECTION, "--save", "--reboot", "--json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["verified"] is True
    writes = [call for call in server.calls if call[0] == "write"]
    assert writes == [("write", 80, 6, 7), ("write", 80, 0, 0), ("write", 80, 0, 255)]
    assert server.calls.index(("read", 80, 6, 1)) < server.calls.index(("write", 80, 0, 0))
    assert all(client.closed for client in server.clients)


@pytest.mark.parametrize("failure", ["readback", "save"])
def test_modbus_post_actions_stop_on_failure(server, failure):
    if failure == "readback":
        server.echo_only.add(6)
    else:
        server.exceptions[0] = 4
    result = invoke(["write-register", "6", "7", *CONNECTION, "--save", "--reboot", "--json"])
    assert result.exit_code == 1, result.output
    assert ("write", 80, 0, 255) not in server.calls
    if failure == "readback":
        assert ("write", 80, 0, 0) not in server.calls
    assert result.stdout == ""
    assert all(client.closed for client in server.clients)


@pytest.mark.parametrize("address", ["0", "0x04", "0x05"])
@pytest.mark.parametrize("option", ["--save", "--reboot"])
def test_raw_connection_register_post_actions_fail_before_bus_open(address, option):
    result = invoke(["write-register", address, "0", *CONNECTION, option])
    assert result.exit_code == 2, result.output
    assert "managed reconnection" in result.stderr


def test_modbus_set_id_saves_to_the_verified_new_station(server):
    result = invoke(["set-id", "81", *CONNECTION, "--save", "--json"])
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout)["readback"] == 81
    assert ("write", 80, 5, 81) in server.calls
    assert ("write", 81, 0, 0) in server.calls
    assert all(client.closed for client in server.clients)


@pytest.mark.parametrize(
    ("option", "value"),
    [
        ("--count", "0"),
        ("--interval", "nan"),
        ("--interval", "inf"),
        ("--interval", "-1"),
        ("--duration", "nan"),
        ("--duration", "0"),
        ("--timeout", "inf"),
        ("--timeout", "0"),
        ("--id", "0"),
        ("--id", "248"),
    ],
)
def test_invalid_modbus_read_options_fail_before_io(tmp_path, option, value):
    path = tmp_path / "capture.jsonl"
    result = invoke(["read", *CONNECTION, option, value, "--record", str(path)])
    assert result.exit_code == 2, result.output
    assert option in result.stderr
    assert not path.exists()


def test_modbus_requires_an_explicit_port():
    result = invoke(["read", "-b", "115200", "--count", "1"])
    assert result.exit_code == 2
    assert "--port" in result.stderr


def test_modbus_read_does_not_offer_serial_raw_recording(tmp_path):
    result = invoke(
        ["read", *CONNECTION, "--record-raw", str(tmp_path / "raw.bin"), "--count", "1"]
    )
    assert result.exit_code == 2
    assert "--record-raw" in result.stderr


def test_modbus_transport_failure_reports_error_and_closes_bus(server):
    server.connect_ok = False
    result = invoke(["read", *CONNECTION, "--duration", "0.01"])
    assert result.exit_code == 1
    assert "FAKE" in result.stderr
    assert result.stdout == ""
    assert all(client.closed for client in server.clients)


def test_modbus_ctrl_c_records_sample_and_returns_130(server, monkeypatch, tmp_path):
    path = tmp_path / "stop.jsonl"
    original_echo = cli.click.echo
    interrupted = False

    def interrupt_on_sample(message=None, *args, **kwargs):
        nonlocal interrupted
        if not kwargs.get("err") and not interrupted:
            interrupted = True
            signal.raise_signal(signal.SIGINT)
        return original_echo(message, *args, **kwargs)

    monkeypatch.setattr(cli.click, "echo", interrupt_on_sample)
    result = invoke(["read", *CONNECTION, "--duration", "0.1", "--jsonl", "--record", str(path)])
    assert interrupted
    assert result.exit_code == 130, result.exception
    assert len(records(path)) == 1
    assert all(client.closed for client in server.clients)


def test_modbus_ctrl_c_during_timed_out_transaction_returns_130(server, monkeypatch, tmp_path):
    def interrupt_then_timeout(self, **kwargs):
        signal.raise_signal(signal.SIGINT)
        raise ResponseTimeout("Simulated in-flight transaction timeout")

    monkeypatch.setattr(modbus.ModbusDevice, "read_sample", interrupt_then_timeout)
    path = tmp_path / "interrupted.jsonl"
    result = invoke(["read", *CONNECTION, "--duration", "0.1", "--record", str(path)])
    assert result.exit_code == 130, result.exception
    assert "0 samples" in result.stderr
    assert "Error:" not in result.stderr
    assert result.stdout == ""
    assert records(path) == []
    assert all(client.closed for client in server.clients)
