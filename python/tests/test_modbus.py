"""Contract tests with independent register fixtures and real RTU framing."""

from __future__ import annotations

import binascii
import errno
import math
import struct
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import pytest
from pymodbus.exceptions import ConnectionException, ModbusIOException
from pymodbus.pdu import ExceptionResponse
from pymodbus.pdu.register_message import ReadHoldingRegistersResponse, WriteSingleRegisterResponse

from hipnuc import Decoder, modbus
from hipnuc.errors import DeviceError, ResponseTimeout, TransportError, VerificationError


@pytest.mark.parametrize("error_type", [RuntimeError, AttributeError, KeyboardInterrupt])
def test_modbus_unexpected_connect_failure_closes_client_and_propagates(
    rig, monkeypatch, error_type
):
    server, bus, _ = rig
    error = error_type("Unexpected client failure")

    def fail():
        # Model an error after the client has already acquired a port.
        server.clients[0].socket = object()
        raise error

    monkeypatch.setattr(bus._client, "connect", fail)
    with pytest.raises(error_type) as failure:
        bus.open()
    assert failure.value is error
    assert server.clients[0].closed
    assert server.clients[0].socket is None


def test_modbus_false_connect_does_not_guess_the_cause(rig):
    server, bus, _ = rig
    server.connect_ok = False
    with pytest.raises(TransportError) as failure:
        bus.open()
    assert "access permissions" in str(failure.value)
    assert "Permission denied" not in str(failure.value)
    assert "port is in use" not in str(failure.value)
    assert server.clients[0].closed


def test_modbus_raised_permission_error_keeps_original_cause(rig, monkeypatch):
    server, bus, _ = rig

    def denied():
        raise PermissionError(errno.EACCES, "Permission denied")

    monkeypatch.setattr(bus._client, "connect", denied)
    with pytest.raises(TransportError) as failure:
        bus.open()
    assert "Permission denied" in str(failure.value)
    assert isinstance(failure.value.__cause__, PermissionError)
    assert server.clients[0].closed


def identity_words(name="HI226", version=172, serial="1122334455667788"):
    raw = name.encode("ascii").ljust(16, b"\0")
    raw += struct.pack(">HH", version, 110)
    raw += bytes(10) + bytes.fromhex(serial)
    return list(struct.unpack(">19H", raw))


class FakeServer:
    def __init__(self):
        self.nodes = {}
        self.baudrates = {}
        self.clients = []
        self.calls = []
        self.echo_only = set()
        self.exceptions = {}
        self.lost_acks = set()
        self.bad_echo = False
        self.write_hook = None
        self.read_hook = None
        self.connect_ok = True
        self.add_node(80)

    def add_node(self, node, *, version=172, serial="1122334455667788"):
        registers = {
            address: value
            for address, value in enumerate(identity_words(version=version, serial=serial), 0x70)
        }
        registers.update({4: 5, 5: node, 6: 1, 9: 0x1234, 10: 0, 11: 0, 0xA6: 24})
        self.nodes[node] = registers
        self.baudrates[node] = 115200

    def factory(self, port, **options):
        client = FakeClient(self, port, options)
        self.clients.append(client)
        return client


class FakeClient:
    def __init__(self, server, port, options):
        self.server = server
        self.port = port
        self.options = options
        self.closed = False
        self.comm_params = SimpleNamespace(timeout_connect=options["timeout"])
        self.transaction = SimpleNamespace(
            comm_params=SimpleNamespace(timeout_connect=options["timeout"])
        )
        self.socket = None

    def connect(self):
        self.closed = False
        if self.server.connect_ok and self.socket is None:
            self.socket = SimpleNamespace(
                timeout=self.comm_params.timeout_connect, write_timeout=None
            )
        return self.server.connect_ok

    def close(self):
        self.closed = True
        self.socket = None

    def _node(self, device_id):
        if (
            device_id not in self.server.nodes
            or self.options["baudrate"] != self.server.baudrates[device_id]
        ):
            raise ModbusIOException("No response received after 0 retries")
        return self.server.nodes[device_id]

    def read_holding_registers(self, address, *, count, device_id):
        self.server.calls.append(("read", device_id, address, count))
        registers = self._node(device_id)
        if self.server.read_hook:
            response = self.server.read_hook(device_id, address, count)
            if response is not None:
                return response
        return ReadHoldingRegistersResponse(
            registers=[registers.get(index, 0) for index in range(address, address + count)],
            dev_id=device_id,
        )

    def write_register(self, address, value, *, device_id):
        self.server.calls.append(("write", device_id, address, value))
        registers = self._node(device_id)
        if self.server.write_hook:
            self.server.write_hook(device_id, address, value)
        if address in self.server.exceptions:
            return ExceptionResponse(6, self.server.exceptions[address], device_id=device_id)
        if address not in self.server.echo_only:
            if address == 5:
                self.server.nodes[value] = self.server.nodes.pop(device_id)
                self.server.baudrates[value] = self.server.baudrates.pop(device_id)
                registers[5] = value
            elif address == 0:
                if value == 0xFF:
                    self.server.baudrates[device_id] = modbus.BAUDRATES[registers[4]]
            else:
                registers[address] = value
                if address == 8:
                    registers[10] = 1
                    registers[11] = 0
        if address in self.server.lost_acks:
            raise ModbusIOException("No response received after 0 retries")
        return WriteSingleRegisterResponse(
            address=address + int(self.server.bad_echo),
            registers=[value],
            dev_id=device_id,
        )


@pytest.fixture
def rig(monkeypatch):
    server = FakeServer()
    monkeypatch.setattr(modbus, "ModbusSerialClient", server.factory)
    bus = modbus.ModbusBus("COM_TEST", timeout=0.02)
    yield server, bus, bus.device(80)
    bus.close()


def test_serial_adapter_configuration_and_context_ownership(rig):
    server, bus, _ = rig
    assert server.clients[0].options == {
        "baudrate": 115200,
        "bytesize": 8,
        "parity": "N",
        "stopbits": 1,
        "timeout": 0.02,
        "retries": 0,
        "handle_local_echo": False,
    }
    with bus as opened:
        assert opened is bus
        assert not server.clients[0].closed
    assert server.clients[0].closed
    server.connect_ok = False
    with pytest.raises(TransportError, match="Cannot open"):
        bus.open()


@pytest.mark.parametrize("node", [0, -1, 248, 255, True, 1.5])
def test_only_unicast_addresses_are_accepted(rig, node):
    _, bus, _ = rig
    with pytest.raises(ValueError):
        bus.device(node)


@pytest.mark.parametrize(
    "address,count", [(-1, 1), (0x10000, 1), (0xFFFF, 2), (0, 0), (0, 126), (0, True)]
)
def test_read_bounds_are_checked_before_io(rig, address, count):
    server, _, device = rig
    with pytest.raises(ValueError):
        device.read_registers(address, count)
    assert server.calls == []


def test_identity_block_is_big_endian_and_unknown_versions_work(rig):
    server, _, device = rig
    server.nodes[80][0x78] = 999
    info = device.read_info()
    assert info.product_name == "HI226"
    assert info.firmware_version == "9.9.9"
    assert info.bootloader_version == "1.1.0"
    assert info.serial_number == "1122334455667788"
    assert info.build is None
    assert len(bytes.fromhex(info.raw_response)) == 38
    assert server.calls == [("read", 80, 0x70, 19)]


def test_empty_optional_identity_does_not_gate_device(rig):
    server, _, device = rig
    server.nodes[80].update(dict.fromkeys(range(0x70, 0x83), 0))
    info = device.read_info()
    assert info.product_name is None
    assert info.firmware_version is None
    assert info.serial_number is None
    assert device.read_registers(0x05) == [80]


def load_sample(server):
    # Independently specified wire values: +1g/-0.5g, +/-125deg/s,
    # +10uT using the historical scale, signed Euler/pressure/quaternion.
    raw = struct.pack(">9h", 2048, -1024, 0, 2048, -2048, 0, 320, -640, 0)
    raw += struct.pack(">3i", -90000, 45000, 179999)
    raw += struct.pack(">hi4h2hI", -1234, 10132500, 10000, -5000, 0, 0, -1000, 2000, 0x80000001)
    raw += struct.pack(">6h", 123, -456, 0, 75, 100, 125)
    server.nodes[80].update(enumerate(struct.unpack(">32H", raw), 0x34))
    return raw


@pytest.mark.parametrize("protocol", ["HI91", "HI83", "Modbus"])
def test_common_source_acceleration_matches_across_protocols(rig, protocol):
    # The same product measurement is 9.8/-4.9/2.45 m/s²: HI91 sends
    # 1/-0.5/0.25 G, HI83 sends m/s², and Modbus sends 2048 counts per G.
    expected = (9.8, -4.9, 2.45)
    if protocol == "Modbus":
        server, _, device = rig
        server.nodes[80].update({0x34: 2048, 0x35: (-1024) & 0xFFFF, 0x36: 512})
        sample = device.read_sample(include_status=False)
    else:
        if protocol == "HI91":
            payload = bytearray(76)
            payload[0] = 0x91
            struct.pack_into("<3f", payload, 12, 1.0, -0.5, 0.25)
        else:
            payload = struct.pack("<BHBI3f", 0x83, 0, 0, 1, 9.8, -4.9, 2.45)
        header = b"\x5a\xa5" + struct.pack("<H", len(payload))
        crc = binascii.crc_hqx(header + payload, 0)
        sample = Decoder().feed(header + struct.pack("<H", crc) + payload)[0]
    assert sample.acceleration_m_s2 == pytest.approx(expected)
    assert sample.to_dict()["acceleration_m_s2"] == pytest.approx(expected)


def test_sample_fixed_point_si_sign_word_order_and_uptime(rig):
    server, bus, _ = rig
    expected_raw = load_sample(server)
    device = bus.device(80)
    sample = device.read_sample(include_mru=True)
    values = sample.values
    assert values["acceleration_m_s2"] == pytest.approx((9.8, -4.9, 0))
    assert values["angular_velocity_rad_s"] == pytest.approx(
        (math.radians(125), -math.radians(125), 0)
    )
    assert values["magnetic_field_t"] == pytest.approx((320 / 32.768e6, -640 / 32.768e6, 0))
    assert values["euler_rad"] == pytest.approx((-math.pi / 2, math.pi / 4, math.radians(179.999)))
    assert values["temperature_c"] == -12.34
    assert values["pressure_pa"] == 101325.0
    assert values["quaternion_wxyz"] == (1, -0.5, 0, 0)
    assert values["inclination_rad"] == pytest.approx((math.radians(-11), math.radians(22)))
    assert values["device_time_us"] == 2147483649000
    assert values["device_time_s"] == 2147483.649
    assert values["heave_surge_sway_m"][:2] == (1.23, -4.56)
    assert values["heave_surge_sway_hz"][2] == 1.25
    assert values["main_status"] == 0x1234
    assert sample.raw == expected_raw
    assert sample.complete
    assert sample.metadata["device_time"]["epoch"] == "boot"
    assert sample.metadata["register_snapshot"] == "not_guaranteed"
    assert sample.received_time_ns > 0
    assert sample.to_dict()["acceleration_m_s2"] == list(values["acceleration_m_s2"])
    assert server.calls == [("read", 80, 0x70, 19), ("read", 80, 0x34, 32), ("read", 80, 9, 3)]


def test_documented_scale_is_default_and_identity_is_cached(rig):
    server, _, device = rig
    load_sample(server)
    sample = device.read_sample(include_status=False)
    assert sample.values["magnetic_field_t"][0] == pytest.approx(9.765625e-6)
    assert "main_status" not in sample.values
    assert "heave_surge_sway_m" not in sample.values
    assert len(sample.raw) == 52
    device.read_sample(include_status=False)
    assert server.calls.count(("read", 80, 0x70, 19)) == 1


@pytest.mark.parametrize("version", [170, 171, 172, 173, 999])
def test_old_fc06_echo_is_followed_by_readback_on_every_version(rig, version):
    server, _, device = rig
    server.nodes[80][0x78] = version
    server.echo_only.add(6)
    with pytest.raises(VerificationError, match="read back 1, expected 7"):
        device.write_register(0x06, 7)
    assert ("read", 80, 6, 1) in server.calls
    assert ("write", 80, 0, 0) not in server.calls


def test_lost_write_ack_is_verified_without_replaying_write(rig):
    server, _, device = rig
    server.lost_acks.add(6)
    result = device.write_register(6, 7)
    assert result.verified and not result.acknowledged
    assert result.readback == 7
    assert server.calls == [("write", 80, 6, 7), ("read", 80, 6, 1)]


def test_exception_response_preserves_device_code(rig):
    server, _, device = rig
    server.exceptions[6] = 4
    with pytest.raises(DeviceError) as caught:
        device.write_register(6, 7)
    assert caught.value.code == 4
    assert server.calls == [("write", 80, 6, 7)]


def test_mismatched_echo_and_short_response_are_rejected(rig):
    server, _, device = rig
    server.bad_echo = True
    with pytest.raises(VerificationError, match="echo"):
        device.write_register(6, 7)
    server.read_hook = lambda node, addr, count: ReadHoldingRegistersResponse(
        registers=[0], dev_id=node
    )
    with pytest.raises(VerificationError, match="expected 2"):
        device.read_registers(0x34, 2)


@pytest.mark.parametrize(
    "exception,expected",
    [
        (ModbusIOException("No response received after 0 retries"), ResponseTimeout),
        (ModbusIOException("invalid byte count"), TransportError),
        (ConnectionException("port disconnected"), TransportError),
        (OSError("adapter unplugged"), TransportError),
    ],
)
def test_transport_errors_are_distinct_from_device_errors(rig, exception, expected):
    server, _, device = rig

    def fail(*args):
        raise exception

    server.read_hook = fail
    with pytest.raises(expected):
        device.read_registers(6)


def test_register_configuration_is_read_back_before_explicit_save(rig):
    server, _, device = rig
    assert device.read_registers(0x04, 3) == [5, 80, 1]
    result = device.write_register(0xA6, 520)
    assert result.verified and result.readback == 520
    device.save_config()
    assert server.calls[-3:] == [
        ("write", 80, 0xA6, 520),
        ("read", 80, 0xA6, 1),
        ("write", 80, 0, 0),
    ]
    assert ("write", 80, 0, 0xFF) not in server.calls


def test_raw_access_supports_unknown_single_register_and_does_not_save(rig):
    server, _, device = rig
    result = device.write_register(0x4321, 0xFFFF)
    assert result.to_dict() == {
        "address": 0x4321,
        "value": 0xFFFF,
        "acknowledged": True,
        "verified": True,
        "readback": 0xFFFF,
    }
    assert server.calls == [("write", 80, 0x4321, 0xFFFF), ("read", 80, 0x4321, 1)]


@pytest.mark.parametrize("lost_ack", [False, True])
def test_id_change_reads_and_saves_at_new_id(rig, lost_ack):
    server, _, device = rig
    if lost_ack:
        server.lost_acks.add(5)
    result = device.set_id(81, save=True)
    assert result.verified and result.acknowledged is not lost_ack
    assert device.device_id == 81
    assert server.calls == [
        ("write", 80, 5, 81),
        ("read", 81, 5, 1),
        ("write", 81, 0, 0),
    ]


def test_baud_write_saves_without_reset_or_host_change(rig):
    server, bus, device = rig
    result = device.set_baudrate(460800, save=True)
    assert result.value == 7 and result.verified
    assert bus.baudrate == server.baudrates[80] == 115200
    assert ("write", 80, 0, 0xFF) not in server.calls
    assert server.calls.count(("write", 80, 0, 0)) == 1


def test_build_dependent_256000_code_is_tried_and_read_back(rig):
    server, bus, device = rig
    result = device.set_baudrate(256000)
    assert result.value == 9 and result.verified
    server.echo_only.add(4)
    server.nodes[80][4] = 5
    with pytest.raises(VerificationError):
        device.set_baudrate(256000)
    assert bus.baudrate == 115200


def test_explicit_baud_reboot_accepts_missing_ack_and_confirms_identity(rig, monkeypatch):
    server, bus, device = rig
    monkeypatch.setattr(modbus.time, "sleep", lambda _: None)

    # SAVE still needs its ACK; only the reset response is lost.
    def after_write(node, address, value):
        if address == 0 and value == 0xFF:
            server.lost_acks.add(0)

    server.write_hook = after_write
    result = device.set_baudrate(460800, reboot=True, save=True, timeout=0.02)
    assert result.verified
    assert bus.baudrate == server.baudrates[80] == 460800
    assert len(server.clients) == 2
    assert server.clients[0].closed
    assert server.calls.count(("write", 80, 0, 0)) == 1
    assert server.calls.count(("write", 80, 0, 0xFF)) == 1
    assert server.calls[-1] == ("read", 80, 0x70, 19)


def test_reboot_false_save_option_is_respected(rig, monkeypatch):
    server, _, device = rig
    monkeypatch.setattr(modbus.time, "sleep", lambda _: None)
    device.set_baudrate(460800, reboot=True, save=False, timeout=0.02)
    assert ("write", 80, 0, 0) not in server.calls


def test_configuration_batch_saves_once_only_when_explicit(rig, monkeypatch):
    server, _, device = rig
    monkeypatch.setattr(modbus.time, "sleep", lambda _: None)
    device.write_register(0x06, 7)
    device.write_register(0xA6, 520)
    device.set_id(81)
    device.set_baudrate(460800)
    assert not any(call[0] == "write" and call[2:] == (0, 0) for call in server.calls)
    device.save_config()
    device.reboot(timeout=0.02, baudrate=460800)
    assert server.calls.count(("write", 81, 0, 0)) == 1


def test_baud_reconnect_failure_keeps_new_host_speed_without_replaying_reset(rig, monkeypatch):
    server, bus, device = rig
    monkeypatch.setattr(modbus.time, "sleep", lambda _: None)
    server.echo_only.add(0)
    with pytest.raises(ResponseTimeout, match="after reboot"):
        device.set_baudrate(460800, reboot=True, timeout=0.002)
    assert bus.baudrate == 460800
    assert server.calls.count(("write", 80, 0, 0xFF)) == 1
    bus.reconfigure(115200)
    assert device.read_info().serial_number == "1122334455667788"


def test_write_only_commands_do_not_claim_unavailable_verification(rig):
    _, _, device = rig
    assert not device.save_config().verified
    result = device.write_register(0xA5, 3, verify=False)
    assert result.acknowledged and not result.verified
    result = device.write_register(0x08, 2, verify=False)
    assert result.acknowledged and not result.verified
    assert device.read_status()["calibration_status"] == 1


def test_write_only_echo_does_not_imply_running_calibration(rig):
    server, _, device = rig
    server.echo_only.add(8)
    server.nodes[80][10] = 3  # A previous success is not a new running state.
    result = device.write_register(0x08, 1, verify=False)
    assert result.acknowledged and not result.verified
    assert device.read_status()["calibration_status"] == 3
    assert server.calls.count(("write", 80, 8, 1)) == 1


def test_bus_lock_covers_write_and_readback_across_nodes(rig):
    server, bus, device = rig
    server.add_node(81, serial="8877665544332211")
    another = bus.device(81)
    writing = threading.Event()
    release_write = threading.Event()
    reader_started = threading.Event()

    def hold_write(*args):
        writing.set()
        assert release_write.wait(2)

    def read_other():
        reader_started.set()
        return another.read_registers(6)

    server.write_hook = hold_write
    with ThreadPoolExecutor(max_workers=2) as pool:
        write_future = pool.submit(device.write_register, 6, 7)
        assert writing.wait(2)
        read_future = pool.submit(read_other)
        assert reader_started.wait(2)
        assert server.calls == [("write", 80, 6, 7)]
        release_write.set()
        assert write_future.result().verified
        assert read_future.result() == [1]
    assert server.calls == [("write", 80, 6, 7), ("read", 80, 6, 1), ("read", 81, 6, 1)]


def crc_frame(body):
    """Independent small RTU fixture encoder, not SDK production code."""
    crc = 0xFFFF
    for value in body:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return body + struct.pack("<H", crc)


class SerialPeer:
    """A byte-level serial peer for the real PyModbus client/framer."""

    def __init__(self, *, local_echo=False, corrupt=False):
        self.local_echo = local_echo
        self.corrupt = corrupt
        self.pending = bytearray()
        self.writes = []
        self.is_open = True
        self.options = None

    @property
    def in_waiting(self):
        return len(self.pending)

    def write(self, request):
        self.writes.append(bytes(request))
        assert request == crc_frame(request[:-2])
        if request[1] == 3:
            assert request.hex() == "500300700013085d"
            words = identity_words()
            response = crc_frame(bytes.fromhex("500326") + struct.pack(">19H", *words))
        else:
            response = request
        if self.corrupt:
            response = response[:-1] + bytes([response[-1] ^ 1])
        if self.local_echo:
            self.pending.extend(request)
        self.pending.extend(response)
        return len(request)

    def read(self, size):
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data

    def close(self):
        self.is_open = False


@pytest.mark.parametrize("local_echo", [False, True])
def test_real_pymodbus_315_serial_rtu_frames_and_local_echo(monkeypatch, local_echo):
    import pymodbus.client.serial

    peer = SerialPeer(local_echo=local_echo)

    def serial_factory(port, **options):
        assert port == "SERIAL_SIMULATOR"
        peer.options = options
        return peer

    monkeypatch.setattr(pymodbus.client.serial.serial, "serial_for_url", serial_factory)
    with modbus.ModbusBus("SERIAL_SIMULATOR", timeout=0.02, handle_local_echo=local_echo) as bus:
        device = bus.device(80)
        assert device.read_info().firmware_version == "1.7.2"
        result = device.write_register(0xA5, 1, verify=False)
        assert result.acknowledged and not result.verified
        assert peer.writes[-1] == bytes.fromhex("500600a5000155a8")
    assert not peer.is_open
    assert peer.options["baudrate"] == 115200
    assert peer.options["parity"] == "N"


def test_real_pymodbus_rejects_bad_crc_without_retrying(monkeypatch):
    import pymodbus.client.serial

    peer = SerialPeer(corrupt=True)
    monkeypatch.setattr(
        pymodbus.client.serial.serial, "serial_for_url", lambda *args, **kwargs: peer
    )
    with modbus.ModbusBus("SIMULATOR", timeout=0.01) as bus:
        with pytest.raises(ResponseTimeout):
            bus.device(80).read_info()
    assert len(peer.writes) == 1


class WireClock:
    def __init__(self):
        self.now = 0.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class LowBaudSerialPeer(SerialPeer):
    """Deliver real RTU bytes no faster than a 4800-baud 8N1 wire.

    USB deliveries are either coalesced or split into three batches. The
    second split batch arrives after 0.5 s but is not yet a complete ADU, so
    the real transaction manager must also have the longer deadline.
    """

    def __init__(self, clock, *, fragmented):
        super().__init__()
        self.clock = clock
        self.fragmented = fragmented
        self.schedule = []
        self.drop_next = False
        self.used_timeouts = []

    def _deliver(self):
        while self.schedule and self.schedule[0][0] <= self.clock.now:
            _, data = self.schedule.pop(0)
            self.pending.extend(data)

    @property
    def in_waiting(self):
        self._deliver()
        return len(self.pending)

    def read(self, size):
        self._deliver()
        return super().read(size)

    def write(self, request):
        self.writes.append(bytes(request))
        self.used_timeouts.append((self.timeout, self.write_timeout))
        assert request == crc_frame(request[:-2])
        if self.drop_next:
            self.drop_next = False
            return len(request)
        if request[1] == 3:
            count = int.from_bytes(request[4:6], "big")
            words = list(range(count))
            response = crc_frame(
                request[:2] + bytes([2 * count]) + struct.pack(f">{count}H", *words)
            )
        else:
            response = request
        ends = (200, 230, 255) if self.fragmented and len(response) == 255 else (len(response),)
        cursor = 0
        for end in ends:
            # Delivery follows request transmission, slave turnaround, and
            # physical reception of this many response bytes, all at 8N1.
            at = self.clock.now + (len(request) + 3.5 + end) * 10 / 4800
            self.schedule.append((at, response[cursor:end]))
            cursor = end
        return len(request)


@pytest.mark.parametrize("fragmented", [False, True])
def test_real_pymodbus_low_baud_long_read_then_short_timeout(monkeypatch, fragmented):
    import pymodbus.client.serial as serial_module
    import pymodbus.transaction.transaction as transaction_module

    clock = WireClock()
    peers = []

    def factory(port, **options):
        peer = LowBaudSerialPeer(clock, fragmented=fragmented)
        peer.timeout = options["timeout"]
        peer.write_timeout = None
        peers.append(peer)
        return peer

    monkeypatch.setattr(serial_module.serial, "serial_for_url", factory)
    monkeypatch.setattr(serial_module.time, "time", clock.time)
    monkeypatch.setattr(serial_module.time, "sleep", clock.sleep)
    monkeypatch.setattr(transaction_module, "monotonic", clock.time)

    # Negative control: the original fixed 0.5-s PyModbus budget loses this
    # legitimate 255-byte response. No production timeout formula is mocked.
    original = serial_module.ModbusSerialClient("WIRE", baudrate=4800, timeout=0.5, retries=0)
    original.connect()
    try:
        with pytest.raises(ModbusIOException, match="No response received"):
            original.read_holding_registers(0x100, count=125, device_id=80)
    finally:
        original.close()

    with modbus.ModbusBus("WIRE", baudrate=4800, timeout=0.5) as bus:
        device = bus.device(80)
        started = clock.now
        assert device.read_registers(0x100, 125) == list(range(125))
        assert clock.now - started > 0.531  # Response wire time alone exceeds the old budget.
        peer = peers[-1]
        long_timeout, long_write_timeout = peer.used_timeouts[-1]
        assert long_timeout > 1.0
        assert long_write_timeout == long_timeout
        assert bus._client.comm_params.timeout_connect == long_timeout
        assert bus._client.transaction.comm_params.timeout_connect == long_timeout

        # The same connected client must shrink its next request's budget,
        # including the serial socket and the complete-response deadline.
        peer.drop_next = True
        started = clock.now
        with pytest.raises(ResponseTimeout):
            device.read_registers(0x100, 1)
        assert 0.5 <= clock.now - started < 0.65
        short_timeout, short_write_timeout = peer.used_timeouts[-1]
        assert short_timeout < 0.6 < long_timeout
        assert short_write_timeout == short_timeout
        assert bus._client.transaction.comm_params.timeout_connect == short_timeout

        result = device.write_register(0xA5, 1, verify=False)
        assert result.acknowledged
        assert peer.used_timeouts[-1][0] < 0.6
        assert len(peer.writes) == 3  # No hidden retry of either reads or writes.
    assert not peers[-1].is_open
