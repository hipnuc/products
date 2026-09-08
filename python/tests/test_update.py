"""Scripted bootloaders verify protocol failures without touching a device.

Independent protocol reference: product mcuboot.c/kptl.c and bootloader app_co.c.
Serial programming uses word-aligned blocks: AT32 memory_write programs len/2
halfwords, and HC32 EFM_SequenceProgram requires word-aligned start addresses.
CAN app_co.c does not propagate flash-operation failures; its acknowledgements
are transport evidence, not board upgrade or model-match evidence.
"""

import binascii
from collections import deque
import struct
import sys
from types import SimpleNamespace

import pytest

from hipnuc import update
from hipnuc.errors import DeviceError, ResponseTimeout, TransportError, VerificationError
from test_firmware_image import record, write_hex


class Clock:
    def __init__(self):
        self.now = 0

    def monotonic(self):
        return self.now

    def time(self):
        return 1000000 + self.now

    def sleep(self, seconds):
        self.now += seconds


@pytest.fixture
def clock(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(update, "time", clock)
    return clock


@pytest.fixture
def firmware(tmp_path):
    return write_hex(
        tmp_path, [record(4, data=b"\x08\x00"), record(0, 0x4000, bytes(range(20))), record(1)]
    )


def response(tag, status=0, value=None):
    payload = bytes((0xA7 if tag == 7 else 0xA0, 0, 0, 2))
    payload += struct.pack("<II", status, tag if value is None else value)
    header = b"\x5a\xa4" + struct.pack("<H", len(payload))
    return header + struct.pack("<H", binascii.crc_hqx(header + payload, 0)) + payload


class SerialBootloader:
    def __init__(self, clock):
        self.clock = clock
        self.timeout = 0.05
        self.received = bytearray()
        self.pending = bytearray()
        self.commands = []
        self.writes = []
        self.closed = False
        self.packet_size = 8
        self.flash_size = 256 * 1024
        self.app_start = 0x08004000
        self.failure = None
        self.read_chunk = 1

    def close(self):
        self.closed = True

    def reset_input_buffer(self):
        self.pending.clear()

    def read(self, size):
        if self.failure == "read":
            raise OSError("disconnected")
        if self.failure == "trickle":
            self.clock.sleep(0.099)
        if not self.pending:
            self.clock.sleep(self.timeout)
        size = min(size, self.read_chunk)
        data = self.pending[:size]
        del self.pending[:size]
        return bytes(data)

    def write(self, frame):
        self.writes.append(frame)
        if self.failure == "write":
            return 0
        if frame == b"REBOOT BL\r\n":
            return len(frame)
        if frame == b"\x5a\xa6":
            if self.failure != "ping_timeout":
                reply = bytes.fromhex("5AA7000201500000")
                checksum = binascii.crc_hqx(reply, 0)
                if self.failure == "ping_crc":
                    checksum ^= 1
                self.pending.extend(reply + struct.pack("<H", checksum))
            return len(frame)
        assert int.from_bytes(frame[2:4], "little") == len(frame) - 6
        assert binascii.crc_hqx(frame[:4] + frame[6:], 0) == int.from_bytes(frame[4:6], "little")
        reply = b"\x5a\xa1"
        if frame[1] == 0xA4:
            tag = frame[6]
            parameters = struct.unpack(f"<{frame[9]}I", frame[10:])
            self.commands.append((tag, parameters))
            if tag == 7:
                properties = {3: self.app_start, 4: self.flash_size, 11: self.packet_size}
                reply += response(7, value=properties[parameters[0]])
            else:
                if tag == 4:
                    self.expected = parameters[1]
                status = 123 if self.failure == "erase_status" and tag == 2 else 0
                reply += response(tag, status=status)
            if tag == 2:
                if self.failure == "crc":
                    reply = reply[:-1] + bytes((reply[-1] ^ 1,))
                elif self.failure == "echo":
                    reply = b"\x5a\xa1" + response(tag, value=4)
                elif self.failure == "param_count":
                    bad = bytearray(response(tag))
                    bad[9] = 1
                    bad[4:6] = struct.pack("<H", binascii.crc_hqx(bad[:4] + bad[6:], 0))
                    reply = b"\x5a\xa1" + bad
            if tag == 11 and self.failure == "reset_timeout":
                reply = b""
        else:
            self.received.extend(frame[6:])
            if self.failure in ("nak", "abort"):
                reply = b"\x5a" + (b"\xa2" if self.failure == "nak" else b"\xa3")
            elif len(self.received) == self.expected and self.failure != "final_timeout":
                reply += response(4)
        self.pending.extend(reply)
        return len(frame)


def install_serial(monkeypatch, clock):
    device = SerialBootloader(clock)
    monkeypatch.setattr(update.serial, "Serial", lambda *args, **kwargs: device)
    return device


def test_serial_full_update_fragmentation_and_progress(monkeypatch, clock, firmware):
    device = install_serial(monkeypatch, clock)
    progress = []
    result = update.update_serial(
        firmware,
        port="TEST",
        baudrate=115200,
        progress=lambda written, total: progress.append((written, total)),
    )
    assert result == update.UpdateResult(20, True, True, True, False)
    assert device.received == bytes(range(20))
    assert progress == [(0, 20), (8, 20), (16, 20), (20, 20)]
    assert device.closed
    assert [tag for tag, _ in device.commands] == [7, 7, 2, 4, 11]
    assert device.commands[2] == (2, (0x08004000, 20))


@pytest.mark.parametrize(
    "failure,error",
    [
        ("ping_timeout", ResponseTimeout),
        ("ping_crc", VerificationError),
        ("erase_status", DeviceError),
        ("crc", VerificationError),
        ("echo", VerificationError),
        ("param_count", VerificationError),
        ("nak", DeviceError),
        ("abort", DeviceError),
        ("final_timeout", ResponseTimeout),
        ("reset_timeout", ResponseTimeout),
        ("write", TransportError),
        ("read", TransportError),
    ],
)
def test_serial_failures_never_retry_erase_or_data_or_reset(
    monkeypatch, clock, firmware, failure, error
):
    device = install_serial(monkeypatch, clock)
    device.failure = failure
    with pytest.raises(error):
        update.update_serial(firmware, port="TEST", baudrate=115200)
    assert device.closed
    tags = [tag for tag, _ in device.commands]
    assert tags.count(2) <= 1
    assert tags.count(4) <= 1
    assert tags.count(11) == (1 if failure == "reset_timeout" else 0)


@pytest.mark.parametrize(
    "field,value",
    [("packet_size", 0), ("packet_size", 3), ("flash_size", 0), ("flash_size", 19)],
)
def test_serial_checks_image_before_erasing(monkeypatch, clock, firmware, field, value):
    device = install_serial(monkeypatch, clock)
    setattr(device, field, value)
    with pytest.raises((ValueError, VerificationError)):
        update.update_serial(firmware, port="TEST", baudrate=115200)
    assert all(tag == 7 for tag, _ in device.commands)
    assert device.closed


def test_serial_timeout_not_restarted_by_each_byte(monkeypatch, clock, firmware):
    device = install_serial(monkeypatch, clock)
    device.failure = "trickle"
    with pytest.raises(ResponseTimeout):
        update.update_serial(firmware, port="TEST", baudrate=115200)
    assert clock.now < 4
    assert device.closed


@pytest.mark.parametrize(
    "exception", [KeyboardInterrupt(), OSError("disk full"), RuntimeError("bug")]
)
def test_serial_progress_failure_preserved_and_does_not_reset(
    monkeypatch, clock, firmware, exception
):
    device = install_serial(monkeypatch, clock)

    def progress(written, total):
        if written:
            raise exception

    with pytest.raises(type(exception)) as caught:
        update.update_serial(firmware, port="TEST", baudrate=115200, progress=progress)
    assert caught.value is exception
    assert device.closed
    assert all(tag != 11 for tag, _ in device.commands)


def test_bad_image_does_not_open_serial(monkeypatch, tmp_path):
    monkeypatch.setattr(update.serial, "Serial", lambda *a, **k: pytest.fail("opened serial"))
    with pytest.raises(FileNotFoundError):
        update.update_serial(tmp_path / "missing", port="TEST", baudrate=115200)


def test_serial_port_open_failure_is_actionable(monkeypatch, firmware):
    def fail(*args, **kwargs):
        raise PermissionError(13, "Access denied")

    monkeypatch.setattr(update.serial, "Serial", fail)
    with pytest.raises(TransportError, match="TEST"):
        update.update_serial(firmware, port="TEST", baudrate=115200)


@pytest.mark.parametrize("during_transfer", [False, True])
def test_serial_close_failure_does_not_mask_transfer_error(
    monkeypatch, clock, firmware, during_transfer
):
    device = install_serial(monkeypatch, clock)

    def close():
        raise OSError("close failed")

    device.close = close
    if during_transfer:
        device.failure = "erase_status"
    with pytest.raises(DeviceError if during_transfer else TransportError) as caught:
        update.update_serial(firmware, port="TEST", baudrate=115200)
    assert "status" in str(caught.value) if during_transfer else "close failed" in str(caught.value)


class Message:
    def __init__(self, arbitration_id, data, **kwargs):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = False
        self.is_error_frame = False
        self.is_remote_frame = False
        self.is_fd = False
        self.dlc = len(self.data)
        self.timestamp = 0
        self.__dict__.update(kwargs)


class CanBootloader:
    def __init__(self, clock):
        self.clock = clock
        self.requests = []
        self.pending = deque()
        self.received = bytearray()
        self.failure = None
        self.closed = False

    def send(self, message, timeout):
        request = message.data
        assert message.arbitration_id == 0x608
        assert not message.is_extended_id
        assert len(request) == 8
        self.requests.append(request)
        reply = bytearray(request)
        reply[0] = 0x60
        if request[0] == 0x23:
            if (request[3] == 9 and self.failure == "start_timeout") or (
                request[3] == 5 and self.failure == "enter_timeout"
            ):
                return
        elif request[0] == 0x21:
            if self.failure == "init_timeout":
                return
            if self.failure == "abort":
                reply = bytearray.fromhex("80511F0120000405")
            elif self.failure == "echo":
                reply[3] = 2
        else:
            size = 7 if not request[0] & 15 else ((15 - (request[0] & 15)) // 2) + 1
            self.received.extend(request[1 : 1 + size])
            reply[0] = 0x20 | (request[0] & 0x10)
            if self.failure == "toggle":
                reply[0] ^= 0x10
            if self.failure == "segment_timeout":
                return
        if self.failure == "start_abort" and request[:4] == b"\x23\x51\x1f\x09":
            reply = bytearray.fromhex("80511F0920000405")
        self.pending.append(Message(0x589, b"noise123"))  # Other node must be ignored.
        self.pending.append(Message(0x588, reply))

    def recv(self, timeout):
        if self.pending:
            return self.pending.popleft()
        self.clock.sleep(timeout)
        return None

    def shutdown(self):
        self.closed = True


@pytest.fixture
def bus(monkeypatch, clock):
    monkeypatch.setitem(sys.modules, "can", SimpleNamespace(Message=Message, CanError=OSError))
    return CanBootloader(clock)


@pytest.mark.parametrize("size", range(1, 22))
def test_can_final_segment_sizes_and_toggle(bus, tmp_path, size):
    path = tmp_path / "firmware.bin"
    path.write_bytes(bytes(range(size)))
    progress = []
    result = update.update_can(
        bus, 8, path, raw_binary=True, progress=lambda w, t: progress.append((w, t))
    )
    assert result == update.UpdateResult(size, True, True, True, False)
    assert bus.received == path.read_bytes()
    assert not bus.closed
    assert progress[0] == (0, size) and progress[-1] == (size, size)
    assert [data[3] for data in bus.requests if data[0] == 0x23] == [5, 6, 9]


@pytest.mark.parametrize(
    "failure,error",
    [
        ("init_timeout", ResponseTimeout),
        ("segment_timeout", ResponseTimeout),
        ("abort", DeviceError),
        ("echo", VerificationError),
        ("toggle", VerificationError),
        ("start_abort", DeviceError),
    ],
)
def test_can_transfer_failures(bus, firmware, failure, error):
    bus.failure = failure
    with pytest.raises(error):
        update.update_can(bus, 8, firmware)
    assert sum(data[0] == 0x21 for data in bus.requests) == 1
    if failure != "start_abort":
        assert b"\x23\x51\x1f\x09\0\0\0\0" not in bus.requests
    assert not bus.closed


def test_can_start_timeout_is_reported_without_claiming_application_running(bus, firmware):
    bus.failure = "start_timeout"
    result = update.update_can(bus, 8, firmware)
    assert result.transfer_acknowledged and result.start_requested
    assert not result.start_acknowledged and not result.application_verified


def test_can_missing_enter_ack_still_confirms_bootloader_without_reenter(bus, firmware):
    bus.failure = "enter_timeout"
    assert update.update_can(bus, 8, firmware).transfer_acknowledged
    assert sum(data[:4] == b"\x23\x51\x1f\x05" for data in bus.requests) == 1


def test_can_cancel_during_data_does_not_request_start(bus, firmware):
    def progress(written, total):
        if written:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        update.update_can(bus, 8, firmware, progress=progress)
    assert all(data[:4] != b"\x23\x51\x1f\x09" for data in bus.requests)
    assert not bus.closed


@pytest.mark.parametrize("node", [0, 128, 255, True, 1.5])
def test_invalid_can_target_never_sends(bus, firmware, node):
    with pytest.raises(ValueError):
        update.update_can(bus, node, firmware)
    assert not bus.requests


@pytest.mark.parametrize(
    "changes",
    [
        {"is_remote_frame": True},
        {"is_error_frame": True},
        {"is_fd": True},
        {"dlc": 7},
        {"data": b"\x60\x51\x1f\x05"},
    ],
)
def test_can_invalid_reply_flags_and_lengths(bus, changes):
    request = b"\x23\x51\x1f\x05\0\0\0\0"
    reply = Message(0x588, b"\x60" + request[1:])
    reply.__dict__.update(changes)
    bus.pending.append(reply)
    with pytest.raises(VerificationError):
        update._sdo_write(bus, 8, 5)


def test_can_rejects_unmatched_abort(bus):
    bus.pending.append(Message(0x588, bytes.fromhex("80511F0220000405")))
    with pytest.raises(VerificationError, match="Unmatched"):
        update._sdo_write(bus, 8, 5)


def test_can_discards_stale_start_ack_before_update(bus, firmware):
    bus.failure = "start_timeout"
    bus.pending.append(Message(0x588, bytes.fromhex("60511F0900000000")))
    assert not update.update_can(bus, 8, firmware).start_acknowledged


def test_can_late_enter_ack_does_not_break_confirm(bus):
    bus.pending.append(Message(0x588, bytes.fromhex("60511F0500000000")))
    update._sdo_write(bus, 8, 6)


def test_can_deadline_survives_continuous_unrelated_traffic(bus, clock):
    def receive(timeout):
        clock.sleep(0.02)
        return Message(0x589, bytes(8))

    bus.recv = receive
    with pytest.raises(ResponseTimeout):
        update._sdo_write(bus, 8, 6)
    assert clock.now <= 0.12


@pytest.mark.parametrize("operation", ["send", "recv"])
def test_can_io_error_is_not_misclassified_as_bootloader_timeout(bus, operation):
    def failure(*args, **kwargs):
        raise OSError("adapter disconnected")

    setattr(bus, operation, failure)
    with pytest.raises(TransportError, match="adapter disconnected") as caught:
        update._sdo_write(bus, 8, 5)
    assert not isinstance(caught.value, ResponseTimeout)


def test_can_wrong_data_echo_stops_before_reset(bus, firmware):
    original = bus.recv

    def receive(timeout):
        reply = original(timeout)
        if reply and reply.data[0] == 0x20 and reply.arbitration_id == 0x588:
            reply.data = b"\x20\xff" + reply.data[2:]
        return reply

    bus.recv = receive
    with pytest.raises(VerificationError, match="data echo"):
        update.update_can(bus, 8, firmware)
    assert all(data[:4] != b"\x23\x51\x1f\x09" for data in bus.requests)


def test_can_send_time_counts_against_response_deadline(bus, clock):
    original = bus.send

    def send(message, timeout):
        original(message, timeout)
        clock.sleep(timeout)

    bus.send = send
    with pytest.raises(ResponseTimeout):
        update._sdo_write(bus, 8, 5)
    assert clock.now == pytest.approx(0.1)


def test_serial_response_arriving_after_deadline_is_not_accepted(clock):
    device = SerialBootloader(clock)
    device.pending.extend(b"\x5a\xa1")
    device.read_chunk = 2
    original = device.read

    def read(size):
        reply = original(size)
        clock.sleep(0.11)
        return reply

    device.read = read
    with pytest.raises(ResponseTimeout):
        update._Kboot(device).ack(0.1)


def test_can_response_arriving_after_deadline_is_not_accepted(bus, clock):
    def receive(timeout):
        clock.sleep(0.11)
        return Message(0x588, bytes.fromhex("60511F0500000000"))

    bus.recv = receive
    with pytest.raises(ResponseTimeout):
        update._sdo_write(bus, 8, 5)


def application_hex(tmp_path, address, size):
    end = address + size - 1
    return write_hex(
        tmp_path,
        [
            record(4, data=(address >> 16).to_bytes(2, "big")),
            record(0, address & 0xFFFF, b"X"),
            record(4, data=(end >> 16).to_bytes(2, "big")),
            record(0, end & 0xFFFF, b"Y"),
            record(1),
        ],
    )


@pytest.mark.parametrize("address,size", [(0x08004000, 21), (0x08004001, 20)])
def test_serial_unaligned_image_rejected_before_device_io(monkeypatch, tmp_path, address, size):
    image = application_hex(tmp_path, address, size)
    monkeypatch.setattr(update.serial, "Serial", lambda *a, **k: pytest.fail("serial opened"))
    with pytest.raises(ValueError):
        update.update_serial(image, port="TEST", baudrate=115200)


def test_serial_uses_reported_capacity_and_the_images_address(monkeypatch, clock, tmp_path):
    image = application_hex(tmp_path, 0x08010000, 300 * 1024)
    device = install_serial(monkeypatch, clock)
    device.flash_size = 512 * 1024
    device.packet_size = 512
    device.read_chunk = 512
    assert update.update_serial(image, port="TEST", baudrate=115200).bytes_written == 300 * 1024
    assert device.commands[2] == (2, (0x08010000, 300 * 1024))


def test_serial_nonword_packet_limit_keeps_write_addresses_aligned(monkeypatch, clock, firmware):
    device = install_serial(monkeypatch, clock)
    device.packet_size = 7
    update.update_serial(firmware, port="TEST", baudrate=115200)
    assert [len(frame) - 6 for frame in device.writes if frame[:2] == b"\x5a\xa5"] == [4] * 5


def test_can_duplicate_confirm_ack_during_erase_is_ignored(bus, firmware):
    original = bus.send

    def send(message, timeout):
        if message.data[0] == 0x21:
            bus.pending.append(Message(0x588, bytes.fromhex("60511F0600000000")))
        original(message, timeout)

    bus.send = send
    assert update.update_can(bus, 8, firmware).transfer_acknowledged
    assert sum(data[0] == 0x21 for data in bus.requests) == 1


def test_can_old_timestamp_cannot_acknowledge_new_request(bus, clock):
    original = bus.send

    def send(message, timeout):
        original(message, timeout)
        for reply in bus.pending:
            reply.timestamp = clock.time() - 1

    bus.send = send
    with pytest.raises(ResponseTimeout):
        update._sdo_write(bus, 8, 5)
