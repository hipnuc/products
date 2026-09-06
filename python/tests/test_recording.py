"""Recording must preserve samples, received bytes and file error semantics."""

from datetime import datetime, timezone
import io
import json
from pathlib import Path

import pytest

from hipnuc.models import Sample
from hipnuc.recording import Recorder


def sample():
    return Sample(
        "HI91",
        {
            "acceleration_m_s2": [1.0, None, float("nan")],
            "utc": datetime(2026, 1, 2, tzinfo=timezone.utc),
        },
        b"shared outer frame",
        received_time_ns=123,
        complete=False,
        issues=("invalid component",),
        metadata={"source": "测试", "station_id": 80},
    )


def test_constructor_does_not_create_files_or_resolve_paths(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "resolve", lambda *_args, **_kwargs: pytest.fail("unexpected I/O"))
    recorder = Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin")
    assert list(tmp_path.iterdir()) == []
    assert recorder.samples_written == recorder.raw_bytes_written == 0
    with pytest.raises(ValueError, match="not open"):
        recorder.write(sample())
    with pytest.raises(ValueError, match="not open"):
        recorder.write_raw(b"data")


def test_jsonl_and_raw_preserve_independent_data_and_counts(tmp_path):
    jsonl_path, raw_path = tmp_path / "样本.jsonl", tmp_path / "原始.bin"
    measurement = sample()
    raw_chunks = [b"noise\x00", b"bad crc\xff", b"shared outer frame", b"OK\r\n"]
    with Recorder(jsonl_path, raw_path=raw_path) as recorder:
        for chunk in raw_chunks:
            recorder.write_raw(chunk)
        recorder.write(measurement)
        recorder.write(measurement)
        assert recorder.samples_written == 2
        assert recorder.raw_bytes_written == sum(map(len, raw_chunks))
    raw_jsonl = jsonl_path.read_bytes()
    assert raw_jsonl.count(b"\n") == 2
    assert b"\r" not in raw_jsonl
    assert [json.loads(line) for line in raw_jsonl.splitlines()] == [measurement.to_dict()] * 2
    assert "测试" in jsonl_path.read_text(encoding="utf-8")
    assert b"NaN" not in raw_jsonl
    assert b"raw_hex" not in raw_jsonl
    assert raw_path.read_bytes() == b"".join(raw_chunks)


def test_disabled_formats_are_harmless_callbacks(tmp_path):
    with Recorder(tmp_path / "samples.jsonl") as recorder:
        recorder.write_raw(b"unrecorded")
        recorder.write(sample())
        assert recorder.raw_bytes_written == 0
        assert recorder.samples_written == 1
    with Recorder(raw_path=tmp_path / "capture.bin") as recorder:
        recorder.write(sample())
        recorder.write_raw(bytearray(b"ab"))
        recorder.write_raw(memoryview(b"cdef").cast("H"))
        assert recorder.samples_written == 0
        assert recorder.raw_bytes_written == 6
    assert (tmp_path / "capture.bin").read_bytes() == b"abcdef"


def test_no_output_is_rejected():
    with pytest.raises(ValueError, match="At least one"):
        Recorder()


@pytest.mark.parametrize("overwrite", [False, True])
def test_same_output_path_rejected_before_touching_file(tmp_path, overwrite):
    path = tmp_path / "capture"
    path.write_bytes(b"keep")
    alias = tmp_path / ".." / tmp_path.name / "capture"
    with pytest.raises(ValueError, match="must be different"):
        Recorder(path, raw_path=alias, overwrite=overwrite).open()
    assert path.read_bytes() == b"keep"


def test_hard_link_output_alias_rejected_before_truncation(tmp_path):
    path, alias = tmp_path / "capture", tmp_path / "alias"
    path.write_bytes(b"keep")
    try:
        alias.hardlink_to(path)
    except OSError as error:
        pytest.skip(f"Hard links are unavailable on this filesystem: {error}")
    with pytest.raises(ValueError, match="must be different"):
        Recorder(path, raw_path=alias, overwrite=True).open()
    assert path.read_bytes() == b"keep"


@pytest.mark.parametrize("existing_output", ["samples.jsonl", "capture.bin"])
def test_existing_output_prevents_creation_of_other_output(tmp_path, existing_output):
    existing = tmp_path / existing_output
    existing.write_bytes(b"keep")
    with pytest.raises(FileExistsError):
        Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin").open()
    assert existing.read_bytes() == b"keep"
    assert list(tmp_path.iterdir()) == [existing]


def test_explicit_overwrite_replaces_existing_contents(tmp_path):
    path = tmp_path / "capture.bin"
    path.write_bytes(b"old recording")
    with Recorder(raw_path=path, overwrite=True) as recorder:
        recorder.write_raw(b"new")
    assert path.read_bytes() == b"new"


def test_close_is_idempotent_and_does_not_allow_accidental_reopen(tmp_path):
    recorder = Recorder(tmp_path / "samples.jsonl").open()
    assert recorder.open() is recorder
    recorder.write(sample())
    recorder.close()
    recorder.close()
    with pytest.raises(ValueError, match="closed"):
        recorder.open()
    assert len((tmp_path / "samples.jsonl").read_text(encoding="utf-8").splitlines()) == 1


class TextFile(io.StringIO):
    def __init__(self, *, fail_flush=False, fail_close=False, fail_write=False):
        super().__init__()
        self.flushes = 0
        self.fail_flush = fail_flush
        self.fail_close = fail_close
        self.fail_write = fail_write

    def write(self, data):
        if self.fail_write:
            raise OSError("write failed")
        return super().write(data)

    def flush(self):
        self.flushes += 1
        if self.fail_flush:
            raise OSError("flush failed")
        return super().flush()

    def close(self):
        super().close()
        if self.fail_close:
            raise OSError("close failed")


class RawFile(io.BytesIO):
    def __init__(self):
        super().__init__()
        self.flushes = 0

    def flush(self):
        self.flushes += 1
        return super().flush()


def mock_outputs(monkeypatch, text_file, raw_file):
    def open_path(path, *_args, **_kwargs):
        return text_file if path.suffix == ".jsonl" else raw_file

    monkeypatch.setattr(Path, "open", open_path)


def test_active_writes_flush_both_files_at_one_second_deadline(tmp_path, monkeypatch):
    current_time = [100.0]
    monkeypatch.setattr("hipnuc.recording.monotonic", lambda: current_time[0])
    text_file, raw_file = TextFile(), RawFile()
    mock_outputs(monkeypatch, text_file, raw_file)
    with Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin") as recorder:
        recorder.write(sample())
        current_time[0] = 100.999
        recorder.write_raw(b"a")
        assert text_file.flushes == raw_file.flushes == 0
        current_time[0] = 101.0
        recorder.write_raw(b"b")
        assert text_file.flushes == raw_file.flushes == 1
        current_time[0] = 101.5
        recorder.write(sample())
        assert text_file.flushes == raw_file.flushes == 1
        recorder.flush()
        assert text_file.flushes == raw_file.flushes == 2
        current_time[0] = 102.4
        recorder.write(sample())
        assert text_file.flushes == raw_file.flushes == 2
        current_time[0] = 102.5
        recorder.write(sample())
        assert text_file.flushes == raw_file.flushes == 3


def test_flush_failure_still_attempts_other_file_and_context_closes_both(tmp_path, monkeypatch):
    text_file, raw_file = TextFile(fail_flush=True), RawFile()
    mock_outputs(monkeypatch, text_file, raw_file)
    with pytest.raises(OSError, match="flush failed"):
        with Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin") as recorder:
            recorder.write(sample())
            recorder.write_raw(b"x")
            recorder.flush()
    assert text_file.flushes == raw_file.flushes == 1
    assert text_file.closed and raw_file.closed


def test_write_failure_propagates_without_incrementing_sample_count(tmp_path, monkeypatch):
    text_file, raw_file = TextFile(fail_write=True), RawFile()
    mock_outputs(monkeypatch, text_file, raw_file)
    with pytest.raises(OSError, match="write failed"):
        with Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin") as recorder:
            recorder.write(sample())
    assert recorder.samples_written == 0
    assert text_file.closed and raw_file.closed


def test_close_failure_still_closes_other_file(tmp_path, monkeypatch):
    text_file, raw_file = TextFile(fail_close=True), RawFile()
    mock_outputs(monkeypatch, text_file, raw_file)
    recorder = Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin").open()
    with pytest.raises(OSError, match="close failed"):
        recorder.close()
    assert text_file.closed and raw_file.closed
    recorder.close()


@pytest.mark.parametrize("failure", [RuntimeError("acquisition failed"), KeyboardInterrupt()])
def test_original_acquisition_error_survives_close_failure(tmp_path, monkeypatch, failure):
    text_file, raw_file = TextFile(fail_close=True), RawFile()
    mock_outputs(monkeypatch, text_file, raw_file)
    with pytest.raises(type(failure)) as caught:
        with Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin"):
            raise failure
    assert caught.value is failure
    assert text_file.closed and raw_file.closed


def test_partial_open_failure_closes_first_file_and_preserves_original_error(tmp_path, monkeypatch):
    text_file = TextFile(fail_close=True)

    def open_path(path, *_args, **_kwargs):
        if path.suffix == ".jsonl":
            return text_file
        raise PermissionError("second output denied")

    monkeypatch.setattr(Path, "open", open_path)
    with pytest.raises(PermissionError, match="second output denied"):
        Recorder(tmp_path / "samples.jsonl", raw_path=tmp_path / "capture.bin").open()
    assert text_file.closed


def test_exception_preserves_completed_records(tmp_path):
    jsonl_path, raw_path = tmp_path / "samples.jsonl", tmp_path / "capture.bin"
    with pytest.raises(KeyboardInterrupt):
        with Recorder(jsonl_path, raw_path=raw_path) as recorder:
            recorder.write(sample())
            recorder.write_raw(b"received")
            raise KeyboardInterrupt
    assert [json.loads(line) for line in jsonl_path.read_text(encoding="utf-8").splitlines()] == [
        sample().to_dict()
    ]
    assert raw_path.read_bytes() == b"received"
