"""Synchronous JSONL sample and raw serial byte recording."""

from __future__ import annotations

import json
from pathlib import Path
from time import monotonic
from types import TracebackType
from typing import BinaryIO, TextIO

from .models import Sample


class Recorder:
    """Write samples and/or received serial chunks to local files.

    Construction does not open files. Use ``with`` or call ``open()`` explicitly.
    Files are created exclusively unless ``overwrite=True``. Parent directories
    must already exist. Writes to a disabled format are harmless, so ``write``
    and ``write_raw`` can both be used as device callbacks with either format.

    Files are flushed every second while writes are active, on ``flush()``, and
    when closed. No background work runs while the recorder is idle. Raw output
    must receive the original byte chunks, never the per-sample ``raw`` field:
    undecodable bytes and command replies have no measurement sample.
    File failures raise normal Python I/O exceptions.
    """

    def __init__(
        self,
        jsonl_path: str | Path | None = None,
        *,
        raw_path: str | Path | None = None,
        overwrite: bool = False,
    ) -> None:
        if jsonl_path is None and raw_path is None:
            raise ValueError("At least one recording path is required")
        self.jsonl_path = Path(jsonl_path) if jsonl_path is not None else None
        self.raw_path = Path(raw_path) if raw_path is not None else None
        self.overwrite = overwrite
        self.samples_written = 0
        self.raw_bytes_written = 0
        self._jsonl: TextIO | None = None
        self._raw: BinaryIO | None = None
        self._opened = False
        self._closed = False
        self._last_flush = 0.0

    def open(self) -> Recorder:
        """Open the configured outputs and return this recorder."""
        if self._closed:
            raise ValueError("Recorder is closed")
        if self._opened:
            return self
        paths = [path for path in (self.jsonl_path, self.raw_path) if path is not None]
        if len(paths) == 2:
            if paths[0].resolve() == paths[1].resolve() or (
                paths[0].exists() and paths[1].exists() and paths[0].samefile(paths[1])
            ):
                raise ValueError("JSONL and raw recording paths must be different")
        if not self.overwrite:
            for path in paths:
                if path.exists():
                    raise FileExistsError(f"Recording already exists: {path}")
        try:
            if self.jsonl_path is not None:
                self._jsonl = self.jsonl_path.open(
                    "w" if self.overwrite else "x", encoding="utf-8", newline="\n"
                )
            if self.raw_path is not None:
                self._raw = self.raw_path.open("wb" if self.overwrite else "xb")
        except BaseException:
            # Preserve the setup failure even if cleanup also encounters an error.
            try:
                self.close()
            except OSError:
                pass
            raise
        self._opened = True
        self._last_flush = monotonic()
        return self

    def __enter__(self) -> Recorder:
        return self.open()

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        if exc_type is None:
            self.close()
        else:
            # An acquisition/write error takes precedence over a later close error.
            try:
                self.close()
            except OSError:
                pass

    def _require_open(self) -> None:
        if not self._opened:
            raise ValueError("Recorder is not open")

    def _flush_if_due(self) -> None:
        if monotonic() - self._last_flush >= 1.0:
            self.flush()

    def write(self, sample: Sample) -> None:
        """Write one JSONL record; do nothing when JSONL output is disabled."""
        self._require_open()
        if self._jsonl is not None:
            record = json.dumps(sample.to_dict(), ensure_ascii=False, allow_nan=False)
            self._jsonl.write(record + "\n")
            self.samples_written += 1
            self._flush_if_due()

    def write_raw(self, data: bytes | bytearray | memoryview) -> None:
        """Write an original received byte chunk; do nothing without raw output."""
        self._require_open()
        if self._raw is not None:
            self._raw.write(data)
            self.raw_bytes_written += data.nbytes if isinstance(data, memoryview) else len(data)
            self._flush_if_due()

    def flush(self) -> None:
        """Flush both files; attempt both even if one fails."""
        self._require_open()
        failure = None
        for stream in (self._jsonl, self._raw):
            if stream is not None:
                try:
                    stream.flush()
                except OSError as error:
                    if failure is None:
                        failure = error
        if failure is not None:
            raise failure
        self._last_flush = monotonic()

    def close(self) -> None:
        """Flush and close both outputs; repeated closes are harmless."""
        failure = None
        for stream in (self._jsonl, self._raw):
            if stream is not None:
                try:
                    stream.close()
                except OSError as error:
                    if failure is None:
                        failure = error
        self._jsonl = None
        self._raw = None
        self._opened = False
        self._closed = True
        if failure is not None:
            raise failure
