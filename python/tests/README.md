# Testing

From the SDK's `python/` directory, with its virtual environment activated:

```sh
python -m pip install -e ".[dev]"
python -m pytest tests -q
python -m ruff check .
python -m ruff format --check --quiet .
```

Tests use synthetic frames, simulated devices and a POSIX pseudoterminal.
They do not open physical serial devices. The fixed protocol vectors in
`fixtures/` have explicit physical and wire expectations calculated independently
of the SDK decoder.

The Python workflow installs directly from source before testing Python 3.10–3.14 on
Windows, Ubuntu 22.04/24.04/26.04 and ARM Linux; selected versions also run on
macOS. A separate Ubuntu 22.04 job creates a virtual environment with
`/usr/bin/python3` (3.10), then installs directly from the SDK directory. It does
not use `setup-python`, so it exercises the customer installation path.
Debian Bookworm/Trixie ARM containers exercise 32/64-bit user space. These jobs
do not substitute for device/adapter tests on the target operating system.

To check the customer installation path, use a separate virtual environment,
run `python -m pip install .` (without `-e`), then run
`python tools/check_install.py`. It checks imports, CLI entry points and example
imports from outside the source checkout, without opening devices or recording
files. Examples expose `main()` but perform no I/O when imported; they use
editable constants rather than argument parsers. CLI help checks remain separate.
It also checks the installed Recorder and common Sample properties using a
temporary JSONL recording.

Focused failure-path checks include malformed ZDA calendar fields followed by a
valid binary frame, two consecutive Modbus reconnection failures on Python 3.10,
and a real SIGINT during the first recording callback of a multi-frame chunk.
The recording example must finish that decoded batch, restore the prior signal
handler, and still propagate disk or disconnect errors. Idle SIGINT uses a fake
clock to check the existing read-timeout bound. CLI command-file failures identify
the failed step on stderr without emitting incomplete JSON on stdout.

Passing simulations does not establish serial-adapter timing, persistence after
power loss, or configuration effects on physical devices.

`python tools/benchmark_decoder.py --frames 10000` measures decoding
against the fixed corpus. Add `--tracemalloc` to observe Python allocation peaks;
that mode also slows timing. Results describe the running CPU/interpreter and
exclude hardware, serial drivers and file recording.

## Optional packaging check

Packaging is checked once in CI on Ubuntu 24.04 / Python 3.11, together with lint
and formatting. No packages are uploaded or published. For a local check:

```sh
python -m pip install "build>=1,<2"
python -m build
```

The default build creates a source package, then builds a wheel from that package.
The source package includes documentation, four examples, tests and tools; the
wheel contains only the runtime package and installation metadata. Install the
wheel in a separate virtual environment and run `python tools/check_install.py`
to check its contents and entry points. Build output and `*.egg-info/` metadata
are generated files ignored by Git and do not need manual maintenance.
