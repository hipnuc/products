# Testing

From the SDK's `python/` directory, with its virtual environment activated:

```sh
python -m pip install -e ".[dev]"
python -m pytest tests -q
python -m ruff check .
python -m ruff format --check --quiet .
```

The tests use synthetic frames with hand-computed expected values, simulated
devices and a POSIX pseudoterminal; they never open a physical serial port.

To check the customer installation path, follow the README in a clean virtual
environment. In a fresh terminal, activate it again and run
`python tools/check_install.py`. It verifies the
installed SDK from outside the checkout, CLI help and quiet example imports.
It does not build release packages.

CI tests Windows and Ubuntu 24.04 with Python 3.10–3.14, macOS with 3.10 and
3.14, Ubuntu 26.04 with 3.14, ARM64 Linux with 3.12 and emulated ARMv7 Linux
with 3.11. Ubuntu 22.04 also exercises the system Python, pip upgrade and
installation in a virtual environment, with activation in each new shell.

Onboarding regressions cover USB-only discovery, explicit native UARTs,
connection-error classification, JSON output, and recording retries after a
failed connection. The examples also exercise Ctrl-C and file-error cleanup.

Passing tests do not establish serial-adapter timing or the effect of a
configuration change on a physical device.
