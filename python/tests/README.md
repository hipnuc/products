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

To check the customer installation path, install the SDK without `-e` into a
separate virtual environment and run `python tools/check_install.py` from
outside the source checkout. CI runs the tests on Windows, Ubuntu 22.04 and
macOS with Python 3.10 and 3.14, plus the Ubuntu system-Python quick start.

Passing tests do not establish serial-adapter timing or the effect of a
configuration change on a physical device.
