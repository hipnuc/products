"""Run installed-style CLI checks without opening any real device."""
from pathlib import Path
import subprocess
import sys
import tempfile


def run(executable, args, expected):
    with tempfile.TemporaryDirectory() as directory:
        result = subprocess.run(
            [str(executable), *args], cwd=directory, capture_output=True, text=True, timeout=10
        )
    assert result.returncode == expected, (args, result.returncode, result.stdout, result.stderr)
    return result


def main():
    build = Path(sys.argv[1]).resolve()
    name = "hipnuc-update.exe" if sys.platform == "win32" else "hipnuc-update"
    candidates = list(build.rglob(name))
    assert len(candidates) == 1, candidates
    executable = candidates[0]
    for args in ([], ["--help"], ["help"]):
        assert "Usage:" in run(executable, args, 0).stdout
    for args in (["image.hex"], ["image.hex", "-p", "FAKE", "-b", "0"],
                 ["image.hex", "-p", "FAKE", "-b", "115200x"], ["--unknown"]):
        assert "Error:" in run(executable, args, 2).stderr
    run(executable, ["no-such-file.hex", "-p", "FAKE", "-b", "115200"], 1)
    print("Serial updater: help, argument validation and missing-file failure passed")


if __name__ == "__main__":
    main()
