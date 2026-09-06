"""Verify the installed distribution from outside the source checkout."""

from pathlib import Path
import subprocess
import sys
import tempfile
from textwrap import dedent


def main():
    with tempfile.TemporaryDirectory(prefix="hipnuc-install-") as directory:
        code = dedent(
            """
            import json
            from importlib.metadata import files, version
            from pathlib import Path
            import hipnuc

            assert 'site-packages' in Path(hipnuc.__file__).parts, hipnuc.__file__
            assert hipnuc.Decoder().feed(b'') == []
            assert hipnuc.ModbusBus
            assert hipnuc.SerialDevice().port is None
            assert not any(
                {'tests', 'examples'} & set(path.parts) for path in files('hipnuc-sdk')
            )
            sample = hipnuc.Sample('HI91', {'acceleration_m_s2': [1.0, 2.0, 3.0]}, b'raw')
            assert sample.acceleration_m_s2 == (1.0, 2.0, 3.0)
            assert sample.temperature_c is None
            path = Path('sample.jsonl')
            recorder = hipnuc.Recorder(path)
            assert not path.exists()
            with recorder:
                recorder.write(sample)
            assert recorder.samples_written == 1
            assert json.loads(path.read_text(encoding='utf-8')) == sample.to_dict()
            print(version('hipnuc-sdk'))
            print(hipnuc.__file__)
            """
        )
        subprocess.run([sys.executable, "-I", "-c", code], cwd=directory, check=True)
        for args in (
            ["--help"],
            ["help"],
            ["read", "--help"],
            ["scan", "--help"],
            ["command", "--help"],
            ["baudrate", "--help"],
            ["modbus", "read", "--help"],
            ["modbus", "baudrate", "--help"],
            ["modbus", "set-id", "--help"],
            ["modbus", "write-register", "--help"],
        ):
            subprocess.run(
                [sys.executable, "-I", "-m", "hipnuc", *args],
                cwd=directory,
                check=True,
                stdout=subprocess.DEVNULL,
            )
        executable = Path(sys.executable).parent / (
            "hipnuc.exe" if sys.platform == "win32" else "hipnuc"
        )
        subprocess.run([str(executable), "--version"], cwd=directory, check=True)
        examples = Path(__file__).resolve().parents[1] / "examples"
        assert {path.name for path in examples.glob("*.py")} == {
            "read_samples.py",
            "record_samples.py",
            "send_commands.py",
            "modbus_multinode.py",
        }
        import_check = dedent(
            """
            import os
            import runpy
            import sys
            from unittest.mock import patch
            import hipnuc

            def reject_file_writes(event, args):
                if event == 'open' and args[2] & (
                    os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND
                ):
                    raise AssertionError(f'Example import tried to write a file: {args[0]}')

            sys.addaudithook(reject_file_writes)
            error = AssertionError('Example import tried to access a device or recording')
            with (
                patch.object(hipnuc.SerialDevice, 'open', side_effect=error),
                patch.object(hipnuc.ModbusBus, 'open', side_effect=error),
                patch.object(hipnuc.Recorder, 'open', side_effect=error),
                patch('serial.Serial', side_effect=error),
                patch('serial.tools.list_ports.comports', side_effect=error),
            ):
                namespace = runpy.run_path(sys.argv[1], run_name='__example_check__')
                assert callable(namespace['main'])
            """
        )
        for example in sorted(examples.glob("*.py")):
            subprocess.run(
                [sys.executable, "-I", "-B", "-c", import_check, str(example)],
                cwd=directory,
                check=True,
                stdout=subprocess.DEVNULL,
            )
            print(f"Example import passed without I/O: {example.name}")


if __name__ == "__main__":
    main()
