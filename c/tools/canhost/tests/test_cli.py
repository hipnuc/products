import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = Path(sys.argv.pop()).resolve()


class CliTests(unittest.TestCase):
    def run_cli(self, *arguments, scene="stream", cwd=None):
        environment = dict(os.environ, CANHOST_TEST_SCENE=scene)
        return subprocess.run(
            [str(BINARY), *arguments], env=environment, cwd=cwd,
            text=True, capture_output=True, timeout=5,
        )

    def test_help_and_invalid_arguments_never_open_interface(self):
        for arguments in [(), ("--help",), ("reg", "--help"), ("read", "--help")]:
            result = self.run_cli(*arguments)
            self.assertEqual(result.returncode, 0)
            self.assertNotIn("FAKE_OPEN", result.stderr)
        cases = [
            ("read",), ("device", "list"), ("-i", "fake0", "read"),
            ("read", "-i", "fake0", "--count", "-1"),
            ("read", "-i", "fake0", "--count", "18446744073709551616"),
            ("read", "-i", "fake0", "-n", "256"),
            ("read", "-i", "fake0", "--duration", "nan"),
            ("read", "-i", "fake0", "--duration", "inf"),
            ("read", "-i", "fake0", "--overwrite"),
            ("read", "-i", "fake0", "--interval", "1"),
            ("reg", "write", "1", "4294967296", "-i", "fake0", "-n", "8"),
            ("reg", "read", "1", "-i", "fake0", "-n", "255"),
            ("reg", "read", "1", "-i", "fake0", "-n", "85"),
            ("update", "image.bin", "-i", "fake0", "-n", "128", "--bin"),
            ("sync", "0xEF01", "-i", "fake0", "-n", "8"),
        ]
        for arguments in cases:
            with self.subTest(arguments=arguments):
                result = self.run_cli(*arguments)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn("FAKE_OPEN", result.stderr)

    def test_complete_receive_batch_and_full_source_address(self):
        result = self.run_cli("read", "-i", "fake0", "--count", "1")
        self.assertEqual(result.returncode, 0, result.stderr)
        records = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(records), 5)
        self.assertEqual([item["node_id"] for item in records], [8, 250, 8, 255, 8])
        self.assertAlmostEqual(records[0]["acceleration_m_s2"][0], 9.8, places=5)
        self.assertEqual(records[0]["rx_time_us"], 1700000000000000)
        result = self.run_cli("read", "-i", "fake0", "-n", "255", "--count", "1")
        self.assertEqual(result.returncode, 0)
        self.assertEqual(len(result.stdout.splitlines()), 1)

    def test_record_protection_flush_close_and_interrupt(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "samples.jsonl"
            path.write_text("keep")
            (Path(directory) / "canhost.ini").write_text("interface=should-not-load")
            result = self.run_cli("read", "-i", "fake0", "--record", str(path), "--count", "1", cwd=directory)
            self.assertEqual(result.returncode, 1)
            self.assertNotIn("FAKE_OPEN", result.stderr)
            self.assertEqual(path.read_text(), "keep")
            result = self.run_cli("read", "-i", "fake0", "--record", str(path), "--overwrite",
                                  "--count", "1", scene="interrupt")
            self.assertEqual(result.returncode, 130, result.stderr)
            self.assertEqual(len(path.read_text().splitlines()), 5)
            self.assertEqual(result.stdout, "")
            self.assertIn("FAKE_CLOSE", result.stderr)
            result = self.run_cli("read", "-i", "fake0", "--record", str(path), "--overwrite",
                                  "--count", "1", scene="close_error")
            self.assertEqual(result.returncode, 1)
            self.assertIn("close failed", result.stderr)
        result = self.run_cli("read", "-i", "fake0", "--record", "/dev/full", "--overwrite", "--count", "1")
        self.assertEqual(result.returncode, 1)
        self.assertIn("flush failed", result.stderr)

    def test_no_data_and_receive_failure(self):
        for scene in ("idle", "receive_error", "open_error"):
            result = self.run_cli("read", "-i", "fake0", "--duration", "0.01", scene=scene)
            self.assertEqual(result.returncode, 1, result.stderr)

    def test_passive_scan_and_sync(self):
        result = self.run_cli("scan", "-i", "fake0")
        self.assertEqual(result.returncode, 0)
        self.assertEqual(len(result.stdout.splitlines()), 3)
        self.assertIn("node=255", result.stdout)
        result = self.run_cli("sync", "0xFF34", "-i", "fake0", "-n", "8", "--count", "2")
        self.assertEqual(result.returncode, 0)
        self.assertIn("Sent 2 trigger requests", result.stderr)
        result = self.run_cli("sync", "0xFF34", "-i", "fake0", "-n", "8", "--count", "2", scene="send_error")
        self.assertEqual(result.returncode, 1)

    def test_register_matching_rejection_and_echo(self):
        result = self.run_cli("reg", "read", "0", "-i", "fake0", "-n", "8")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("value=42", result.stdout)
        for scene in ("reg_error", "reg_other_host", "reg_other_node"):
            result = self.run_cli("reg", "read", "0", "-i", "fake0", "-n", "8", scene=scene)
            self.assertEqual(result.returncode, 1, result.stderr)
        result = self.run_cli("reg", "write", "10", "32", "-i", "fake0", "-n", "8", scene="reg_echo_error")
        self.assertEqual(result.returncode, 1)

    def test_update_end_conditions_and_interrupt(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.bin"
            path.write_bytes(bytes(range(21)))
            for scene, expected in [("stream", 0), ("goto_timeout", 0),
                                    ("goto_receive_error", 1), ("goto_send_error", 1),
                                    ("goto_abort", 1), ("update_interrupt", 130)]:
                with self.subTest(scene=scene):
                    result = self.run_cli("update", str(path), "-i", "fake0", "-n", "8", "--bin", scene=scene)
                    self.assertEqual(result.returncode, expected, result.stderr)
                    self.assertEqual(result.stdout, "")
                    self.assertIn("FAKE_CLOSE", result.stderr)


if __name__ == "__main__":
    unittest.main()
