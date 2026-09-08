"""ROS 2 integration checks using a pseudo-terminal, never physical hardware."""
import binascii
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None


def binary_sample(bitmap, vector):
    payload = struct.pack("<BHBI3f", 0x83, 0, 0, bitmap, *vector)
    header = b"\x5a\xa5" + struct.pack("<H", len(payload))
    return header + struct.pack("<H", binascii.crc_hqx(header + payload, 0)) + payload


@unittest.skipUnless(rclpy is not None, "requires a sourced ROS 2 installation")
class RuntimeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.observer = rclpy.create_node("hipnuc_runtime_test")
        self.diagnostics = []
        self.samples = []
        self.observer.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostics.append, 10
        )
        self.observer.create_subscription(Imu, "imu/data", self.samples.append, 100)
        self.process = None
        self.output = tempfile.TemporaryFile(mode="w+")

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.output.close()
        self.observer.destroy_node()
        rclpy.shutdown()

    def start(self, executable, *parameters):
        command = ["ros2", "run", "hipnuc_imu", executable, "--ros-args",
                   "-p", "use_sim_time:=true"]
        for parameter in parameters:
            command.extend(["-p", parameter])
        self.process = subprocess.Popen(
            command, stdout=self.output, stderr=subprocess.STDOUT, start_new_session=True
        )

    def wait_for(self, predicate, timeout=8, send=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.observer, timeout_sec=0.05)
            if predicate():
                return
            if self.process.poll() is not None:
                self.output.seek(0)
                self.fail(self.output.read())
            if send is not None:
                send()
        self.output.seek(0)
        self.fail("condition timed out\n" + self.output.read())

    def status(self):
        if self.diagnostics:
            return self.diagnostics[-1].status[0]
        return None

    def test_serial_failure_recovery_partial_samples_and_disconnect(self):
        with tempfile.TemporaryDirectory() as directory:
            port = Path(directory) / "sensor"
            self.start("serial_node", f"port:={port}", "baudrate:=115200")
            self.wait_for(lambda: self.status() is not None and self.status().level == 2)
            # Diagnostics progress even while simulated ROS time stays at zero.
            self.assertEqual(self.diagnostics[-1].header.stamp.sec, 0)
            first, second = os.openpty()
            try:
                port.symlink_to(os.ttyname(second))
                self.wait_for(lambda: self.status().level == 1)
                acceleration = binary_sample(1, (1.25, 2.5, 9.8))
                gyro = binary_sample(2, (0.125, 0.25, 0.5))
                self.wait_for(
                    lambda: len(self.samples) >= 2,
                    send=lambda: os.write(first, acceleration + gyro),
                )
                acc = next(m for m in self.samples if m.linear_acceleration_covariance[0] == 0)
                gyr = next(m for m in self.samples if m.angular_velocity_covariance[0] == 0)
                self.assertEqual(acc.linear_acceleration.x, 1.25)
                self.assertEqual(acc.angular_velocity_covariance[0], -1)
                self.assertEqual(gyr.linear_acceleration_covariance[0], -1)
                self.assertEqual(gyr.orientation_covariance[0], -1)
                self.assertEqual(gyr.angular_velocity.x, 0.125)
                self.wait_for(lambda: self.status().level == 0)
                self.wait_for(lambda: self.status().level == 1, timeout=5)
                os.close(first)
                first = None
                self.wait_for(lambda: self.status().level == 2)
                topics = dict(self.observer.get_topic_names_and_types())
                self.assertNotIn("/gnss/fix", topics)
                self.assertNotIn("/ins/velocity", topics)
                self.assertNotIn("/imu/pressure", topics)
            finally:
                if first is not None:
                    os.close(first)
                os.close(second)

    def test_can_open_failure_keeps_diagnostics_alive(self):
        self.start("can_node", "interface:=hipnuc_missing")
        self.wait_for(lambda: len(self.diagnostics) >= 3)
        self.assertTrue(all(item.status[0].level == 2 for item in self.diagnostics))
        self.assertEqual(self.diagnostics[-1].header.stamp.sec, 0)
        self.assertNotIn("/imu/pressure", dict(self.observer.get_topic_names_and_types()))


if __name__ == "__main__":
    unittest.main()
