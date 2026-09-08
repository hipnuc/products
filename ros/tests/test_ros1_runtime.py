"""Installed ROS 1 nodes with a pseudo-terminal and an isolated ROS master."""
import binascii
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
import xmlrpc.client

try:
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import Imu
except ImportError:
    rospy = None


@unittest.skipUnless(rospy is not None, "requires a sourced ROS 1 installation")
class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as available:
            available.bind(("127.0.0.1", 0))
            port = available.getsockname()[1]
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
        cls.output = tempfile.TemporaryFile()
        cls.master = subprocess.Popen(
            ["roscore", "-p", str(port)], stdout=cls.output, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        cls.addClassCleanup(cls.stop_process, cls.master)
        cls.addClassCleanup(cls.output.close)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                with xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]) as server:
                    if server.getPid("/hipnuc_runtime_test")[0] == 1:
                        break
            except OSError:
                time.sleep(0.1)
        else:
            raise AssertionError("ROS master did not start")
        rospy.init_node("hipnuc_runtime_test", disable_signals=True)
        rospy.set_param("/use_sim_time", True)

    @staticmethod
    def stop_process(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def test_installed_serial_node_failure_recovery_and_partial_imu(self):
        diagnostics, samples = [], []
        diag_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, diagnostics.append)
        imu_sub = rospy.Subscriber("imu/data", Imu, samples.append)
        self.addCleanup(diag_sub.unregister)
        self.addCleanup(imu_sub.unregister)
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile() as output:
            path = Path(directory) / "port"
            node = subprocess.Popen(
                ["rosrun", "hipnuc_imu", "serial_node", "_port:=" + str(path), "_baudrate:=115200"],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
            )
            self.addCleanup(self.stop_process, node)

            def wait_for(predicate, send=None):
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    if predicate():
                        return
                    self.assertIsNone(node.poll(), "serial node exited")
                    if send:
                        send()
                    time.sleep(0.05)
                output.seek(0)
                self.fail(output.read().decode(errors="replace"))

            wait_for(lambda: diagnostics and diagnostics[-1].status[0].level == 2)
            self.assertEqual(diagnostics[-1].header.stamp.to_sec(), 0)
            master, slave = os.openpty()
            try:
                path.symlink_to(os.ttyname(slave))
                payload = struct.pack("<BHBI3f", 0x83, 0, 0, 1, 1.25, 2.5, 9.8)
                header = b"\x5a\xa5" + struct.pack("<H", len(payload))
                frame = header + struct.pack("<H", binascii.crc_hqx(header + payload, 0)) + payload
                wait_for(lambda: samples, lambda: os.write(master, frame))
                self.assertEqual(samples[-1].linear_acceleration.x, 1.25)
                self.assertEqual(samples[-1].angular_velocity_covariance[0], -1)
                self.assertEqual(samples[-1].orientation_covariance[0], -1)
                os.close(master)
                master = None
                previous = len(diagnostics)
                wait_for(lambda: len(diagnostics) > previous and diagnostics[-1].status[0].level == 2)
                self.assertNotIn("/imu/pressure", dict(rospy.get_published_topics()))
            finally:
                if master is not None:
                    os.close(master)
                os.close(slave)


if __name__ == "__main__":
    unittest.main()
