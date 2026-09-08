"""Keep both ROS message schemas aligned with the C field-presence contract."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]


class SchemaTests(unittest.TestCase):
    def test_message_versions_match_and_preserve_all_presence_bits(self):
        first = (ROOT / "ros/ros1/src/hipnuc_imu/msg/HipnucImu.msg").read_text()
        second = (ROOT / "ros/ros2/hipnuc_msgs/msg/HipnucImu.msg").read_text()
        self.assertEqual(first, second)
        header = (ROOT / "c/hipnuc/hipnuc_sample.h").read_text()
        bits = {
            name: 1 << int(bit)
            for name, bit in re.findall(
                r"#define HIPNUC_VALID_(\w+)\s+\(UINT64_C\(1\) << (\d+)\)", header
            )
        }
        self.assertGreater(len(bits), 32)
        constants = {
            name: int(value)
            for name, value in re.findall(r"uint64 VALID_(\w+)=(\d+)", second)
        }
        self.assertEqual(constants, bits)
        self.assertIn("uint64 valid", second)

    def test_standard_pressure_is_deferred_but_product_field_remains(self):
        for generation in ("ros/ros1/src/hipnuc_imu", "ros/ros2/hipnuc_imu"):
            package = ROOT / generation
            for path in sorted((package / "src").glob("*_node.cpp")):
                source = path.read_text()
                with self.subTest(path=path):
                    self.assertNotIn("FluidPressure", source)
                    self.assertNotIn("fluid_pressure", source)
                    self.assertNotIn("imu/pressure", source)
                    self.assertNotIn("publish_temperature_pressure", source)
                    self.assertIn('"publish_temperature"', source)
            for path in sorted((package / "config").glob("*.yaml")):
                source = path.read_text()
                self.assertNotIn("publish_temperature_pressure", source)
                self.assertIn("publish_temperature:", source)
        schema = (ROOT / "ros/ros2/hipnuc_msgs/msg/HipnucImu.msg").read_text()
        self.assertRegex(schema, r"(?m)^float32 pressure\s")
        converter = (ROOT / "ros/common/hipnuc_convert.hpp").read_text()
        self.assertNotIn("fill_pressure", converter)


if __name__ == "__main__":
    unittest.main()
