# ROS drivers

[English](README.md) | [中文](README_zh.md)

Thin ROS packages on top of the C core in `c/hipnuc`. Both publish the same
topics; `ros2/` targets Humble and Jazzy, `ros1/` targets Noetic.

| Topic | Type | When |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | every frame with acceleration, gyro or quaternion |
| `imu/mag` | `sensor_msgs/MagneticField` | magnetometer present |
| `imu/temperature`, `imu/pressure` | `sensor_msgs/Temperature`, `FluidPressure` | present |
| `gnss/fix` | `sensor_msgs/NavSatFix` | INS/GNSS position present (altitude is ellipsoid height, NaN when the geoid separation is unknown) |
| `ins/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | ENU velocity present, `frame_id` = `enu_frame_id` |
| `hipnuc/imu` | `hipnuc_msgs/HipnucImu` (ROS 1: `hipnuc_imu/HipnucImu`) | every frame; all fields in SI with a validity mask |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | once per second: connection, frame rate, CRC/invalid counts |

Stamps use the node clock. Covariances are zero (unknown) or `-1` in the first
element when the quantity is not provided. Conventions are documented in
`ros/common/hipnuc_convert.hpp`.

## ROS 2

```sh
cd ros/ros2
colcon build
source install/setup.bash
ros2 launch hipnuc_imu serial.launch.py port:=/dev/ttyUSB0 baudrate:=115200
ros2 launch hipnuc_imu can.launch.py interface:=can0 node_id:=8
```

## ROS 1

```sh
cd ros/ros1
catkin_make
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0 baudrate:=115200
roslaunch hipnuc_imu can.launch interface:=can0 node_id:=8
```

## Notes

- Parameters live in `config/serial.yaml` and `config/can.yaml`; launch
  arguments override port/baudrate and interface/node_id.
- Serial access needs the `dialout` group; `99-hipnuc.rules` (in the package)
  gives a stable `/dev/hipnuc` name for the evaluation board.
- The CAN node merges the J1939 PGNs of one device (`node_id`) and publishes
  when `trigger_pgn` (default yaw, 0xFF41) arrives; CANFD83 frames publish
  directly. Frames from other source addresses are counted, not published.
- To use the packages in your own workspace, copy `ros/common` and the
  package, and set `HIPNUC_CORE_DIR` to the `c/hipnuc` directory.
