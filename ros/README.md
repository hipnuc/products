# ROS drivers

[English](README.md) | [中文](README_zh.md)

Serial and SocketCAN drivers for HiPNUC IMU/AHRS/MRU/INS devices, using the
shared C decoder. Supported targets: ROS 2 Humble (Ubuntu 22.04), Jazzy (24.04),
Lyrical (26.04), and ROS 1 Noetic (20.04).

**Before starting, configure the device for ENU output with its default attitude
convention.** The driver does not verify or change device configuration.
Set `frame_id` to your sensor's body frame; the driver does not publish TF.

## ROS 2

With ROS installed, keep the complete repository inside your workspace:

```sh
source /opt/ros/jazzy/setup.bash   # use your installed distribution
mkdir -p ~/hipnuc_ws/src
cd ~/hipnuc_ws/src
git clone https://github.com/hipnuc/products.git
cd ..
rosdep install --from-paths $(colcon list --paths-only) --ignore-src -r -y
colcon build --packages-up-to hipnuc_imu
source install/setup.bash
ros2 launch hipnuc_imu serial.launch.py port:=/dev/ttyUSB0 baudrate:=115200
```

For CAN, configure your SocketCAN interface at the device's bitrate, then run:

```sh
ros2 launch hipnuc_imu can.launch.py interface:=can0 node_id:=8
```

## ROS 1

With Noetic installed, use a separate workspace from ROS 2. Keep the complete
checkout under `src`; the source selection below builds its ROS 1 packages:

```sh
source /opt/ros/noetic/setup.bash
mkdir -p ~/hipnuc_ros1_ws/src
cd ~/hipnuc_ros1_ws/src
git clone https://github.com/hipnuc/products.git
cd ..
rosdep install --from-paths src/products/ros/ros1/src --ignore-src -r -y
catkin_make --source src/products/ros/ros1/src
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0 baudrate:=115200
# Or: roslaunch hipnuc_imu can.launch interface:=can0 node_id:=8
```

## Messages

| Topic | Message | Contents |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | current acceleration, angular velocity and/or quaternion |
| `imu/mag` | `sensor_msgs/MagneticField` | magnetic field, T |
| `imu/temperature` | `sensor_msgs/Temperature` | temperature, °C |
| `hipnuc/imu` | `HipnucImu` | product fields, source, device time, status and presence bits |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | connection, receive rate and error counts |

Only quantities present in the current frame are published. Classic CAN
publishes each PGN independently; it does not wait for yaw or merge old fields.
An `Imu` covariance starting with `-1` marks a missing quantity; zero covariance
means unknown. Check whether your consumer accepts partial IMU messages and
provide application-specific uncertainty where required.

Headers carry host reception time from the ROS clock, not device sampling time.
Acceleration is specific force, including gravity. Navigation position and
velocity remain in the independent product message; standard navigation topics
are not provided. The product message is not a drop-in input for a generic
fusion node. Read only fields whose `VALID_*` bits are set; presence is separate
from convergence and GNSS fix status. Pressure is retained only as a raw product
field because its availability and freshness are not established by the protocol.
Its definition is in
[HipnucImu.msg](ros2/hipnuc_msgs/msg/HipnucImu.msg).

## Connection tips

- Edit the package's `config/serial.yaml` or `config/can.yaml` for topic switches
  and frame name. For ROS 2, rebuild after editing the source YAML files.
  Launch arguments override the connection parameters.
- For `Permission denied`, run `sudo usermod -aG dialout "$USER"`, then log out
  and back in. A virtual environment does not grant serial access.
- Prefer a path under `/dev/serial/by-id/` when several USB adapters are present.
  The driver retries an unavailable port/interface and continues publishing
  diagnostics, including when ROS simulated time is paused.
