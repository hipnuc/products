# ROS driver

[English](README.md) | [中文](README_zh.md)

ROS 2 (Humble, Jazzy, Lyrical) and ROS 1 (Noetic) driver for HiPNUC IMU, AHRS
and INS modules on a serial port. The driver never changes the device's
settings: on connecting it reads the model, firmware version and axes setting
once, then only receives. Firmware 1.6.9 or later is required.

## Quick start (ROS 2)

Serial access needs the `dialout` group: run `sudo usermod -aG dialout "$USER"`
once, then log out and back in.

With ROS 2 installed (`sudo apt install ros-dev-tools` provides colcon and
rosdep; run `sudo rosdep init` once on a new machine):

```sh
source /opt/ros/jazzy/setup.bash        # your distribution
mkdir -p ~/hipnuc_ws/src && cd ~/hipnuc_ws/src
git clone https://github.com/hipnuc/products.git
cd ~/hipnuc_ws
rosdep update && rosdep install --from-paths src/products/ros/hipnuc_imu --ignore-src -y
colcon build --packages-select hipnuc_imu
source install/setup.bash
ros2 launch hipnuc_imu serial.launch.py port:=/dev/ttyUSB0
```

If `rosdep update` cannot reach its online index, skip the rosdep line: a
standard ROS installation (ros-base or desktop) already has every dependency.
The same applies to ROS 1 below.

The driver finds the baud rate itself, trying `baudrate` (default 115200)
first, and logs the model, firmware and baud rate it found. In a second
terminal, run `source ~/hipnuc_ws/install/setup.bash` and check the data:

```sh
ros2 topic hz /imu/data
ros2 topic echo /diagnostics
```

With several USB adapters, use the stable name under `/dev/serial/by-id/` as
`port`.

## Axes and TF

The driver publishes the data in the axes the device outputs; it does not rotate
them. `frame_id` (default `imu_link`) names these axes, and a static transform
from your robot frame to `imu_link` tells ROS how the module sits.

To see which axes the device outputs, hold it still, standing on the end its Y
arrow points to (Y arrow down), and watch
`ros2 topic echo /imu/data --field linear_acceleration`:

| Acceleration with the Y arrow down | Output axes | Transform `base_link` → `imu_link`, module level with the Y arrow forward |
| --- | --- | --- |
| y about −9.8 | x right, y forward, z up (factory) | yaw −90° |
| x about −9.8 | x forward, y left, z up (ROS) | none |

Recent firmware also logs the axes setting at startup. Setting `COORD 4` gives
ROS axes only on some recent firmware; elsewhere the data keep the factory
axes, so check the result as above.
Publish the transform with your other static transforms, for example:

```sh
ros2 run tf2_ros static_transform_publisher --yaw -1.5708 --frame-id base_link --child-frame-id imu_link
```

Add the mounting offset (`--x --y --z`) and any further rotation to the same
transform, or give the `imu_link` joint in your URDF the same values.

Orientation is reported against the device's reference frame. In 6-axis mode,
heading starts from the power-up direction, so with the −90° transform the robot
starts at a heading of 90°; robot_localization's `imu0_relative` starts it at
zero. 9-axis mode with the factory axes references east-north-up, as ROS
expects; with `COORD 4` most firmware references north-west-up instead.

## SLAM

**Rate.** Most SLAM front ends want 200 Hz or more; the factory output is
100 Hz at 115200 baud, and the driver logs the commands below when it measures
less than 200 Hz. Output periods are whole milliseconds, so use 200 Hz
(`0.005`) or 500 Hz (`0.002`); 400 Hz is not available. HI91 needs 230400 baud
for 200 Hz and 921600 for 500 Hz. Stop the node, then install and run the
Python CLI `hihost` (see [python/](../python/README.md)):

```sh
hihost baudrate 230400 -p /dev/ttyUSB0 --save
hihost command "LOG HI91 ONTIME 0.005" -p /dev/ttyUSB0 -b 230400 --save
```

The device keeps these settings after power-off. The driver finds the new baud
rate, so the launch command stays the same.

**Timestamps.** A message's header stamp is the moment the device began sending
the frame: the host receive time minus the frame's time on the wire (7.1 ms for
HI91 at 115200, 0.9 ms at 921600). Stamps on `imu/data` always increase. The
USB adapter and the device itself add a small further delay; FAST-LIO and
Point-LIO take a lidar-to-IMU time offset if you need to align more closely.

**Extrinsics.** FAST-LIO, Point-LIO and LIO-SAM read the rotation between IMU
and lidar from their own configuration, not from TF. FAST-LIO and Point-LIO
`extrinsic_R` turns lidar vectors into IMU axes; LIO-SAM `extrinsicRot` turns
IMU vectors into lidar axes. For a lidar mounted with x forward and z up:

| Device axes | FAST-LIO / Point-LIO `extrinsic_R` | LIO-SAM `extrinsicRot` and `extrinsicRPY` |
| --- | --- | --- |
| x right, y forward, z up | `[0, -1, 0, 1, 0, 0, 0, 0, 1]` | `[0, 1, 0, -1, 0, 0, 0, 0, 1]` |
| x forward, y left, z up | identity | identity |

**Per package.**

- Point-LIO: set `acc_norm` to 9.81 (its default 1.0 expects g) and raise
  `satu_acc` to the accelerometer range in m/s².
- LIO-SAM: in 6-axis mode set `useImuHeadingInitialization: false`.
- Cartographer: set `tracking_frame` to `imu_link`.
- robot_localization: it treats zero covariance as almost exact, so fuse only
  the quantities you need, for example yaw rate.

## Topics and parameters

| Topic | Type | Default |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | on |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | on |
| `imu/data_raw` | `sensor_msgs/Imu` without orientation | `publish_data_raw` |
| `imu/mag` | `sensor_msgs/MagneticField`, T (models with a magnetometer) | `publish_mag` |
| `imu/temperature` | `sensor_msgs/Temperature`, °C | `publish_temperature` |
| `hipnuc/imu` | `hipnuc_imu/HipnucImu`, every decoded field | `publish_hipnuc` |

`imu/data` is published for every frame with both acceleration and angular
velocity. Acceleration is specific force in m/s² (about +9.8 on the up axis at
rest), angular velocity is rad/s. Covariances are all zero, meaning unknown; an
orientation covariance starting with −1 means the frame has no orientation.
`/diagnostics` shows the model, firmware, baud rate, frame rate, error counts
and device status.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | serial port |
| `baudrate` | `115200` | tried first; the other common baud rates follow |
| `frame_id` | `imu_link` | frame of all messages |
| `publish_*` | see above | optional topics |

Parameters are read at startup. The launch file takes `port`, `baudrate`,
`frame_id` and `params_file`; these arguments always set the connection and
`frame_id`, and the file holds the `publish_*` switches. For your own bringup,
copy `ros/hipnuc_imu/config/serial.yaml` from the repository, edit it and pass
its path:

```sh
ros2 launch hipnuc_imu serial.launch.py params_file:=/path/to/imu.yaml
```

The data topics are reliable. QoS overrides such as depth are accepted at
startup, but keep them reliable: reliable subscribers, such as FAST-LIO for
ROS 2, receive nothing from a best-effort publisher.

`hipnuc/imu` carries Euler angles, device status, device time and UTC, and the
INS position and velocity of INS models. Read a field only when its `VALID_*`
bit is set in `valid`. Some robot kits ship an older driver also named
`hipnuc_imu` (topic `/IMU_data`); remove it from the workspace first.

**Upgrading from 2.x.** The node is now `hipnuc_imu_node` (formerly
`serial_node`) and the message type `hipnuc_imu/msg/HipnucImu` (formerly
`hipnuc_msgs/msg/HipnucImu`); `imu/mag`, `imu/temperature` and `hipnuc/imu`
are off until you switch them on. The package directory moved, so in an
existing ROS 2 workspace delete `build/hipnuc_imu` and `install/hipnuc_imu`
before building.

The ROS driver supports serial connections only. For CAN models, use the C
example [read_can.c](../c/examples/read_can.c), `hihost can read`, or
ros2_socketcan to receive raw frames.

## ROS 1 (Noetic)

Use a separate workspace (`sudo apt install git python3-rosdep` provides the
tools; run `sudo rosdep init` once on a new machine). The source directory below
is required: catkin skips directories marked for colcon.

```sh
source /opt/ros/noetic/setup.bash
mkdir -p ~/hipnuc_ros1_ws/src && cd ~/hipnuc_ros1_ws/src
git clone https://github.com/hipnuc/products.git
cd ~/hipnuc_ros1_ws
rosdep update --include-eol-distros     # Noetic is end-of-life
rosdep install --from-paths src/products/ros/ros1/src --ignore-src -y
catkin_make --source src/products/ros/ros1/src
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0
```

Topics, parameters and messages are the same as in ROS 2; the static transform
is `rosrun tf2_ros static_transform_publisher 0 0 0 -1.5708 0 0 base_link imu_link`
(on ros-base, `sudo apt install ros-noetic-tf2-ros` first).

## Troubleshooting

The node reports its state in the log and on `/diagnostics`.

| Message | What to do |
| --- | --- |
| `cannot open ... Permission denied` | Join the `dialout` group and log in again. |
| `cannot open ... No such file or directory` | Check the cable; list ports with `ls /dev/serial/by-id`. |
| `cannot open ... busy` or `in use` | Close the other program using the port, such as CHCenter or `hihost`. |
| Warnings in the first 20 s or so after plugging in | ModemManager may be probing the port; the driver reconnects by itself. |
| `no data` | Check the wiring and that the device output is on. |
| `no valid frames at any baud rate` | Make the device output HI91, HI83 or HI81. |
| `imu/data stays silent` | Add acceleration and angular velocity to the HI83 fields (MAP bits 0 and 1). |
| `imu/data runs at ... Hz` | Raise the rate as shown under [SLAM](#slam) if your stack needs it. |
| Orientation is 90° off | Use the transform from [Axes and TF](#axes-and-tf). |
