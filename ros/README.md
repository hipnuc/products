# ROS drivers

[English](README.md) | [中文](README_zh.md)

Serial and SocketCAN drivers for HiPNUC IMU/AHRS/MRU/INS devices, using the
shared C decoder. Supported targets: ROS 2 Humble (Ubuntu 22.04), Jazzy (24.04),
Lyrical (26.04), and ROS 1 Noetic (20.04).
Noetic has reached upstream end of life; use ROS 2 for new projects.

**Before starting, configure the device for ENU output with its default attitude
convention.** The driver does not verify or change device configuration.
Set the `frame_id` launch argument to your sensor's body frame; the driver
does not publish TF.

Source builds need colcon and rosdep for ROS 2, or catkin_make and rosdep for ROS 1.
On Ubuntu, install the ROS 2 development tools with `sudo apt install ros-dev-tools`.
If rosdep has never been initialized on this machine, run `sudo rosdep init` once.

## ROS 2

With ROS installed, keep the complete repository inside your workspace:

```sh
source /opt/ros/jazzy/setup.bash   # use your installed distribution
mkdir -p ~/hipnuc_ws/src
cd ~/hipnuc_ws/src
git clone https://github.com/hipnuc/products.git
cd ..
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths $(colcon list --paths-only) --ignore-src -r -y
colcon build --packages-up-to hipnuc_imu
source install/setup.bash
ros2 launch hipnuc_imu serial.launch.py port:=/dev/ttyUSB0 baudrate:=115200
```

For CAN, configure your SocketCAN interface at the device's bitrate, then run:

```sh
ros2 launch hipnuc_imu can.launch.py interface:=can0 node_id:=8
```

For an existing ROS 2 robot workspace, place the complete checkout under its
`src` directory and use the same dependency and build commands above.

To set ROS 2 data publisher QoS from node arguments, start the node directly:

```sh
ros2 run hipnuc_imu serial_node --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=115200 \
  -p qos_overrides./imu/data.publisher.reliability:=best_effort
```

Data topics support `history`, `depth` and `reliability` overrides at startup;
`/diagnostics` does not. The key must use the final topic name after namespaces
and remapping, such as `/robot1/imu/data`. These node arguments cannot be
appended to `ros2 launch`; put them in your own parameter file or launch file.
Subscribers must allow best-effort delivery when the publisher uses it;
a subscription requiring reliable delivery cannot receive from that publisher.

## ROS 1

With Noetic installed, use a separate workspace from ROS 2. Keep the complete
checkout under `src`; the source selection below builds its ROS 1 packages:

```sh
source /opt/ros/noetic/setup.bash
mkdir -p ~/hipnuc_ros1_ws/src
cd ~/hipnuc_ros1_ws/src
git clone https://github.com/hipnuc/products.git
cd ..
rosdep update --rosdistro noetic --include-eol-distros
rosdep install --from-paths src/products/ros/ros1/src --ignore-src -r -y
catkin_make --source src/products/ros/ros1/src
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0 baudrate:=115200
# Or: roslaunch hipnuc_imu can.launch interface:=can0 node_id:=8
```

The explicit source directory is required, not a shortcut: catkin skips any
directory holding a `COLCON_IGNORE` marker, so a plain `catkin_make` does not
find `ros/ros1` and instead reports the ROS 2 packages as a non-catkin
workspace. With catkin_tools, set the same directory once:

```sh
catkin config --source-space src/products/ros/ros1/src && catkin build
```

To use the driver from an existing robot workspace, keep the SDK in the separate
workspace above and use it as an underlay before rebuilding your robot packages:

```sh
source ~/hipnuc_ros1_ws/devel/setup.bash
cd ~/robot_ws
catkin_make --force-cmake
source devel/setup.bash
```

With catkin_tools, use `catkin config --extend ~/hipnuc_ros1_ws/devel` before
`catkin build` in the robot workspace.

## Messages

| Topic | Message | Contents |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | available acceleration (m/s²), angular velocity (rad/s) and orientation |
| `imu/mag` | `sensor_msgs/MagneticField` | magnetic field, T |
| `imu/temperature` | `sensor_msgs/Temperature` | temperature, °C |
| `hipnuc/imu` | `HipnucImu` | product fields, source, device time, status and presence bits |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | connection, receive rate and error counts |

An `Imu` is published when the current frame or PGN provides any valid
acceleration, angular velocity or quaternion. Missing quantities have covariance
element 0 set to `-1` and must be ignored; an all-zero covariance means unknown.
Fields from different PGNs are never combined. Classic CAN can therefore publish
partial IMU messages; magnetic field and temperature still use their standard
topics when available. Acceleration is specific force, including gravity.

`imu_filter_madgwick` and other filters requiring complete inertial input need
valid acceleration and angular velocity together in each input message. Select
an output format that supplies both; a single-quantity PGN is insufficient.
Keep filter input and output topics distinct to prevent feedback. Unknown
covariance is not enough for direct fusion: provide uncertainty appropriate to
the application and consumer.

Headers use the host ROS clock after decoding and before publication, not kernel
reception or device sampling timestamps. `use_sim_time` defaults to `false`;
enabling it requires a valid `/clock`. Transport and scheduling delays remain,
so assess time synchronization for your fusion application.

Navigation position and velocity remain in the independent product message;
standard navigation topics are not provided. The product message is not a
drop-in input for a generic fusion node. Read only fields whose `VALID_*` bits
are set; presence is separate
from convergence and GNSS fix status. Pressure is retained only as a raw product
field because its availability and freshness are not established by the protocol.
The product type is
[`hipnuc_msgs/msg/HipnucImu`](ros2/hipnuc_msgs/msg/HipnucImu.msg) in ROS 2 and
[`hipnuc_imu/HipnucImu`](ros1/src/hipnuc_imu/msg/HipnucImu.msg) in ROS 1.
After an SDK update that changes `HipnucImu`, rebuild the driver and all packages
that use this message.

## Connection tips

- `port`/`baudrate` (or `interface`/`node_id`) and `frame_id` (default
  `imu_link`) are launch arguments. `params_file` defaults to the package's
  `config/serial.yaml` or `config/can.yaml`, which holds the `publish_imu`,
  `publish_mag`, `publish_temperature` and `publish_hipnuc` switches.
- Driver parameters are read at startup; restart the node after changing them.
  ROS 2 rejects runtime changes to these parameters. Standard ROS namespaces
  and node renaming are supported through your launch file or ROS remapping.
- For `Permission denied`, run `sudo usermod -aG dialout "$USER"`, then log out
  and back in. A virtual environment does not grant serial access.
- Prefer a path under `/dev/serial/by-id/` when several USB adapters are present.
  The driver retries an unavailable port/interface and continues publishing
  diagnostics, including when ROS simulated time is paused.

To keep deployment settings in your own robot bringup package, copy the matching
YAML from the ROS generation you use, edit it, and pass its absolute path:

```sh
ros2 launch hipnuc_imu serial.launch.py params_file:=/path/to/bringup/config/imu.yaml
# ROS 1:
roslaunch hipnuc_imu serial.launch params_file:=/path/to/bringup/config/imu.yaml
```

The CAN launch files accept `params_file` in the same way. Connection settings
and `frame_id` come from launch arguments, including their defaults, and override
matching YAML entries. Restart the node after editing your parameter file;
rebuilding the driver package is unnecessary.
