# ROS 驱动

[English](README.md) | [中文](README_zh.md)

面向 HiPNUC IMU/AHRS/MRU/INS 的串口与 SocketCAN 驱动，复用 C 协议库。
目标平台：ROS 2 Humble（Ubuntu 22.04）、Jazzy（24.04）、Lyrical（26.04），
以及 ROS 1 Noetic（20.04）。
Noetic 已结束上游维护，新项目建议使用 ROS 2。

**启动前，请将设备配置为 ENU 输出，并使用默认姿态约定。**
驱动不验证或修改设备配置。用 launch 参数 `frame_id` 指定传感器机体坐标系；驱动不发布 TF。

源码构建需要 ROS 2 的 colcon 和 rosdep，或 ROS 1 的 catkin_make 和 rosdep。
Ubuntu 的 ROS 2 开发工具可通过 `sudo apt install ros-dev-tools` 安装。
若这台电脑尚未初始化 rosdep，先执行一次 `sudo rosdep init`。

## ROS 2

安装 ROS 后，将完整仓库放入工作空间：

```sh
source /opt/ros/jazzy/setup.bash   # 改为已安装的发行版
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

使用 CAN 时，先按设备波特率配置 SocketCAN 接口，再运行：

```sh
ros2 launch hipnuc_imu can.launch.py interface:=can0 node_id:=8
```

## ROS 1

安装 Noetic 后，使用与 ROS 2 分开的工作空间。完整仓库仍放在 `src` 下，
以下命令通过指定源目录构建其中的 ROS 1 软件包：

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
# 或：roslaunch hipnuc_imu can.launch interface:=can0 node_id:=8
```

指定源目录是必需的：catkin 会跳过含 `COLCON_IGNORE` 标记的目录，因此直接执行
`catkin_make` 找不到 `ros/ros1`，反而会把 ROS 2 的软件包报成非 catkin 工作空间。
使用 catkin_tools 时同样先指定一次源目录：

```sh
catkin config --source-space src/products/ros/ros1/src && catkin build
```

## 消息

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | 加速度与角速度成对，或一个姿态 |
| `imu/mag` | `sensor_msgs/MagneticField` | 磁场，T |
| `imu/temperature` | `sensor_msgs/Temperature` | 温度，°C |
| `hipnuc/imu` | `HipnucImu` | 产品字段、来源、设备时间、状态与字段存在位 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 连接、接收速率和错误计数 |

只发布当前帧中存在的物理量，绝不把旧字段拼进新消息。`Imu` 中缺失的物理量会被填零并把
协方差首元素置为 `-1`，而忽略协方差的上层程序会把这些零当成测量值积分，因此
`imu/data` 只在一帧同时带有加速度和角速度、或带有姿态时才发布。串口输出和 CAN FD
本身就成对提供；Classic CAN 每个 PGN 只带一个物理量，因此那里的 `imu/data` 承载姿态，
其余字段都在 `hipnuc/imu` 中。全零协方差表示未知，需要不确定度时请按应用自行提供。

消息头使用 ROS 时钟标记的主机接收时间，不代表设备采样时间。
加速度为包含重力的比力。定位与速度保留在独立产品消息中，不提供标准导航话题。
产品消息不能直接替代通用融合节点的标准输入。只读取 `VALID_*` 位已设置的字段；
字段存在不代表姿态收敛或 GNSS 定位有效。气压仅保留为产品原始字段，
协议尚不能确认其可用性与数据新鲜度，因此不提供标准气压话题。完整定义见
[HipnucImu.msg](ros2/hipnuc_msgs/msg/HipnucImu.msg)。

## 连接提示

- `port`/`baudrate`（或 `interface`/`node_id`）和 `frame_id`（默认 `imu_link`）
  都是 launch 参数。软件包的 `config/serial.yaml` 与 `config/can.yaml` 只包含
  `publish_imu`、`publish_mag`、`publish_temperature`、`publish_hipnuc` 开关；
  ROS 2 修改源码中的 YAML 后需要重新构建。
- 驱动参数在启动时读取，修改后须重启节点；ROS 2 会拒绝运行时修改这些参数。
  可通过自己的 launch 文件或 ROS 重映射设置标准 namespace 和节点名称。
- ROS 2 的发布者 QoS 可以逐话题覆盖，无需重新编译，例如链路质量差时使用
  `--ros-args -p qos_overrides./imu/data.publisher.reliability:=best_effort`。
- 出现 `Permission denied` 时，执行 `sudo usermod -aG dialout "$USER"`，然后注销并重新登录。
  虚拟环境不会授予串口权限。
- 多个 USB 转接器并存时，优先使用 `/dev/serial/by-id/` 下的路径。
  端口或接口不可用时驱动会重试，并持续发布诊断；ROS 仿真时间暂停也不影响诊断检查。
