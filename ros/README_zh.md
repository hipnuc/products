# ROS 驱动

[English](README.md) | [中文](README_zh.md)

面向 HiPNUC IMU/AHRS/MRU/INS 的串口与 SocketCAN 驱动，复用 C 协议库。
目标平台：ROS 2 Humble（Ubuntu 22.04）、Jazzy（24.04）、Lyrical（26.04），
以及 ROS 1 Noetic（20.04）。
Noetic 已结束上游维护，新项目建议使用 ROS 2。

**启动前，请将设备配置为 ENU 输出，并使用默认姿态约定。**
驱动不验证或修改设备配置。请先用 CHCenter 或 `python/` 下的命令行工具
设置输出格式，并确认设备已在输出数据，再启动节点。
用 launch 参数 `frame_id` 指定传感器机体坐标系；驱动不发布 TF。

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

使用 CAN 时，先按设备波特率启用接口；ROS 1 同样需要这两条命令：

```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ros2 launch hipnuc_imu can.launch.py interface:=can0 node_id:=8
```

接入已有 ROS 2 机器人工作空间时，将完整仓库放入其 `src` 目录，
再执行上面的依赖安装与构建命令。

要通过节点参数覆盖 ROS 2 数据发布者的 QoS，可直接启动节点：

```sh
ros2 run hipnuc_imu serial_node --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=115200 \
  -p qos_overrides./imu/data.publisher.reliability:=best_effort
```

数据话题支持在启动时覆盖 `history`、`depth`、`reliability`，不包括 `/diagnostics`。
参数键必须使用 namespace 和重映射之后的话题全名，例如 `/robot1/imu/data`。
这些节点参数不能追加到 `ros2 launch`；可放在自己的参数文件或 launch 文件中。
发布者使用 best-effort 时，订阅者也需允许 best-effort；要求 reliable 的订阅者无法接收。

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

接入已有机器人工作空间时，SDK 保留在上面的独立工作空间中，
先将其作为 underlay 加载，再重新构建自己的机器人软件包：

```sh
source ~/hipnuc_ros1_ws/devel/setup.bash
cd ~/robot_ws
catkin_make --force-cmake
source devel/setup.bash
```

使用 catkin_tools 时，在机器人工作空间中先执行
`catkin config --extend ~/hipnuc_ros1_ws/devel`，再运行 `catkin build`。

## 消息

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | 当前可用的加速度（m/s²）、角速度（rad/s）与姿态 |
| `imu/mag` | `sensor_msgs/MagneticField` | 磁场，T |
| `imu/temperature` | `sensor_msgs/Temperature` | 温度，°C |
| `hipnuc/imu` | `HipnucImu` | 产品字段、来源、设备时间、状态与字段存在位 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 连接、接收速率和错误计数 |

当前帧或 PGN 中加速度、角速度、四元数任一有效时，即发布 `Imu`。
缺失物理量的协方差首元素为 `-1`，使用方必须忽略该物理量；全零协方差表示未知。
驱动不拼接不同 PGN，因此 Classic CAN 可以发布只含部分测量的 IMU 消息；
磁场和温度有数据时仍发布到各自的标准话题。加速度为包含重力的比力。

`imu_filter_madgwick` 等需要完整惯性输入的滤波器，要求每条输入消息同时包含有效的
加速度和角速度。请选择同时提供两者的输出格式；单物理量 PGN 不满足此条件。
滤波器的输入和输出必须使用不同话题，避免输出被再次送回输入。
未知协方差不足以直接融合，需要按应用和使用方要求提供不确定度。

消息头取解码后、发布前的主机 ROS 时钟，不是内核接收时间或设备采样时间。
`use_sim_time` 默认为 `false`；启用后必须提供有效的 `/clock`。
传输和调度延迟仍然存在，融合应用需自行评估时间同步。

定位与速度保留在独立产品消息中，不提供标准导航话题。
产品消息不能直接替代通用融合节点的标准输入。只读取 `VALID_*` 位已设置的字段；
字段存在不代表姿态收敛或 GNSS 定位有效。气压仅保留为产品原始字段，
协议尚不能确认其可用性与数据新鲜度，因此不提供标准气压话题。
产品消息类型在 ROS 2 中为
[`hipnuc_msgs/msg/HipnucImu`](ros2/hipnuc_msgs/msg/HipnucImu.msg)，在 ROS 1 中为
[`hipnuc_imu/HipnucImu`](ros1/src/hipnuc_imu/msg/HipnucImu.msg)。
SDK 更新涉及 `HipnucImu` 定义变化时，请重新构建驱动及所有使用该消息的软件包。

## 连接提示

- `port`/`baudrate`（或 `interface`/`node_id`）和 `frame_id`（默认 `imu_link`）
  都是 launch 参数。`params_file` 默认使用软件包的 `config/serial.yaml` 或
  `config/can.yaml`，其中包含 `publish_imu`、`publish_mag`、
  `publish_temperature`、`publish_hipnuc` 开关。
- 驱动参数在启动时读取，修改后须重启节点；ROS 2 会拒绝运行时修改这些参数。
  可通过自己的 launch 文件或 ROS 重映射设置标准 namespace 和节点名称。
- 出现 `Permission denied` 时，执行 `sudo usermod -aG dialout "$USER"`，然后注销并重新登录。
  虚拟环境不会授予串口权限。
- 多个 USB 转接器并存时，优先使用 `/dev/serial/by-id/` 下的路径。
  端口或接口不可用时驱动会重试，状态每次变化都会打印日志，并持续发布诊断；
  ROS 仿真时间暂停也不影响诊断检查。

要把部署配置放在自己的机器人 bringup 包中，复制对应 ROS 版本的 YAML，
修改后通过绝对路径传入：

```sh
ros2 launch hipnuc_imu serial.launch.py params_file:=/path/to/bringup/config/imu.yaml
# ROS 1：
roslaunch hipnuc_imu serial.launch params_file:=/path/to/bringup/config/imu.yaml
```

CAN launch 文件同样支持 `params_file`。连接设置和 `frame_id` 始终取自 launch 参数，
包括这些参数的默认值，并覆盖 YAML 中的同名项。
修改自己的参数文件后重启节点即可，无需重新构建驱动包。
