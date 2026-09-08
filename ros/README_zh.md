# ROS 驱动

[English](README.md) | [中文](README_zh.md)

面向 HiPNUC IMU/AHRS/MRU/INS 的串口与 SocketCAN 驱动，复用 C 协议库。
目标平台：ROS 2 Humble（Ubuntu 22.04）、Jazzy（24.04）、Lyrical（26.04），
以及 ROS 1 Noetic（20.04）。

**启动前，请将设备配置为 ENU 输出，并使用默认姿态约定。**
驱动不验证或修改设备配置。`frame_id` 表示传感器机体坐标系，驱动不发布 TF。

## ROS 2

安装 ROS 后，将完整仓库放入工作空间：

```sh
source /opt/ros/jazzy/setup.bash   # 改为已安装的发行版
mkdir -p ~/hipnuc_ws/src
cd ~/hipnuc_ws/src
git clone https://github.com/hipnuc/products.git
cd ..
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
rosdep install --from-paths src/products/ros/ros1/src --ignore-src -r -y
catkin_make --source src/products/ros/ros1/src
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0 baudrate:=115200
# 或：roslaunch hipnuc_imu can.launch interface:=can0 node_id:=8
```

## 消息

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | 当前加速度、角速度及／或四元数 |
| `imu/mag` | `sensor_msgs/MagneticField` | 磁场，T |
| `imu/temperature` | `sensor_msgs/Temperature` | 温度，°C |
| `hipnuc/imu` | `HipnucImu` | 产品字段、来源、设备时间、状态与字段存在位 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 连接、接收速率和错误计数 |

只发布当前帧中存在的物理量。Classic CAN 逐 PGN 发布，不等待 yaw，也不拼接旧字段。
`Imu` 协方差首元素为 `-1` 表示该物理量缺失；全零协方差表示未知。
请确认上层程序是否支持部分 IMU 消息，并按应用要求提供不确定度。

消息头使用 ROS 时钟标记的主机接收时间，不代表设备采样时间。
加速度为包含重力的比力。定位与速度保留在独立产品消息中，不提供标准导航话题。
产品消息不能直接替代通用融合节点的标准输入。只读取 `VALID_*` 位已设置的字段；
字段存在不代表姿态收敛或 GNSS 定位有效。气压仅保留为产品原始字段，
协议尚不能确认其可用性与数据新鲜度，因此不提供标准气压话题。完整定义见
[HipnucImu.msg](ros2/hipnuc_msgs/msg/HipnucImu.msg)。

## 连接提示

- 在软件包的 `config/serial.yaml` 或 `config/can.yaml` 中修改话题开关和坐标系名称。
  启动参数覆盖连接参数。
- 出现 `Permission denied` 时，执行 `sudo usermod -aG dialout "$USER"`，然后注销并重新登录。
  虚拟环境不会授予串口权限。
- 多个 USB 转接器并存时，优先使用 `/dev/serial/by-id/` 下的路径。
  端口或接口不可用时驱动会重试，并持续发布诊断；ROS 仿真时间暂停也不影响诊断检查。
