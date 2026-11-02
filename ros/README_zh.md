# ROS 驱动

[English](README.md) | [中文](README_zh.md)

适用于 HiPNUC IMU、AHRS、INS 模块的 ROS 2（Humble、Jazzy、Lyrical）和 ROS 1（Noetic）串口驱动。
驱动不会修改设备设置：连接时读取一次型号、固件版本和轴向设置，之后只接收数据。需要固件 1.6.9 及以上。

## 快速开始（ROS 2）

访问串口需要 `dialout` 组：执行一次 `sudo usermod -aG dialout "$USER"`，然后注销并重新登录。

先安装好 ROS 2（`sudo apt install ros-dev-tools` 会装上 colcon 和 rosdep；新机器上执行一次 `sudo rosdep init`）：

```sh
source /opt/ros/jazzy/setup.bash        # 换成你的发行版
mkdir -p ~/hipnuc_ws/src && cd ~/hipnuc_ws/src
git clone https://github.com/hipnuc/products.git
cd ~/hipnuc_ws
rosdep update && rosdep install --from-paths src/products/ros/hipnuc_imu --ignore-src -y
colcon build --packages-select hipnuc_imu
source install/setup.bash
ros2 launch hipnuc_imu serial.launch.py port:=/dev/ttyUSB0
```

如果 `rosdep update` 无法访问在线索引，可以跳过 rosdep 那一行：标准的 ROS 安装（ros-base 或 desktop）
已包含全部依赖。下文 ROS 1 同理。

驱动会自动识别波特率，先试 `baudrate`（默认 115200），并在日志中给出找到的型号、固件和波特率。
在另一个终端执行 `source ~/hipnuc_ws/install/setup.bash`，然后检查数据：

```sh
ros2 topic hz /imu/data
ros2 topic echo /diagnostics
```

接了多个 USB 转串口时，`port` 用 `/dev/serial/by-id/` 下的固定名称。

## 坐标轴与 TF

驱动按设备输出的轴向原样发布数据，不做旋转。`frame_id`（默认 `imu_link`）表示这组轴，
再用一条从机器人坐标系到 `imu_link` 的静态变换告诉 ROS 模块是怎么安装的。

确认设备输出哪种轴向：让模块静止，以外壳 Y 箭头所指的一端朝下竖起（Y 箭头朝下），
观察 `ros2 topic echo /imu/data --field linear_acceleration`：

| Y 箭头朝下时的加速度 | 输出轴向 | 变换 `base_link` → `imu_link`（模块水平、Y 箭头朝前） |
| --- | --- | --- |
| y 约 −9.8 | x 右、y 前、z 上（出厂） | 偏航 −90° |
| x 约 −9.8 | x 前、y 左、z 上（ROS） | 无 |

较新的固件还会在启动日志中给出轴向设置。设置 `COORD 4` 只在部分较新的固件上输出 ROS 轴向，
其他固件的数据仍是出厂轴向，请按上面的方法确认。把这条变换和其他静态变换一起发布，例如：

```sh
ros2 run tf2_ros static_transform_publisher --yaw -1.5708 --frame-id base_link --child-frame-id imu_link
```

安装位置偏移（`--x --y --z`）和其他旋转也写进同一条变换，或者在 URDF 里给 `imu_link` 关节填相同的值。

姿态以设备自身的参考系为基准。6 轴模式下航向以上电时的朝向为起点，所以加上 −90° 变换后机器人的
初始航向为 90°；需要从 0 开始时，可使用 robot_localization 的 `imu0_relative`。9 轴模式在出厂轴向
下以东北天为参考，符合 ROS 的约定；设置 `COORD 4` 时多数固件改以北西天为参考。

## SLAM

**频率。** 多数 SLAM 前端需要 200 Hz 以上；出厂输出为 115200 波特率、100 Hz，驱动测得低于 200 Hz
时会在日志中给出下面的命令。输出周期为整毫秒，可选 200 Hz（`0.005`）或 500 Hz（`0.002`），没有
400 Hz。HI91 跑 200 Hz 需要 230400 波特率，500 Hz 需要 921600。先停止节点，再安装并使用 Python
命令行 `hihost`（见 [python/](../python/README_zh.md)）：

```sh
hihost baudrate 230400 -p /dev/ttyUSB0 --save
hihost command "LOG HI91 ONTIME 0.005" -p /dev/ttyUSB0 -b 230400 --save
```

设备掉电后仍保持这些设置。驱动会自动找到新的波特率，启动命令不用改。

**时间戳。** 消息头的时间戳是设备开始发送该帧的时刻：主机收到的时刻减去该帧在线上的传输时间
（HI91 在 115200 下为 7.1 ms，921600 下为 0.9 ms）。`imu/data` 的时间戳始终递增。USB 转串口和设备
本身还有少量延迟；需要更精确地对齐时，FAST-LIO 和 Point-LIO 可以设置雷达到 IMU 的时间偏移。

**外参。** FAST-LIO、Point-LIO 和 LIO-SAM 从各自的配置文件读取 IMU 与雷达之间的旋转，不读 TF。
FAST-LIO 和 Point-LIO 的 `extrinsic_R` 把雷达向量转到 IMU 轴；LIO-SAM 的 `extrinsicRot` 把 IMU 向量转到
雷达轴。雷达 x 朝前、z 朝上安装时：

| 设备轴向 | FAST-LIO / Point-LIO `extrinsic_R` | LIO-SAM `extrinsicRot` 和 `extrinsicRPY` |
| --- | --- | --- |
| x 右、y 前、z 上 | `[0, -1, 0, 1, 0, 0, 0, 0, 1]` | `[0, 1, 0, -1, 0, 0, 0, 0, 1]` |
| x 前、y 左、z 上 | 单位阵 | 单位阵 |

**各软件包。**

- Point-LIO：`acc_norm` 设为 9.81（默认 1.0 按 g 计），`satu_acc` 按加速度计量程调大（单位 m/s²）。
- LIO-SAM：6 轴模式下设 `useImuHeadingInitialization: false`。
- Cartographer：`tracking_frame` 设为 `imu_link`。
- robot_localization：它把协方差 0 当作几乎完全准确，请只融合需要的量，例如偏航角速度。

## 话题与参数

| 话题 | 类型 | 默认 |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | 开 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 开 |
| `imu/data_raw` | 不含姿态的 `sensor_msgs/Imu` | `publish_data_raw` |
| `imu/mag` | `sensor_msgs/MagneticField`，T（带磁力计的型号） | `publish_mag` |
| `imu/temperature` | `sensor_msgs/Temperature`，°C | `publish_temperature` |
| `hipnuc/imu` | `hipnuc_imu/HipnucImu`，全部解码字段 | `publish_hipnuc` |

每一帧同时含加速度和角速度时发布 `imu/data`。加速度是比力，单位 m/s²（静止时朝上的轴约为 +9.8），
角速度单位 rad/s。协方差全为 0，表示未知；姿态协方差第一个元素为 −1 表示该帧不含姿态。
`/diagnostics` 给出型号、固件、波特率、帧率、错误计数和设备状态。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 串口 |
| `baudrate` | `115200` | 先尝试的波特率，之后依次尝试其他常用波特率 |
| `frame_id` | `imu_link` | 所有消息的坐标系 |
| `publish_*` | 见上表 | 可选话题 |

参数在启动时读取。launch 文件接受 `port`、`baudrate`、`frame_id` 和 `params_file`；连接参数和
`frame_id` 始终取自这些参数，文件中只放 `publish_*` 开关。在自己的 launch 文件中使用时，复制仓库中的
`ros/hipnuc_imu/config/serial.yaml`，修改后传入路径：

```sh
ros2 launch hipnuc_imu serial.launch.py params_file:=/path/to/imu.yaml
```

数据话题为 reliable。启动时可以覆盖深度等 QoS，但请保持 reliable：reliable 的订阅者（例如 ROS 2 版
FAST-LIO）收不到 best_effort 发布者的数据。

`hipnuc/imu` 包含欧拉角、设备状态、设备时间和 UTC，以及 INS 型号的位置和速度。只有 `valid` 中对应的
`VALID_*` 位被置位时才读取该字段。部分机器人套件自带的旧驱动也叫 `hipnuc_imu`（话题 `/IMU_data`），
请先把它移出工作空间。

**从 2.x 升级。** 节点改名为 `hipnuc_imu_node`（原 `serial_node`），消息类型改为
`hipnuc_imu/msg/HipnucImu`（原 `hipnuc_msgs/msg/HipnucImu`）；`imu/mag`、`imu/temperature` 和
`hipnuc/imu` 默认关闭，需要时再打开。软件包目录换了位置，在已有的 ROS 2 工作空间中请先删除
`build/hipnuc_imu` 和 `install/hipnuc_imu` 再编译。

ROS 驱动只支持串口。CAN 型号请使用 C 例程 [read_can.c](../c/examples/read_can.c)、`hihost can read`，
或用 ros2_socketcan 接收原始帧。

## ROS 1（Noetic）

请使用单独的工作空间（`sudo apt install git python3-rosdep` 会装上所需工具；新机器上执行一次
`sudo rosdep init`）。下面的源码目录参数不能省略：catkin 会跳过标记给 colcon 的目录。

```sh
source /opt/ros/noetic/setup.bash
mkdir -p ~/hipnuc_ros1_ws/src && cd ~/hipnuc_ros1_ws/src
git clone https://github.com/hipnuc/products.git
cd ~/hipnuc_ros1_ws
rosdep update --include-eol-distros     # Noetic 已停止维护
rosdep install --from-paths src/products/ros/ros1/src --ignore-src -y
catkin_make --source src/products/ros/ros1/src
source devel/setup.bash
roslaunch hipnuc_imu serial.launch port:=/dev/ttyUSB0
```

话题、参数和消息与 ROS 2 相同；静态变换为
`rosrun tf2_ros static_transform_publisher 0 0 0 -1.5708 0 0 base_link imu_link`
（ros-base 需先执行 `sudo apt install ros-noetic-tf2-ros`）。

## 故障排查

节点在日志和 `/diagnostics` 中报告当前状态。

| 提示 | 处理 |
| --- | --- |
| `cannot open ... Permission denied` | 加入 `dialout` 组后重新登录。 |
| `cannot open ... No such file or directory` | 检查线缆；用 `ls /dev/serial/by-id` 列出串口。 |
| `cannot open ... busy` 或 `in use` | 关闭占用该串口的其他程序，例如 CHCenter 或 `hihost`。 |
| 插上后约 20 秒内的告警 | 可能是 ModemManager 在探测该串口，驱动会自动重连。 |
| `no data` | 检查接线，并确认设备已开启输出。 |
| `no valid frames at any baud rate` | 让设备输出 HI91、HI83 或 HI81。 |
| `imu/data stays silent` | 在 HI83 字段中加入加速度和角速度（MAP 第 0、1 位）。 |
| `imu/data runs at ... Hz` | 如果你的软件需要，按 [SLAM](#slam) 一节提高频率。 |
| 姿态差 90° | 使用[坐标轴与 TF](#坐标轴与-tf) 中的变换。 |
