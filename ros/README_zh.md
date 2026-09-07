# ROS 驱动

[English](README.md) | [中文](README_zh.md)

基于 `c/hipnuc` C 核心的薄 ROS 包。两者发布相同话题；`ros2/` 面向 Humble 与 Jazzy，
`ros1/` 面向 Noetic。

| 话题 | 类型 | 何时发布 |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/Imu` | 每帧含加速度、角速度或四元数时 |
| `imu/mag` | `sensor_msgs/MagneticField` | 有磁场数据时 |
| `imu/temperature`、`imu/pressure` | `sensor_msgs/Temperature`、`FluidPressure` | 有数据时 |
| `gnss/fix` | `sensor_msgs/NavSatFix` | 有 INS/GNSS 位置时（高度为椭球高，大地水准面差未知时为 NaN） |
| `ins/velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 有 ENU 速度时，`frame_id` = `enu_frame_id` |
| `hipnuc/imu` | `hipnuc_msgs/HipnucImu`（ROS 1：`hipnuc_imu/HipnucImu`） | 每帧；全部字段 SI 单位并带有效位掩码 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 每秒一次：连接状态、帧率、CRC/无效帧计数 |

时间戳使用节点时钟。协方差为零（未知）或首元素 `-1`（该量未提供）。约定见
`ros/common/hipnuc_convert.hpp`。

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

## 说明

- 参数在 `config/serial.yaml` 与 `config/can.yaml`；launch 参数可覆盖 port/baudrate 和
  interface/node_id。
- 串口访问需要 `dialout` 组；包内的 `99-hipnuc.rules` 为评估板提供固定的 `/dev/hipnuc` 设备名。
- CAN 节点合并同一设备（`node_id`）的各 J1939 PGN，在 `trigger_pgn`（默认 yaw，0xFF41）到达时
  发布；CANFD83 帧直接发布。其它源地址的帧只计数不发布。
- 在自己的工作区使用时，复制 `ros/common` 和对应包，并把 `HIPNUC_CORE_DIR` 指向 `c/hipnuc` 目录。
