# Linux下HiPNUC CAN 驱动及example使用指南

## 项目概述

本项目提供了在Linux系统下使用HiPNUC IMU设备的CAN通信驱动程序和示例代码。支持CANopen和J1939两种协议，可以实时接收和解析IMU传感器数据，包括加速度、角速度、姿态角、四元数、气压等信息。

## 硬件要求

- **CAN适配器**: PEAK-CAN USB转CAN适配器（推荐）或其他Linux兼容的CAN设备
- **设备**: HiPNUC系列产品
- **操作系统**: Linux系统（Ubuntu 20.04+推荐）

## 软件依赖

```
sudo apt update
sudo apt install build-essential cmake git

# 安装CAN工具包
sudo apt install can-utils

# 如果使用PEAK-CAN设备，需要安装PEAK驱动
# 详见: https://www.peak-system.com/fileadmin/media/linux/index.php
```

## 编译安装

```
# 创建构建目录
mkdir build && cd build

# 编译
cmake ..
make

# 运行程序
./imu_can_reader
```

## CAN接口配置

### 1. 检查CAN硬件

```
ip link show

# 查看CAN设备类型
cat /sys/class/net/can0/type
# 输出280表示CAN设备
```

### 2. 配置CAN接口

```
sudo ip link set down can0

# 设置CAN波特率（根据IMU设备配置，通常为500kbps）
sudo ip link set can0 type can bitrate 500000

# 启动CAN接口
sudo ip link set up can0

# 验证配置
ip link show can0
```

## 程序使用方法

### 基本用法

```
./imu_can_reader

# 示例输出:
=====================================
     Available CAN Interfaces       
=====================================
 *can0      : up
=====================================
Selected: can0
Interface 'can0' is ready!

=== HiPNUC IMU CAN Parser ===
Interface: can0 | Node ID: 8 | Total: 45.2 Hz

Message Type  | Rate (Hz) | Data
ACCEL         |     10.1  | X: -0.221 Y:  0.209 Z:  0.949 (m/s²)
GYRO          |     10.0  | X: -0.062 Y: -0.006 Z: -0.010 (rad/s)
EULER         |     10.0  | Roll: 13.05 Pitch: 12.19 Yaw: -122.48 (deg)
QUAT          |     10.0  | W: -0.486 X: -0.150 Y:  0.038 Z:  0.860
PRESSURE      |      5.0  | 100676.0 Pa
```

## 常见问题排查

### 1. CAN接口问题

**问题**: `RTNETLINK answers: Operation not supported`

```
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe vcan

# 检查模块是否加载
lsmod | grep can
```

**问题**: `Cannot find device "can0"`

```
lsusb | grep -i peak  # 对于PEAK-CAN设备

# 检查内核日志
dmesg | grep -i can

# 手动创建虚拟CAN接口（用于测试）
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 3. 波特率不匹配

**问题**: 接收不到数据或数据错误

```
# 常见波特率: 125000, 250000, 500000, 1000000

# 重新配置CAN接口波特率
sudo ip link set down can0
sudo ip link set can0 type can bitrate 250000  # 尝试不同波特率
sudo ip link set up can0
```

### 4. 节点ID问题

**问题**: 程序运行但无数据显示

```
./imu_can_reader can0 8   # 默认
./imu_can_reader can0 2   # 尝试ID 2
```

## 技术支持

- **官方文档**: 参考`指令与编程手册`获取完整的IMU配置和协议说明
- **官方网站**: [www.hipnuc.com](http://www.hipnuc.com)

------

**注意**: 使用前请确保IMU设备已正确配置CAN输出模式和相应的波特率。如遇到问题，请首先检查硬件连接和CAN接口配置。
