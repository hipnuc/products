[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC SDK 与例程

提供 HiPNUC IMU/AHRS/MRU 和 INS 的驱动库、工具与例程，用于数据读取、设备配置和记录。

## 选择语言或平台

选择适合你的入口，按其中的说明开始使用。

| 语言 / 平台 | 主要用途 |
| --- | --- |
| [Python SDK](python/README_zh.md) | 串口/NMEA、Modbus RTU、设备配置与记录，附简短例程 |
| [C / Linux 串口](examples/C) | 使用 `hihost` 读取、配置和记录串口数据 |
| [CAN](examples/CAN) | Linux、STM32 例程，J1939/CANopen 解码和 DBC 文件 |
| [STM32 串口](examples/stm32_serial) | 在嵌入式应用中接入串口解码 |
| [Arduino](examples/arduino) | 使用 Arduino 读取设备数据 |
| [ROS 2](examples/ROS2) | 接入 ROS 2 应用 |
| [ROS 1 / Melodic](examples/ROS_Melodic) | 接入 ROS Melodic 应用 |
| [MATLAB](examples/matlab) | 读取 CHCenter 日志，计算 Allan 方差 |
| [EtherCAT](examples/ecat) | EtherCAT 接入例程 |
| [C 解码库](drivers) | 在自己的项目中集成二进制、NMEA、J1939 或 CANopen 解码 |

## 工具与资料

- [CHCenter Windows 上位机](https://download.hipnuc.com/internal/pc_host/CHCenter.zip)：通过图形界面评估和配置已连接的设备。
- [产品手册与下载](https://download.hipnuc.com)：接线、通信协议和各型号的配置说明。
- [CP210x USB 转串口驱动](usb_uart_drivers)：适配使用该芯片的评估板，串口无法识别时按需安装。
- [HiPNUC 官网](https://www.hipnuc.com)

## 联系我们

![联系二维码](img/qr_qqq.jpg)
