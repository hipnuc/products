[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC SDK 与例程

用于读取、配置和录制 HiPNUC IMU/AHRS/MRU 与 INS 设备数据的库、工具和例程（固件 1.6.9 及以上）。

## 选择你的平台

| 目录 | 内容 |
| --- | --- |
| [python/](python/README_zh.md) | Python SDK 与 `hipnuc` 命令行：自动找设备、读取、录制、发指令、Modbus RTU |
| [c/hipnuc/](c/hipnuc/README_zh.md) | C 核心：复制几个文件或用 CMake 链接；串口二进制、NMEA、J1939/CANFD83、固件升级 |
| [c/tools/](c/tools/README_zh.md) | Linux 命令行工具 `hihost`（串口）和 `canhost`（SocketCAN），基于 C 核心 |
| [stm32/serial/](stm32/serial/README_zh.md) | STM32F103 Keil 工程：USART2 接收并打印姿态 |
| [stm32/can/](stm32/can/README_zh.md) | STM32F103 Keil 工程：接收 J1939 帧 |
| [ros/](ros/README_zh.md) | ROS 2（Humble/Jazzy）与 ROS 1（Noetic）驱动包 |
| [ethercat/](ethercat/README_zh.md) | HI15 EtherCAT 例程（IgH 主站） |
| [matlab/](matlab/allan/README.md) | 读取录制的 CSV 数据并计算 Allan 方差 |
| [protocol/dbc/](protocol/dbc/README.md) | 供 CAN 分析工具使用的 DBC 文件 |

## 工具与文档

- CHCenter：Windows/Linux 桌面程序，用于评估、配置和固件升级。
- 产品手册、规格书与下载：HiPNUC 资料网站。
- [CP210x USB 转串口驱动](usb_uart_drivers)：评估板使用。

## 联系我们

![联系二维码](img/qr_qqq.jpg)
