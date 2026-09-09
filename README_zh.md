[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC SDK 与例程

用于读取、配置和录制 HiPNUC IMU/AHRS/MRU 与 INS 设备数据的库、工具和例程（固件 1.7.0 及以上）。
不支持早期 1.7.1 中使用 4 字节时间戳的 HI83 布局。

## 选择你的平台

| 目录 | 内容 |
| --- | --- |
| [stm32/](stm32/README_zh.md) | STM32F103 Keil 工程：UART 和 J1939 接收 |
| [ros/](ros/README_zh.md) | ROS 2 Humble/Jazzy/Lyrical 与 ROS 1 Noetic 驱动 |
| [c/](c/README_zh.md) | 可复制的串口/CAN 解码器、Windows/Linux 串口 API 和 C/C++ 例程 |
| [python/](python/README_zh.md) | Python SDK 与 `hihost` 命令行：串口、Modbus RTU、CAN、录制和固件升级 |

其他资源：[HI15 EtherCAT](ethercat/README_zh.md)、
[MATLAB 分析 HI91 CSV/JSONL 录制](matlab/README_zh.md)、
[Classic CAN J1939 数据库](dbc/README_zh.md)。

## 工具与文档

- [官方下载](https://download.hipnuc.com)：Windows/Linux 版 CHCenter、产品手册和规格书。
- [HiPNUC 官网](https://www.hipnuc.com)：产品与联系方式。
- 评估板 USB 驱动：系统需要时安装[当前 CP210x 驱动](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers)。常用 Linux 已内置驱动，串口权限可能需要加入 `dialout` 组。

## 联系我们

![联系二维码](img/qr_qqq.jpg)
