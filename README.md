[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC SDKs and Examples

Libraries, tools and examples for reading, configuring and recording data from
HiPNUC IMU/AHRS/MRU and INS devices (firmware 1.6.9 or later).
The early 1.7.1 HI83 layout with a 4-byte timestamp is not supported.

## Choose your platform

| Directory | What it is |
| --- | --- |
| [stm32/](stm32/README.md) | STM32F103 Keil examples for UART and J1939 reception |
| [ros/](ros/README.md) | ROS 2 Humble/Jazzy/Lyrical and ROS 1 Noetic drivers |
| [c/](c/README.md) | Copyable serial/CAN decoders, Windows/Linux serial API and C/C++ examples |
| [python/](python/README.md) | Python SDK and `hihost` CLI: serial, Modbus RTU, CAN, recording and firmware update |

Also available: [EtherCAT for HI15](ethercat/README.md),
[MATLAB analysis of HI91 CSV/JSONL recordings](matlab/README.md)
and the [Classic CAN J1939 database](dbc/README.md).

## Tools and documentation

- [Official downloads](https://download.hipnuc.com): CHCenter for Windows/Linux, product manuals and datasheets.
- [HiPNUC](https://www.hipnuc.com): products and contact information.
- Evaluation-board USB drivers: use the [current CP210x driver](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers) if your OS needs one. Common Linux distributions include it; serial access may require the `dialout` group.

## Contact

![Contact QR code](img/qr_qqq.jpg)
