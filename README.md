[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC SDKs and Examples

Libraries, tools and examples for reading, configuring and recording data from
HiPNUC IMU/AHRS/MRU and INS devices (firmware 1.6.9 or later).

## Choose your platform

| Directory | What it is |
| --- | --- |
| [python/](python/README.md) | Python SDK and `hipnuc` command line: find the device, read, record, send commands, Modbus RTU |
| [c/hipnuc/](c/hipnuc/README.md) | C core: copy a few files or link with CMake; serial binary, NMEA, J1939/CANFD83, firmware update |
| [c/tools/](c/tools/README.md) | Linux command-line tools `hihost` (serial) and `canhost` (SocketCAN), built on the C core |
| [stm32/serial/](stm32/serial/README.md) | STM32F103 Keil project: receive on USART2, print attitude |
| [stm32/can/](stm32/can/README.md) | STM32F103 Keil project: receive J1939 frames |
| [ros/](ros/README.md) | ROS 2 (Humble/Jazzy) and ROS 1 (Noetic) driver packages |
| [ethercat/](ethercat/README.md) | HI15 EtherCAT example (IgH master) |
| [matlab/](matlab/allan/README.md) | Read recorded CSV data and compute Allan variance |
| [protocol/dbc/](protocol/dbc/README.md) | DBC files for CAN analysis tools |

## Tools and documentation

- CHCenter: the Windows/Linux desktop application for evaluation, configuration and firmware update.
- Product manuals, datasheets and downloads: HiPNUC documentation site.
- [CP210x USB-to-UART drivers](usb_uart_drivers) for the evaluation boards.

## Contact

![Contact QR code](img/qr_qqq.jpg)
