[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC 产品软件例程包

## 目录结构

- usb_uart_drivers/: CP210xUSB 转串口驱动程序
  -  Linux: Ubuntu 18.04以上版本一般免驱. 如果未能识别请见Silabs[官网](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=overview)安装步骤, 或自行Google”Linux下安装CP210x驱动”解决。
  -  Windows 请直接运行exe驱动安装程序

- examples: 各种平台和语言的示例程序及其源码，包括 STM32, ROS, Linux, MATLAB 等。

## 快速开始

### Windows

1. 安装驱动: 从 `usb_uart_drivers` 文件夹安装 USB转串口驱动程序。
2. 下载Windows上位机软件：下载并安装 [CHCenter](http://download.hipnuc.com/internal/pc_host/CHCenter.7z)
3. 硬件连接：将评估板的 USB 口连接到 PC，打开 CHCenter，连接到相应的 COM 口，开始产品评估。

### linux

1. 推荐直接使用[Linux C 示例](examples/linux)入门

## 示例代码

提供多种语言和平台的示例代码：

- [python 示例(包含模块配置等)](examples/python)
- [C/STM32 示例](examples/STM32)
- [ROS (Melodic) 示例](examples/ROS_Melodic)
- [ROS2 示例](examples/ROS2)
- [Linux C 示例(包括模块配置,固件,读取示例等，Linux下入门推荐)](examples/linux)

## 配置模块

第一次配置模块强烈建议使用Windows上位机软件 CHCenter， 熟悉后，如果您打算用命令脚本或者在Linux环境下，可以参考python例程，里面包含基于Python的数据读取和模块配置教程。

## 资源

- 官方网站：[www.hipnuc.com](http://www.hipnuc.com)
- 产品资料和文档：[GitHub 仓库](https://github.com/hipnuc/products.git)

## 联系我们

欢迎通过以下方式关注我们和获取最新信息：

![QR Code](img/qr_qqq.jpg)
