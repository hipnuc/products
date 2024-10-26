[English](README.md) | [中文](README_zh.md)

![Logo](img/logo.png)

# HiPNUC Product Software Examples Package

## Directory Structure

- usb_uart_drivers/: CP210x USB-to-UART Bridge Driver
  - Linux: Generally plug-and-play for Ubuntu 18.04 and above. If the device is not recognized, please refer to the installation guide on Silabs [official website](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=overview), or search for "CP210x driver installation on Linux" for solutions.
  - Windows: Execute the driver installer directly

- examples/: Example programs and source code for various platforms and languages, including STM32, ROS, Linux, MATLAB, etc.

## Quick Start

### Windows

1. Driver Installation: Install the USB-to-UART bridge driver from the `usb_uart_drivers` folder.
2. Host Software: Download and install [CHCenter](http://download.hipnuc.com/internal/pc_host/CHCenter.7z)
3. Hardware Connection: Connect the evaluation board's USB port to your PC, launch CHCenter, establish connection to the corresponding COM port to begin product evaluation.

### Linux

1. Recommended to start with the [Linux C examples](examples/linux)

## Example Code

Example code is provided for multiple languages and platforms:

- [Python Examples (Including module configuration)](examples/python)
- [C/STM32 Examples](examples/STM32)
- [ROS (Melodic) Examples](examples/ROS_Melodic)
- [ROS2 Examples](examples/ROS2)
- [Linux C Examples (Includes module configuration, firmware, and reading examples - recommended for Linux users)](examples/linux)

## Module Configuration

For first-time configuration, it is strongly recommended to use the Windows host software CHCenter. Once familiar, if you plan to use command scripts or work in a Linux environment, refer to the Python examples which include tutorials for Python-based data reading and module configuration.

## Resources

- Official Website: [www.hipnuc.com](http://www.hipnuc.com)
- Product Documentation: [GitHub Repository](https://github.com/hipnuc/products.git)

## Contact Us

Follow us and get the latest updates through:

![QR Code](img/qr_qqq.jpg)
