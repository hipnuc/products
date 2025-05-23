# STM32例程

## 简介

本例程提供了一个使用C语言编写的示例代码，用于演示如何通过USART2接收来自HiPNUC IMU的传感器数据，并通过USART1输出解码后的结果。

- 支持解码HiPNUC 二进制协议，包括HI91, HI92, HI81 以及NMEA GGA/RMC/SXT 数据帧。
- **测试环境**：Windows 10/Windows 11
- **编译器**：Keil MDK V5.38
- **开发板**：正点原子-战舰V3 STM32F103ZET6
- **测试设备**：HiPNUC 产品

## 硬件连接

1. 在使用之前确认你已经对开发板硬件有初步了解，可以正常下载Keil程序，开发板烧写HelloWorld可以正常在串口输出信息。

### 连接步骤

1. 将IMU模块正确插入到模块评估板上。
2. 使用杜邦线将IMU模块与开发板连接，具体连接如下：

| 超核IMU | 正点原子开发板 |
| ------- | -------------- |
| RXD     | PA2(TXD)       |
| TXD     | PA3(RXD)       |
| 3.3V    | 3V3            |
| GND     | GND            |

3. 用USB线插到开发板的调试串口(USART1: PA9/PA10)上，另一端插到电脑上（调试，输出信息串口）

## 观察输出

​	打开串口调试助手，打开开发板对应的串口号，观察数据输出

```
parse ok, frame len:76
tag:            0x91
acc(m/s^(2)):   -2.162 2.050 9.299
gyr(deg/s):     -0.062 -0.006 -0.010
mag(uT):        7.892 14.625 -60.042
Roll/Pitch/Yaw(deg):13.052 12.188 -122.477
quat:           -0.486 -0.150 0.038 0.860
timestamp(ms):  1840392
```


## 注意事项

1. 确保IMU模块和开发板连接正确，避免因接触不良导致的数据传输问题。
2. 在进行串口调试时，请确保选择了正确的串口号和波特率。
3. 如果使用DMA进行USART接收，请确保DMA配置正确，并在代码中启用相应的宏定义。

## 更多信息

更多信息和技术支持，请访问[HiPNUC官网](http://www.hipnuc.com)。
