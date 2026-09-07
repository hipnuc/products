# STM32 CAN（J1939）例程

[English](README.md) | [中文](README_zh.md)

通过 CAN1 接收 HiPNUC 的 J1939 帧，在 USART1 打印合并后的姿态。
开发板：正点原子 战舰 V3（STM32F103ZET6）及其板载 CAN 收发器，Keil MDK 5 + ARM Compiler 5，
StdPeriph 库。

## 接线

| 设备 | 开发板 |
| --- | --- |
| CAN_H / CAN_L | 收发器 CAN_H / CAN_L（PA11 RX，PA12 TX） |
| GND | GND |

总线两端各接 120 Ω 终端电阻。调试串口：USART1（PA9/PA10）的 USB 转串口，115200。

## 运行

1. 打开 `USER/hipnuc_can_decode.uvprojx`，编译、下载。
2. 打开调试串口。每 200 ms 打印一次最新的 roll/pitch/yaw、加速度、角速度和帧计数。
3. `USER/main.c` 顶部的设置：`CAN_BAUD_KBPS`（设备默认 500）、`DEVICE_NODE_ID`（设备默认 8）。

## 在自己的代码里使用数据

每个 J1939 帧只携带少量字段。`main.c` 的循环用 `hipnuc_j1939_parse()` 把每帧解到 `part`，
再用 `hipnuc_j1939_merge()` 合并进 `merged`；两者都是 `hipnuc_sample_t`（SI 单位，见
`c/hipnuc/README.md`）。其它源地址的帧会被忽略。

STM32F1 只有经典 CAN；CANFD83 帧需要支持 CAN FD 的控制器。
