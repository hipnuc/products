# STM32 串口例程

[English](README.md) | [中文](README_zh.md)

通过 USART2 接收 HiPNUC 数据，在 USART1 打印姿态。
开发板：正点原子 战舰 V3（STM32F103ZET6），Keil MDK 5 + ARM Compiler 5，StdPeriph 库。
其它 STM32F10x 板卡改 `USER/hipnuc_board.c` 中的引脚即可。

## 接线

| IMU | 开发板 |
| --- | --- |
| TXD | PA3（USART2 RX） |
| RXD | PA2（USART2 TX） |
| 3.3V / GND | 3V3 / GND |

调试串口：开发板 USART1（PA9/PA10）的 USB 转串口，115200。

## 运行

1. 打开 `USER/hipnuc_serial_decode.uvprojx`，编译、下载。
2. 打开调试串口。每 200 ms 打印一次 roll/pitch/yaw、加速度和帧率；没有数据或没有合法帧时
   会给出提示。
3. 设备不是 115200 时修改 `USER/main.c` 里的 `IMU_BAUDRATE`。

## 在自己的代码里使用数据

`main.c` 就是完整应用：

```c
hipnuc_board_init(IMU_BAUDRATE);
while (1) {
    if (hipnuc_board_poll(&sample)) {
        /* sample.roll、sample.pitch、sample.yaw（rad），sample.acc（m/s^2）…… */
    }
}
```

`USER/hipnuc_board.c` 负责串口（默认 DMA 环形缓冲，`HIPNUC_BOARD_USE_DMA 0` 改为逐字节中断）、
解码器和到 `hipnuc_sample_t` 的转换；`hipnuc_board_stats()` 给出字节数、帧数、CRC 错误和接收溢出。
解码器文件直接来自 `c/hipnuc`（字段说明见其 README）。

921600 波特率下请至少每 10 ms 调用一次 `hipnuc_board_poll()`（接收缓冲 1024 字节）；
`printf` 不要放在关键路径上：500 Hz 输出时 115200 的调试串口打印不完每一帧。
