# STM32 串口例程

[English](README.md) | [中文](README_zh.md)

通过 USART2 接收 HiPNUC 二进制数据，通过 USART1 查看部分测量。
工程适用于正点原子战舰 V3（STM32F103ZET6），使用 Keil MDK 5、
ARM Compiler 5 和 STM32 标准外设库。

## 接线与运行

| 信号 | STM32 引脚 |
| --- | --- |
| 设备 TX | PA3 — USART2 RX |
| 设备 RX | PA2 — USART2 TX |
| 设备 GND | GND |
| 调试输出 TX | PA9 — USART1 TX |

串口信号为 3.3 V TTL。设备按对应型号要求供电，不要将 RS-232 或
RS-485 信号直接接到 MCU。用 USB-TTL 转换器连接 PA9/GND，串口助手设置为
115200、8N1。

1. 在 [main.c](USER/main.c) 顶部将 `IMU_BAUDRATE` 设置为设备波特率。
2. 打开 `USER/hipnuc_serial_decode.uvprojx`，编译并下载。
3. 设备需启用支持的二进制输出，例如 HI91。例程最多每 200 ms 显示一次新样本，
   每两秒报告接收情况、无数据或恢复接收。

## 使用测量值

在 `main.c` 标出的应用代码位置添加处理：

```c
if (sample.valid & HIPNUC_VALID_ACC) {
    float acceleration_x = sample.acc[0]; /* m/s^2 */
    /* Use acceleration_x here. */
}
```

每次 `hipnuc_board_poll(&sample)` 最多返回一个新样本。读取字段前检查对应
`HIPNUC_VALID_*` 位；不存在的字段不能作为测量值使用。角度单位为 rad，
角速度为 rad/s。完整字段定义见 [C 样本头文件](../../c/hipnuc/hipnuc_sample.h)。

[hipnuc_board.c](USER/hipnuc_board.c) 包含串口初始化和接收实现，使用
DMA1 通道 6、USART2 和 1 ms SysTick；不要在其他位置重复配置这些资源。
解码器直接引用 `c/hipnuc/`，无需维护副本。

默认 DMA 缓冲为 1024 字节：8N1 下，115200 波特率约需 88.9 ms 写满，
921600 约需 11.1 ms。**主循环处理和 DMA 中断服务都必须快于一次缓冲写满时间。**
应用代码保持简短；数据量大时增大 `PRINT_PERIOD_MS` 以降低打印频率，或设为 0
关闭样本打印。串口打印会阻塞，例程不保证任意应用负载下都无丢失。

检测到覆盖后会增加溢出计数，并丢弃受影响的未完成帧。中断被阻塞期间，
一个 DMA 标志无法记录多次回绕，因此计数为零不等于完全没有丢失。
UART 硬件溢出、帧错误和噪声错误另行计数。检测到错误后，下次轮询会丢弃
缓冲中的字节和未完成帧，再继续接收；该计数表示观察到的错误事件数，
不能换算为丢失字节数。
若需逐字节中断接收，在 [hipnuc_board.h](USER/hipnuc_board.h) 中将
`HIPNUC_BOARD_USE_DMA` 改为 `0`；高波特率建议使用 DMA。
