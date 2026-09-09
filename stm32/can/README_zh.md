# STM32 CAN 例程

[English](README.md) | [中文](README_zh.md)

通过 CAN1 接收 HiPNUC J1939 报文。工程适用于正点原子战舰 V3
（STM32F103ZET6），使用 Keil MDK 5、ARM Compiler 5 和 STM32 标准外设库。
STM32F103 仅支持 **Classic CAN，不支持 CAN FD**。

## 接线与运行

| 连接 | 开发板 |
| --- | --- |
| 设备 CAN_H / CAN_L | CAN 收发器 CAN_H / CAN_L |
| 设备 GND | GND |
| 收发器 RXD / TXD | PA11 / PA12 |
| 调试输出 TX | PA9 — USART1 TX |

使用兼容 MCU 3.3 V 逻辑的 CAN 收发器，不要将总线直接连接到 PA11/PA12。
总线两个物理末端各接 120 Ω 终端电阻。设备按对应型号要求供电。
用 USB-TTL 转换器连接 PA9/GND，串口助手设置为 115200、8N1。

1. 在 [main.c](USER/main.c) 顶部设置与设备一致的 `CAN_BAUD_KBPS` 和
   `DEVICE_NODE_ID`（默认 500 kbit/s，源地址 8）。
2. 打开 `USER/hipnuc_can_decode.uvprojx`，编译并下载。
3. 例程最多每 200 ms 显示一条新报文，每两秒报告接收情况、无数据或恢复接收。

工程的 CAN 时序基于 APB1 时钟 36 MHz，支持 125、250、500、1000 kbit/s。

## 使用测量值

在 `main.c` 标出的应用代码位置添加处理：

```c
if (sample.valid & HIPNUC_VALID_ACC) {
    float acceleration_x = sample.acc[0]; /* m/s^2 */
    /* Use acceleration_x here. */
}
```

一个样本**仅包含当前 CAN 报文的字段**。例如 roll/pitch 报文不包含
yaw、加速度或角速度。读取前检查 `sample.valid`，例程不会拼接多条报文
形成整包快照。完整定义见 [C 样本头文件](../../c/hipnuc/hipnuc_sample.h)。

[hipnuc_board.c](USER/hipnuc_board.c) 占用 CAN1、与 USB 低优先级共用的
`USB_LP_CAN1_RX0` 中断、作为控制台的 USART1/PA9 和 1 ms SysTick；
`hipnuc_board_init()` 还会设置 NVIC 优先级分组 2。
中断只将报文放入队列，主循环按完整源地址筛选并解码。
64 个槽位的队列最多存放 63 帧，满时丢弃新到报文，不修改主循环正在读取的帧。
软件队列丢帧与硬件 FIFO 溢出分别计数。

持续调用 `hipnuc_board_poll()`，应用处理保持简短。打印会阻塞；
总线数据量大时将 `PRINT_PERIOD_MS` 设为 0。队列容量有限，硬件溢出计数
表示观察到的事件，不能精确统计长时间屏蔽中断期间的全部丢帧。
