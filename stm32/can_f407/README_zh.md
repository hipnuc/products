# STM32 CAN 例程

[English](README.md) | [中文](README_zh.md)

通过 CAN1 接收 HiPNUC J1939 报文。工程适用于
STM32F407IGT6 开发板，使用 Keil MDK 5、ARM Compiler 5 和 STM32F4 HAL 驱动。
STM32F407 仅支持 **Classic CAN，不支持 CAN FD**。

## 工程结构

`USER/` 存放应用、时钟配置、中断桩和 CAN/USART 板级支持；`CORE/` 仅存放
F407 启动文件；`Libraries/CMSIS/` 与 `Libraries/STM32F4xx_HAL_Driver/` 分别
提供设备头文件和 HAL 驱动。可移植的 HiPNUC J1939 解码器刻意通过
`../../c/hipnuc/` 共享引用，使各平台例程始终使用同一份协议实现。Keil 生成的
`OBJ/` 与 `Listings/` 是构建产物，已由仓库根目录的忽略规则排除，不应提交。

本地库仅保留此 ARM Compiler 5 工程需要的依赖。新增外设时，按需添加驱动文件
及其依赖，不要复制整套 STM32Cube 库。CAN 驱动包含一处本地 FIFO 释放修正，
避免清除尚未处理的硬件溢出标志。

## 接线与运行

| 连接 | 开发板 |
| --- | --- |
| 设备 CAN_H / CAN_L | CAN 收发器 CAN_H / CAN_L |
| 设备 GND | GND |
| 收发器 RXD / TXD | PI9 / PB9（CAN1 RX / TX） |
| 调试输出 TX | PB6 — USART1 TX |

使用兼容 MCU 3.3 V 逻辑的 CAN 收发器，不要将总线直接连接到 PI9/PB9。
总线两个物理末端各接 120 Ω 终端电阻。设备按对应型号要求供电。
用 USB-TTL 转换器连接 PB6/GND，串口助手设置为 115200、8N1。

1. 在 [main.c](USER/main.c) 顶部设置与设备一致的 `CAN_BAUD_KBPS` 和
   `DEVICE_NODE_ID`（默认 500 kbit/s，源地址 8）。
2. 打开 `USER/hipnuc_can_decode_f407.uvprojx`，编译并下载。
3. 例程最多每 200 ms 显示一条新报文，每两秒报告接收情况、无数据或恢复接收。

工程的 CAN 时序基于 APB1 时钟 42 MHz，支持 125、250、500、1000 kbit/s。

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

[hipnuc_board.c](USER/hipnuc_board.c) 管理 CAN1、接收中断和 1 ms SysTick。
中断按完整源地址筛选、解析 J1939 并将完整样本放入队列；主循环仅取出样本。
16 个槽位最多存放 15 个样本，满时丢弃新到样本，不修改主循环正在读取的样本。
软件队列丢样本与硬件 FIFO 溢出分别计数。

持续调用 `hipnuc_board_poll()`，应用处理保持简短。打印会阻塞；
总线数据量大时将 `PRINT_PERIOD_MS` 设为 0。队列容量有限，硬件溢出计数
表示观察到的事件，不能精确统计长时间屏蔽中断期间的全部丢帧。
