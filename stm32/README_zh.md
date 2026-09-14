# STM32 例程

[English](README.md) | [中文](README_zh.md)

本目录包含用于在 STM32 开发板上接收、解析 HiPNUC 数据的 Keil MDK 例程。
请保持仓库目录结构完整：各工程通过 [`../c/hipnuc`](../c/hipnuc) 直接引用
可移植解码器，不维护各自的本地副本。

## 选择例程

| 目录 | 目标芯片 | 输入 / 输出 | 外设库 | Keil 工程 |
| --- | --- | --- | --- | --- |
| [`serial`](serial/README_zh.md) | STM32F103ZET6 | USART2 接收 HiPNUC 二进制数据；USART1 输出日志 | STM32F10x 标准外设库 | [`USER/hipnuc_serial_decode.uvprojx`](serial/USER/hipnuc_serial_decode.uvprojx) |
| [`can`](can/README_zh.md) | STM32F103ZET6 | CAN1 接收 HiPNUC J1939 数据；USART1 输出日志 | STM32F10x 标准外设库 | [`USER/hipnuc_can_decode.uvprojx`](can/USER/hipnuc_can_decode.uvprojx) |
| [`can_f407`](can_f407/README_zh.md) | STM32F407IGT6 | CAN1 接收 HiPNUC J1939 数据；USART1 输出日志 | STM32F4 HAL | [`USER/hipnuc_can_decode_f407.uvprojx`](can_f407/USER/hipnuc_can_decode_f407.uvprojx) |

## 硬件资源概览

| 例程 | HiPNUC / CAN 侧 MCU 引脚 | 调试输出 TX |
| --- | --- | --- |
| `serial` | USART2：PA3 RX、PA2 TX | USART1 PA9 |
| `can` | CAN1：PA11 RX、PA12 TX | USART1 PA9 |
| `can_f407` | CAN1：PI9 RX、PB9 TX（AF9） | USART1 PB6 |

串口例程使用 3.3 V TTL 电平，并且设备与开发板必须共地。CAN 例程必须经由
兼容 3.3 V 逻辑的 CAN 收发器连接；不得将 CAN_H/CAN_L 直接连接到 MCU。
CAN 总线的两个物理末端应各接一个 120 Ω 终端电阻。

## 编译与定制

1. 使用 Keil MDK 5 打开上表对应工程并编译。F103 工程使用 ARM Compiler 5；
   F407 工程同样配置为该工具链，并使用 STM32F4 HAL。
2. 在所选工程的 `USER/main.c` 中配置设备链路：串口例程设置
   `IMU_BAUDRATE`，CAN 例程设置 `CAN_BAUD_KBPS` 和 `DEVICE_NODE_ID`。
3. 在 `main.c` 标出的应用位置加入业务处理。读取已解析字段前，必须检查对应
   的 `HIPNUC_VALID_*` 有效位。
4. 各子目录的 README 记录了该例程的详细接线、支持的波特率及运行诊断信息。

解码器每次返回的是当前接收报文携带的测量值，不会将不同报文、不同时刻的字段
拼接成一份虚拟快照。字段单位和有效位定义见共享的 [C SDK](../c/README_zh.md)。

## 工程组织

`USER/` 存放应用与板级支持。F103 工程将启动文件和标准外设库分别保存在
`CORE/`、`STM32F10x_FWLib/` 中。`can_f407` 在 `Libraries/` 下保存 CMSIS 和
`STM32F4xx_HAL_Driver`；GPIO、USART、CAN、时钟和中断均通过 HAL API 配置。

Keil 生成的 `OBJ/`、`Listings/`、map 文件和 IDE 用户文件均属于构建产物，已由
仓库根目录的 `.gitignore` 排除，无需提交。
