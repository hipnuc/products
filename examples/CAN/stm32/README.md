# HiPNUC STM32 CAN 解析示例（基于战舰 STM32F103）

## 概述
- 这是一个面向初学者的最简 STM32F103 CAN 接收解析示例，代码清晰、功能集中。
- 使用标准外设库（StdPeriph）与外部驱动 `drivers/` 下的解析器：
  - 标准帧（11-bit ID）按 CANopen 约定解析。
  - 扩展帧（29-bit ID）按 J1939 约定解析。
- 解析结果以 JSON 文本通过 `USART1` 打印，便于快速观察和集成。

## 硬件要求
- 开发板：正点原子战舰 STM32F103（F103ZE/F103ZET6 均可）
- 引脚连接：
  - `PA11` → CAN_RX（连接收发器 RO）
  - `PA12` → CAN_TX（连接收发器 DI）
  - `PA9/PA10` → USART1（用于 printf 输出，115200）
- CAN 收发器（如 TJA1050）：确保 STB/EN 等引脚置于工作模式，CAN 总线正确终端匹配（两端 120Ω）。

## 主要特性
- 统一的 CAN 帧处理：中断中读取硬件 FIFO0，推入软件环形队列；主循环仅消费队列并打印 JSON。
- 支持常用波特率枚举：`125K/250K/500K/1M`，默认 `500K`。
- 解析器选择：
  - 扩展帧（`IDE == Extended`）→ `hipnuc_j1939_parser`
  - 标准帧（`IDE == Standard`）→ `canopen_parser`
- 未识别帧自动回退为原始 JSON（包含 `id/ide/dlc/data[]`）。

## 工程结构
- `examples/CAN/stm32/USER/main.c`：示例主程序（初始化、ISR、环形队列、解析打印）。
- `drivers/hipnuc_can_common.*`：通用类型与 JSON 输出工具。
- `drivers/hipnuc_j1939_parser.*`：J1939 帧解析。
- `drivers/canopen_parser.*`：CANopen 帧解析。
- `SYSTEM/delay`、`STM32F10x_FWLib`：延时与标准外设库。

## 配置项（main.c）
- 波特率宏：
  - `CAN_BAUD_125K/CAN_BAUD_250K/CAN_BAUD_500K/CAN_BAUD_1M`
  - 选择宏：`#define CAN_BAUD_KBPS CAN_BAUD_500K`（默认 500K）
- 环形队列大小：`CAN_RX_FIFO_SIZE`（默认 32，单生产者 ISR / 单消费者主循环）

## 使用步骤
1. 打开工程：`examples/CAN/stm32/USER/hipnuc_can_decode.uvprojx`
2. 确认硬件连接（PA11/PA12、USART1、收发器与终端电阻）。
3. 如需修改 CAN 波特率，编辑 `main.c` 的 `CAN_BAUD_KBPS` 宏。
4. 编译、下载到板卡，打开串口（115200，8N1）。
5. 在 CAN 总线上发送标准/扩展帧，观察串口 JSON 输出。

## 输出示例
- 标准帧解析（示例：加速度）：
```
{"node_id":3,"hw_ts_us":0,"data":{"acc_x":0.012345,"acc_y":-0.001234,"acc_z":9.812341}}
```
- 扩展帧解析（示例：四元数）：
```
{"node_id":7,"hw_ts_us":0,"data":{"quat_w":0.9999,"quat_x":0.0001,"quat_y":0.0002,"quat_z":0.0003}}
```
- 未识别帧（回退原始）：
```
{"id":123,"ide":"std","dlc":8,"data":[1,2,3,4,5,6,7,8]}
```

## 解析说明
- 节点号提取：`hipnuc_can_extract_node_id(can_id)` 对扩展/标准帧作不同掩码处理。
- JSON 构建：`hipnuc_can_to_json(data, msg_type, out)` 根据 `msg_type` 输出相应字段。
- J1939 PGN 映射：见 `drivers/hipnuc_j1939_parser.c`（如 `PGN_QUAT/PGN_ACCEL/...`）。
- CANopen TPDO 映射：见 `drivers/canopen_parser.c`（`TPDO1/2/3/4/6/7` 基址）。

## 常见问题
- 只能发不能收：检查收发器待机脚、总线终端、对端波特率、引脚是否为 `PA11/PA12`。
- 扩展帧收不到：确认对端确实发送 29-bit ID；本示例滤波器全开放，通常为打印逻辑问题，现已按 `IDE` 分流解析。
- 中断未触发：确认已使能 `CAN_IT_FMP0` 与 `USB_LP_CAN1_RX0_IRQn`，优先级组设置正确。

## 性能与栈设置
- 栈大小建议：在 Keil MDK 工程的启动文件 `CORE/startup_stm32f10x_hd.s` 中调整 `Stack_Size`：
  - 默认值：`Stack_Size EQU 0x00000800`（2KB）
  - 建议在含较多解析与字符串处理的配置下提升到 `0x1000`（4KB）或 `0x2000`（8KB），依据实际内存余量与功能选择。
- 队列大小：`CAN_RX_FIFO_SIZE` 默认 `32`，可在 `main.c` 中按总线负载调节。



