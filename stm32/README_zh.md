# STM32 例程

[English](README.md) | [中文](README_zh.md)

用 STM32CubeMX 生成的 STM32F407 Keil MDK 工程。它通过 UART（HI91、HI81、HI83）
或 CAN（J1939）读取 HiPNUC IMU，每 2 s 在串口控制台打印一次摘要。在 GK-407VE
核心板上可以直接运行。换成别的板子时，在 CubeMX 里改引脚；也可以把 `User/`
复制进你自己的 CubeMX 工程（见[移植到你自己的 CubeMX 工程](#移植到你自己的-cubemx-工程)）。

## 目录

| 路径（在 `hipnuc_f407/` 下） | 内容 |
| --- | --- |
| `hipnuc_f407.ioc` | CubeMX 配置：引脚、时钟、外设 |
| `Core/`、`Drivers/` | CubeMX 生成：`main.c`、中断处理、HAL、CMSIS |
| `MDK-ARM/hipnuc_f407.uvprojx` | Keil 工程 |
| `User/` | HiPNUC 代码：`app.c`（例程本身）、`imu_uart.c`、`imu_can.c` |
| `User/hipnuc/` | [`c/hipnuc`](../c/hipnuc) 解码器的副本，不要在这里修改 |

`main.c` 就是 CubeMX 生成的那份，只在 USER CODE 段里加了三行：`#include "app.h"`、
`app_start();` 和 `app_poll();`。

## 快速上手

需要 Keil MDK 5 和 Arm Compiler 6（Arm Compiler 5 也能编译本工程）、STM32F4
器件包（打开工程时 Keil 会提示安装），以及 CMSIS-DAP（DAPLink）或 ST-LINK
调试器。

接线（3.3 V 电平，共地）：

| IMU | GK-407VE |
| --- | --- |
| TX | PA3（USART2 RX） |
| RX | PA2（USART2 TX） |
| GND | GND |

控制台是 USART1：PA9 TX、PA10 RX，115200 8N1。接 USB 转串口模块，或者用
DAPLink 的虚拟串口。

用 CAN 时需要一个 3.3 V CAN 收发器：PB9 接它的 TXD，PB8 接它的 RXD，它的
CAN_H/CAN_L 接 IMU，总线两端各接 120 欧终端电阻。不要把 CAN_H/CAN_L 直接接到
MCU 上。

1. 打开 `hipnuc_f407/MDK-ARM/hipnuc_f407.uvprojx`。
2. 编译（F7）并下载（F8）。工程默认用 CMSIS-DAP 调试器；用 ST-LINK 的话，在
   Options for Target > Debug 里选择。
3. 打开控制台。IMU 为默认设置（115200 波特率，HI91 100 Hz）时显示：

```
HiPNUC STM32F407 example: IMU on USART2 at 115200 baud, receive by interrupt
IMU: <型号>
IMU firmware: <版本>
100 Hz  roll -9.8 pitch -0.3 yaw 0.3 | frames 397 crc 0 err 0 dropped 0
```

每打印一次摘要，PA8 上的 LED 翻转一次。

## 设置

- `User/app.c` 里的 `IMU_INTERFACE`：`IMU_UART`（默认）或 `IMU_CAN`。
- `User/imu_uart.h` 里的 `IMU_UART_DMA`：`0` 用中断逐字节接收；`1` 用 DMA
  接收到环形缓冲区。波特率在 460800 及以上，或者其他中断让 CPU 很忙时，用 DMA。
- 波特率和 CAN 位速率在 CubeMX 的 USART2 和 CAN1 里设置，改完点 GENERATE CODE。
  没有 CubeMX 时，可以直接改 `main.c` 里 `MX_USART2_UART_Init()` 的
  `huart2.Init.BaudRate`，或 `MX_CAN1_Init()` 里的分频值；CubeMX 再次生成时会覆盖
  这几行。
- IMU 默认为 115200 波特率、500 kbit/s、CAN 节点 8（对应 `User/app.c` 里的
  `IMU_NODE`）。

例程的 PCLK1 为 42 MHz 时的 CAN 位时序（BS1 11 TQ、BS2 2 TQ、SJW 1 TQ）：

| 位速率 | Prescaler |
| --- | --- |
| 1 Mbit/s | 3 |
| 500 kbit/s | 6 |
| 250 kbit/s | 12 |
| 125 kbit/s | 24 |

## 控制台提示

| 提示 | 检查 |
| --- | --- |
| `no bytes from the IMU` | 电源、GND、IMU TX 是否接到 PA3，IMU 的输出是否打开 |
| `bytes but no frames` | IMU 的波特率是否和 USART2 一致，是否输出 HI91 或 HI83 |
| `frames this example does not decode` | HI83 的字段选择（`LOG HI83 MAP`） |
| `no CAN frames` | 电源、CAN_H/CAN_L、120 欧终端电阻、收发器的 3.3 V 和 GND、位速率 |
| `HiPNUC messages from node N` | 把 `IMU_NODE` 设为 N |
| `CAN frames but no HiPNUC messages` | IMU 是否输出 J1939 |
| `CAN did not start` | PB8 空闲时必须为高电平：检查收发器的 RXD 和供电 |
| 完全没有输出 | 控制台接线（PA9）和 115200 8N1；板上的 8 MHz 晶振（时钟起不来时例程会停住） |

摘要里的 `err` 是溢出次数（UART 丢掉的字节）加上接收重启的次数，`dropped` 是读缓冲区
放不下而丢掉的字节数（中断模式；DMA 模式下超出一个缓冲区的字节会直接丢失，不计数）。

## 使用数据

在 `app_poll()` 里 `/* use imu here */` 的位置加你的代码。用某个字段之前，先检查
`imu.valid` 里对应的 `HIPNUC_VALID_*` 位；单位写在 `User/hipnuc/hipnuc_data.h` 的注释里。
CAN 上每条 J1939 报文只带一个量：`msg` 是最新的一条报文，`imu` 汇总每个量的最新值。

## 移植到你自己的 CubeMX 工程

CubeMX 设置：

| 设置 | UART 中断 | UART DMA | CAN |
| --- | --- | --- | --- |
| USART 设为 Asynchronous，IMU 的波特率，8N1 | 是 | 是 | |
| USART 全局中断 | 是 | 开或关均可 | |
| USARTx_RX DMA 请求：Circular、Byte、内存地址递增 | | 是 | |
| RX 引脚上拉 | 是 | 是 | 是 |
| CAN 位时序对应 IMU 的位速率，打开 Automatic Bus-Off Management | | | 是 |
| CAN RX0 中断 | | | 是 |

然后：

1. 把 `User/` 复制进你的工程。只需要你所用接口的文件：`app.*`、`hipnuc/`，以及
   `imu_uart.*` 或 `imu_can.*`。
2. 在 Keil 里把这些 `.c` 文件加进一个分组，把 `User` 和 `User/hipnuc` 加进 include
   路径。也可以照 `hipnuc_f407/.extSettings` 在你的 `.ioc` 旁边放一个 `.extSettings`
   文件，CubeMX 每次生成时都会自动加上。分组名不要以 "User" 开头，否则 CubeMX 会把
   它自己的 `Application/User/Core` 分组再复制一份。
3. 在 `main.c` 里加三行：USER CODE Includes 段加 `#include "app.h"`，USER CODE 2 段加
   `app_start();`，USER CODE 3 段（循环里）加 `app_poll();`。
4. 在 `imu_uart.h`（或 `imu_can.h`）里，把两行中的 `huart2`（或 `hcan1`）都换成你的
   句柄：`extern` 声明那一行，和 `IMU_UART_HANDLE`（或 `IMU_CAN_HANDLE`）那一行。用 CAN2
   时，还要把 `imu_can_start()` 里的 `filter.FilterBank` 设为 14：0-13 号滤波器组属于 CAN1。
5. `app.c` 通过 `huart1` 打印，并翻转 CubeMX 里标签为 `LED` 的引脚。把 `huart1` 改成你的
   控制台 UART；LED 引脚标上 `LED`，或者删掉那两行 `HAL_GPIO_TogglePin()`。
6. 如果你的工程里已经有 `HAL_UART_RxCpltCallback()`、`HAL_CAN_RxFifo0MsgPendingCallback()`
   或 `fputc()`，把我们的调用合并进你的函数，再删掉 `app.c` 里的那一份。
7. 在 Keil 里勾选 Use MicroLIB，例程的 `printf()` 才能用。
8. 编译、运行，看提示。

`imu_uart.c` 只用 HAL 的 UART API，其他 STM32 系列也能编译。`imu_can.c` 只适用于
bxCAN；带 FDCAN 的系列需要自己转换成 `hipnuc_can_frame_t`。在其他 MCU 上，用你自己的
UART 或 CAN 驱动实现 `imu_uart.h` 里的三个函数（或 `imu_can.h` 里的两个）即可。

用 STM32CubeIDE 或 VS Code 时，复制一份 `.ioc`，为对应工具链生成工程，加上 `User/`
的源文件和 include 路径；链接时加 `-u _printf_float`，摘要里的数值才能打印出来。
`app.c` 已经为 GCC 提供了 `__io_putchar()`。

## 只用解码器

`User/hipnuc/` 里的 7 个文件用任何 C99 编译器都能编译：把收到的字节交给
`hipnuc_decoder_feed()`，或把 CAN 帧交给 `hipnuc_can_decode()`。用法见各自头文件里的
注释。

## 维护说明

- `User/hipnuc/` 是 `c/hipnuc` 的副本。改了 `c/hipnuc` 之后，运行
  `sh stm32/sync_copies.sh`。CI 会检查副本是否完全一致，以及 Keil 工程是否仍是
  Arm Compiler 6、MicroLIB，并且保留了 `app_start()` 和 `app_poll()` 的调用。
- 本工程用 STM32CubeMX 6.18.1、STM32CubeF4 1.28.3 固件包和 "MDK-ARM V5.32" 工具链设置
  生成。首次生成后在 Keil 里做的设置，CubeMX 再次生成时都会保留：Arm Compiler 6、
  MicroLIB、CMSIS-DAP 及 Reset and Run，以及 `Drivers/STM32F4xx_HAL_Driver` 分组的
  No Warnings。HiPNUC 分组来自 `.extSettings`。换用更新版本的 CubeMX 生成后，检查一下
  这几项。
