# STM32 例程

[English](README.md) | [中文](README_zh.md)

两个适用于 STM32F103 的简短 Keil／标准外设库工程。

请保留完整仓库，工程需要引用其中共享的 C 源文件。

| 设备连接方式 | 从这里开始 |
| --- | --- |
| TTL 串口，二进制输出 | [串口例程](serial/README_zh.md) |
| CAN 收发器，J1939 输出 | [CAN 例程](can/README_zh.md) |

每个例程的设置和应用代码都在 `USER/main.c` 中，
`USER/hipnuc_board.c` 负责外设初始化和数据接收。
协议解码器直接复用 [C SDK](../c/README_zh.md)。
