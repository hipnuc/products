# STM32 examples

[English](README.md) | [中文](README_zh.md)

Two small Keil / standard peripheral library projects for STM32F103.

Keep the complete repository so the projects can find the shared C sources.

| Device connection | Start here |
| --- | --- |
| TTL serial, binary output | [Serial example](serial/README.md) |
| CAN transceiver, J1939 output | [CAN example](can/README.md) |

Each example has settings and an application block in `USER/main.c`.
`USER/hipnuc_board.c` handles the board's peripherals and reception;
the protocol decoder is shared directly with the [C SDK](../c/README.md).
