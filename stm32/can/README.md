# STM32 CAN (J1939) example

[English](README.md) | [中文](README_zh.md)

Receives HiPNUC J1939 frames on CAN1 and prints the merged attitude on
USART1. Board: 正点原子 战舰 V3 (STM32F103ZET6) with its CAN transceiver,
Keil MDK 5 with ARM Compiler 5, StdPeriph library.

## Wiring

| Device | Board |
| --- | --- |
| CAN_H / CAN_L | transceiver CAN_H / CAN_L (PA11 RX, PA12 TX) |
| GND | GND |

Terminate the bus with 120 Ω at both ends. Console: USART1 (PA9/PA10)
USB-serial at 115200.

## Run

1. Open `USER/hipnuc_can_decode.uvprojx`, build, download.
2. Open a terminal on the console port. Every 200 ms you see the latest
   roll/pitch/yaw, acceleration, gyro and the frame count.
3. Settings at the top of `USER/main.c`: `CAN_BAUD_KBPS` (device default
   500), `DEVICE_NODE_ID` (device default 8).

## Use the data in your own code

Each J1939 frame carries a few fields. The loop in `main.c` decodes every
frame with `hipnuc_j1939_parse()` into `part` and merges it into `merged`
with `hipnuc_j1939_merge()`; both are `hipnuc_sample_t` (SI units, see
`c/hipnuc/README.md`). Frames from other source addresses are ignored.

STM32F1 has classic CAN only; the CANFD83 frame needs a CAN FD controller.
