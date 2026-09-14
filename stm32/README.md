# STM32 examples

[English](README.md) | [中文](README_zh.md)

Keil MDK examples for receiving and decoding HiPNUC data on STM32 boards.
Keep the repository layout intact: every project compiles the portable decoder
directly from [`../c/hipnuc`](../c/hipnuc), rather than maintaining a local
copy.

## Choose an example

| Directory | Target | Input / output | Peripheral library | Keil project |
| --- | --- | --- | --- | --- |
| [`serial`](serial/README.md) | STM32F103ZET6 | HiPNUC binary data on USART2; console on USART1 | STM32F10x StdPeriph | [`USER/hipnuc_serial_decode.uvprojx`](serial/USER/hipnuc_serial_decode.uvprojx) |
| [`can`](can/README.md) | STM32F103ZET6 | HiPNUC J1939 data on CAN1; console on USART1 | STM32F10x StdPeriph | [`USER/hipnuc_can_decode.uvprojx`](can/USER/hipnuc_can_decode.uvprojx) |
| [`can_f407`](can_f407/README.md) | STM32F407IGT6 | HiPNUC J1939 data on CAN1; console on USART1 | STM32F4 HAL | [`USER/hipnuc_can_decode_f407.uvprojx`](can_f407/USER/hipnuc_can_decode_f407.uvprojx) |

## Hardware summary

| Example | HiPNUC / CAN-side MCU pins | Console TX |
| --- | --- | --- |
| `serial` | USART2: PA3 RX, PA2 TX | USART1 PA9 |
| `can` | CAN1: PA11 RX, PA12 TX | USART1 PA9 |
| `can_f407` | CAN1: PI9 RX, PB9 TX (AF9) | USART1 PB6 |

Serial examples require 3.3 V TTL signals and a common ground. CAN examples
require a suitable 3.3 V CAN transceiver; never connect CAN_H/CAN_L directly
to the MCU. Terminate both physical ends of the CAN bus with 120 ohms.

## Build and customize

1. Open the project listed above with Keil MDK 5 and build it. The F103
   projects use ARM Compiler 5 and the F407 project is configured for the
   same toolchain and STM32F4 HAL.
2. Configure the device link in the selected `USER/main.c`:
   `IMU_BAUDRATE` for serial, or `CAN_BAUD_KBPS` and `DEVICE_NODE_ID` for CAN.
3. Add application-specific processing at the marked location in `main.c`.
   Check the corresponding `HIPNUC_VALID_*` bit before using a decoded field.
4. Consult the example README for its detailed wiring, supported bitrates and
   runtime diagnostics.

The decoder returns a measurement for the current received frame; it does not
combine values from different frames into a synthetic snapshot. Field units
and validity flags are defined by the shared [C SDK](../c/README.md).

## Project organization

`USER/` contains application code and board support. F103 projects keep their
startup files and StdPeriph sources in `CORE/` and `STM32F10x_FWLib/`.
`can_f407` keeps CMSIS and `STM32F4xx_HAL_Driver` under `Libraries/`; GPIO,
USART, CAN, clock and interrupt setup are performed through HAL APIs.

Keil-generated `OBJ/`, `Listings/`, map files and IDE user files are build
artifacts and are excluded by the repository's `.gitignore`.
