# STM32 CAN example

[English](README.md) | [中文](README_zh.md)

Read HiPNUC J1939 messages with CAN1. The supplied project targets the
STM32F407IGT6 and uses Keil MDK 5 / ARM Compiler 5 with the STM32F4 HAL driver.
STM32F407 supports **Classic CAN**, not CAN FD.

## Project layout

`USER/` contains the application, clock setup, interrupt stubs and CAN/USART
board support. `CORE/` contains only the F407 startup file, while
`Libraries/CMSIS/` and `Libraries/STM32F4xx_HAL_Driver/` provide the local
device and HAL dependencies. The portable HiPNUC J1939 decoder is intentionally
referenced from `../../c/hipnuc/`, so fixes to the decoder remain shared by
every platform example. Keil-generated `OBJ/` and `Listings/` directories are
build products and are ignored by the repository.

The local libraries contain only the dependencies needed by this ARM Compiler 5
project. When adding peripherals, add the required driver files and their
dependencies rather than copying the full STM32Cube library. The CAN driver has
a local FIFO-release fix that preserves pending hardware overrun flags.

## Connect and run

| Connection | Board |
| --- | --- |
| Device CAN_H / CAN_L | CAN transceiver CAN_H / CAN_L |
| Device GND | GND |
| Transceiver RXD / TXD | PI9 / PB9 (CAN1 RX / TX) |
| Console TX | PB6 — USART1 TX |

Use a CAN transceiver compatible with the MCU's 3.3 V logic; do not wire the
bus directly to PI9/PB9. Terminate both physical ends of the bus with
120 Ω. Power the device according to its model. Connect a USB-TTL adapter to
PB6/GND and open its terminal at 115200, 8N1.

1. Set `CAN_BAUD_KBPS` and `DEVICE_NODE_ID` in [main.c](USER/main.c) to match
   the device (defaults: 500 kbit/s, source address 8).
2. Open `USER/hipnuc_can_decode_f407.uvprojx`, build and download.
3. The console displays one new message at most every 200 ms, and reports
   reception, missing traffic or recovery every two seconds.

The included timing assumes a 42 MHz APB1 clock. Supported bitrates are
125, 250, 500 and 1000 kbit/s.

## Use the measurements

Add your processing to the marked block in `main.c`:

```c
if (sample.valid & HIPNUC_VALID_ACC) {
    float acceleration_x = sample.acc[0]; /* m/s^2 */
    /* Use acceleration_x here. */
}
```

One sample contains **only the fields of the current CAN message**. A
roll/pitch message does not contain yaw, acceleration or gyro readings.
Check `sample.valid`; the example does not assemble a combined snapshot.
Field definitions are in the [C sample header](../../c/hipnuc/hipnuc_sample.h).

[hipnuc_board.c](USER/hipnuc_board.c) owns CAN1, its RX interrupt and the
1 ms SysTick. The interrupt filters the full source address, decodes J1939
frames and queues complete samples; the main loop only consumes them. The
16-slot software FIFO holds 15 samples and drops new arrivals when full,
preserving samples being read. Software drops and hardware FIFO overruns
have separate counters.

Call `hipnuc_board_poll()` continuously and keep application processing
short. Printing is blocking; set `PRINT_PERIOD_MS` to 0 for heavy traffic.
The queue is finite, and hardware overflow counts are observed events,
not an exact count of all frames lost while interrupts were blocked.
