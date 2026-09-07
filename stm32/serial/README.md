# STM32 serial example

[English](README.md) | [中文](README_zh.md)

Receives HiPNUC data on USART2 and prints attitude on USART1.
Board: 正点原子 战舰 V3 (STM32F103ZET6), Keil MDK 5 with ARM Compiler 5,
StdPeriph library. Any STM32F10x board works after changing the pins in
`USER/hipnuc_board.c`.

## Wiring

| IMU | Board |
| --- | --- |
| TXD | PA3 (USART2 RX) |
| RXD | PA2 (USART2 TX) |
| 3.3V / GND | 3V3 / GND |

Console: the board's USART1 (PA9/PA10) USB-serial at 115200.

## Run

1. Open `USER/hipnuc_serial_decode.uvprojx`, build, download.
2. Open a terminal on the console port. Every 200 ms you see roll/pitch/yaw,
   acceleration and the frame rate; hints appear when no data or no valid
   frames arrive.
3. Edit `IMU_BAUDRATE` in `USER/main.c` if the device does not use 115200.

## Use the data in your own code

`main.c` is the whole application:

```c
hipnuc_board_init(IMU_BAUDRATE);
while (1) {
    if (hipnuc_board_poll(&sample)) {
        /* sample.roll, sample.pitch, sample.yaw (rad), sample.acc (m/s^2), ... */
    }
}
```

`USER/hipnuc_board.c` owns the UART (DMA circular buffer by default, or one
interrupt per byte with `HIPNUC_BOARD_USE_DMA 0`), the decoder and the
conversion to `hipnuc_sample_t`; `hipnuc_board_stats()` reports bytes, frames,
CRC errors and receive overruns. The decoder files come straight from
`c/hipnuc` (see its README for the field reference).

Call `hipnuc_board_poll()` at least every 10 ms at 921600 baud (the receive
buffer holds 1024 bytes), and keep `printf` off the critical path: at 500 Hz
output a 115200 baud console cannot print every frame.
