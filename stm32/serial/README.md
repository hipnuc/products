# STM32 serial example

[English](README.md) | [中文](README_zh.md)

Read HiPNUC binary data on USART2 and view a few measurements on USART1.
The supplied project targets the 正点原子 战舰 V3 board (STM32F103ZET6),
Keil MDK 5 / ARM Compiler 5 and the STM32 standard peripheral library.

## Connect and run

| Signal | STM32 pin |
| --- | --- |
| Device TX | PA3 — USART2 RX |
| Device RX | PA2 — USART2 TX |
| Device GND | GND |
| Console TX | PA9 — USART1 TX |

Use 3.3 V TTL serial signals. Power the device according to its model; do
not connect an RS-232 or RS-485 signal directly to the MCU. Connect a USB-TTL
adapter to PA9/GND and open its terminal at 115200, 8N1.

1. Set `IMU_BAUDRATE` at the top of [main.c](USER/main.c) to the device baudrate.
2. Open `USER/hipnuc_serial_decode.uvprojx`, build and download.
3. Enable a supported output on the device: binary HI91/HI81/HI83, or NMEA
   GGA/RMC. A new sample is displayed at most every 200 ms; every two seconds
   the console reports reception, missing data or recovery.

## Use the measurements

Add your code to the marked application block in `main.c`:

```c
if (sample.valid & HIPNUC_VALID_ACC) {
    float acceleration_x = sample.acc[0]; /* m/s^2 */
    /* Use acceleration_x here. */
}
```

Each call to `hipnuc_board_poll(&sample)` returns at most one new sample.
Check the matching `HIPNUC_VALID_*` bit before reading a field; absent
fields are not measurements. Angles are rad and angular velocity is rad/s.
The [C sample header](../../c/hipnuc/hipnuc_sample.h) defines all fields.

`hipnuc_board_send_command("LOG HI91 ONTIME 0.01")` writes an ASCII command on
PA2; the example never calls it, so PA2 is needed only if you do.

[hipnuc_board.c](USER/hipnuc_board.c) contains the UART setup and reception.
It owns USART2 and its interrupt, USART1/PA9 for the console, the 1 ms SysTick
and — with `HIPNUC_BOARD_USE_DMA` set to 1 — DMA1 channel 6; `hipnuc_board_init()`
also selects NVIC priority group 2. Do not configure these resources elsewhere. The decoder is referenced directly from `c/hipnuc/`.

The default DMA buffer holds 1024 bytes: 88.9 ms at 115200 or 11.1 ms at
921600 baud, 8N1. The main loop **and the DMA interrupt** must be serviced
faster than one buffer time. Keep your application short and increase
`PRINT_PERIOD_MS` (or set it to 0) for heavy traffic. Console
printing is blocking; the example does not guarantee lossless reception
under every application load.

An overwrite increments the overrun counter and discards the buffered bytes
and the partial frame. A DMA flag cannot count multiple wraps while interrupts are
blocked, so an overrun counter of zero is not proof that no bytes were lost.
UART overrun, framing and noise errors have a separate counter. After an
observed error, the next poll discards buffered bytes and the partial frame
before resuming reception. This counts observed error events, not lost bytes.
For byte-interrupt reception, set `HIPNUC_BOARD_USE_DMA` to `0` in
[hipnuc_board.h](USER/hipnuc_board.h); DMA is preferable at high baudrates.
