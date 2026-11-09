/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GK-407VE (STM32F407VET6) board support, on HAL: clocks, 1 ms tick, LED, console, the IMU's UART or CAN.
 */

#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 168 MHz clock, 1 ms tick, LED PA8, console USART1 (PA9/PA10, 115200 8N1,
 * printf). */
void Board_Init(void);

/* The IMU's interface: call one. */
void Board_InitImuUart(void);   /* USART2: PA2 TX, PA3 RX (see imu_uart.h) */
void Board_InitImuCan(void);    /* CAN1: PB9 TX, PB8 RX, to a transceiver (see imu_can.h) */

uint32_t Board_Ms(void);   /* milliseconds since start */
void Board_LedToggle(void);

/* Report a fatal problem and stop: the LED blinks, and once the console is up the reason is
 * repeated on it. Never returns. */
void Board_Fatal(const char *why) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
