/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The UART to the IMU (USART2), on the STM32 HAL. This file and imu_uart.c are
 * the only IMU code that depends on the MCU: to port the example, implement
 * the three functions below with your platform's UART driver (the two
 * counters are optional) and drop the STM32 HAL part.
 */

#ifndef IMU_UART_H
#define IMU_UART_H

#include <stddef.h>
#include <stdint.h>

#define IMU_UART_BAUD 115200U   /* the IMU's factory setting */

/* 0: receive by interrupt, one byte at a time (the simplest).
 * 1: receive by DMA into a circular buffer (460800 baud and above, or a busy
 *    system). Read at least every 40 ms at 921600 baud with a busy line, or
 *    every 0.4 s with 100 Hz HI91: bytes beyond one buffer of 4 KB are lost
 *    without notice (the frame rate drops). */
#define IMU_UART_DMA 0

/* ---- What the example needs from a UART ---- */
void imu_uart_start(void);                       /* start receiving */
size_t imu_uart_read(uint8_t *buf, size_t size);  /* bytes received since the last call, at most size */
void imu_uart_write(const char *text);            /* send a command line; blocks while it goes out */

extern volatile uint32_t imu_uart_errors;    /* overruns (bytes the UART lost) and restarts of reception */
extern volatile uint32_t imu_uart_dropped;   /* bytes dropped because the read buffer was full (interrupt mode) */

/* ---- STM32 HAL only ---- */
#include "stm32f4xx_hal.h"

/* Call it from your HAL_UART_RxCpltCallback() (main.c shows how); it ignores
 * other UARTs. Interrupt mode uses it; in DMA mode it does nothing. */
void imu_uart_rx_complete(UART_HandleTypeDef *huart);

#endif /* IMU_UART_H */
