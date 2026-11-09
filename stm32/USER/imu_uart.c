/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The UART to the IMU, on the STM32 HAL; see imu_uart.h. huart2 (and, in DMA
 * mode, its receive DMA) is set up in board.c, the way CubeMX sets it up in
 * MX_USART2_UART_Init() and MX_DMA_Init(). In CubeMX: enable the USART2
 * global interrupt (interrupt mode), or add the USART2_RX DMA request in
 * Circular mode, byte width (DMA mode); for another UART, change IMU below.
 */

#include "imu_uart.h"

#include <string.h>

extern UART_HandleTypeDef huart2;
#define IMU (&huart2)

#define BUFFER_SIZE 4096U   /* a power of two */

static volatile uint8_t buffer[BUFFER_SIZE];
volatile uint32_t imu_uart_errors, imu_uart_dropped;

#if !IMU_UART_DMA

/* Interrupt mode: HAL delivers one byte per interrupt into rx_byte; the
 * callback moves it into the buffer and asks for the next one. When HAL stops
 * reception (an overrun it cannot recover, or a transmit that holds the
 * handle), imu_uart_read() starts it again. */
static uint8_t rx_byte;
static volatile uint32_t head, tail;   /* free-running counts: head is written by the interrupt, tail by the main loop */

static void start_receive(void)
{
    __HAL_UART_CLEAR_OREFLAG(IMU);   /* a pending overrun would stop reception again at once */
    HAL_UART_Receive_IT(IMU, &rx_byte, 1);
}

void imu_uart_start(void)
{
    start_receive();
}

void imu_uart_rx_complete(UART_HandleTypeDef *huart)
{
    if (huart != IMU) return;
    if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) imu_uart_errors++;   /* bytes were lost before this one */
    if (head - tail < BUFFER_SIZE) {
        buffer[head % BUFFER_SIZE] = rx_byte;
        head++;
    } else {
        imu_uart_dropped++;
    }
    /* This fails when the main loop is just starting HAL_UART_Transmit() (HAL
     * locks the handle); imu_uart_read() then starts reception again. */
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
}

size_t imu_uart_read(uint8_t *buf, size_t size)
{
    size_t n = 0;
    uint32_t h;
    if (IMU->RxState != HAL_UART_STATE_BUSY_RX) {   /* stopped by an overrun or a transmit */
        imu_uart_errors++;
        start_receive();
    }
    h = head;
    while (tail != h && n < size) {
        buf[n++] = buffer[tail % BUFFER_SIZE];
        tail++;
    }
    return n;
}

#else

/* DMA mode: the DMA writes into the buffer in a circle; the main loop reads up
 * to the DMA's write position. No interrupt is needed. */
static uint32_t tail;   /* next byte to read */

static void start_receive(void)
{
    tail = 0;
    if (IMU->hdmarx->State != HAL_DMA_STATE_READY) HAL_DMA_Init(IMU->hdmarx);   /* stop the old transfer */
    HAL_UART_Receive_DMA(IMU, (uint8_t *)buffer, BUFFER_SIZE);
    /* A noise or framing error then costs only its bytes; with the error
     * interrupt on, HAL stops the DMA at each one. */
    __HAL_UART_DISABLE_IT(IMU, UART_IT_ERR);
}

void imu_uart_start(void)
{
    start_receive();
}

void imu_uart_rx_complete(UART_HandleTypeDef *huart)
{
    (void)huart;   /* interrupt mode only */
}

size_t imu_uart_read(uint8_t *buf, size_t size)
{
    size_t n = 0;
    uint32_t pos;
    if (__HAL_UART_GET_FLAG(IMU, UART_FLAG_ORE)) {   /* the DMA was late: the UART receives nothing until this is cleared */
        imu_uart_errors++;
        __HAL_UART_CLEAR_OREFLAG(IMU);
    }
    pos = (BUFFER_SIZE - __HAL_DMA_GET_COUNTER(IMU->hdmarx)) % BUFFER_SIZE;
    while (tail != pos && n < size) {
        buf[n++] = buffer[tail];
        tail = (tail + 1U) % BUFFER_SIZE;
    }
    if (tail == pos && IMU->RxState != HAL_UART_STATE_BUSY_RX) {   /* HAL stopped reception: a transmit timed out */
        imu_uart_errors++;
        start_receive();
    }
    return n;
}

#endif

void imu_uart_write(const char *text)
{
    HAL_UART_Transmit(IMU, (const uint8_t *)text, (uint16_t)strlen(text), 100);
}
