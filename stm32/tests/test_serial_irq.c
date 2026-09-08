#include <stdio.h>
#include <stdlib.h>
#include "stubs/stm32f10x.h"
#define __STM32F10x_H
#define HIPNUC_BOARD_USE_DMA 0
#include "../serial/USER/hipnuc_board.c"

static unsigned decoded_bytes;
static uint8_t decoded[RX_SIZE * 2];
int hipnuc_input(hipnuc_raw_t *raw, uint8_t byte)
{ (void)raw; decoded[decoded_bytes++] = byte; return 0; }
int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *s)
{ (void)raw; (void)s; return 0; }
#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); exit(1); \
} } while (0)

static void send_byte(uint8_t value)
{ USART2->SR = USART_SR_RXNE; USART2->DR = value; USART2_IRQHandler(); }

/* A higher-priority interrupt delays the DR read until a new byte overruns it. */
static void overrun_after_status(void)
{
    fake_irq_restore_hook = NULL;
    USART2->SR |= USART_SR_ORE;
}

int main(void)
{
    hipnuc_sample_t sample;
    unsigned i;
    const uint32_t errors[] = {USART_SR_ORE, USART_SR_FE, USART_SR_NE,
                              USART_SR_ORE | USART_SR_FE | USART_SR_NE};
    hipnuc_board_init(115200);
    for (i = 0; i < RX_SIZE - 1; ++i) send_byte((uint8_t)i);
    hipnuc_board_poll(&sample);
    send_byte(17);
    send_byte(18);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == RX_SIZE + 1 && stats.overruns == 0);
    for (i = 0; i < RX_SIZE - 1; ++i) CHECK(decoded[i] == (uint8_t)i);
    CHECK(decoded[RX_SIZE - 1] == 17 && decoded[RX_SIZE] == 18);

    hipnuc_board_init(115200);
    decoded_bytes = 0;
    decoder.nbyte = 7;
    decoder.crc_error_count = 2;
    for (i = 0; i < RX_SIZE - 1; ++i) send_byte(42);
    send_byte(99);
    CHECK(rx_tail == 0 && rx_buf[0] == 42 && rx_dropped == 1);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 0 && decoder.nbyte == 0 && decoder.crc_error_count == 2);
    CHECK(stats.bytes == RX_SIZE && stats.overruns == 1);
    send_byte(73);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 1 && decoded[0] == 73);

    for (i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        hipnuc_board_init(115200);
        decoded_bytes = 0;
        decoder.nbyte = 7;
        decoder.crc_error_count = 2;
        send_byte(42);
        USART2->SR = USART_SR_RXNE | errors[i];
        USART2->DR = 99;
        USART2_IRQHandler();
        hipnuc_board_poll(&sample);
        CHECK(decoded_bytes == 0 && decoder.nbyte == 0);
        CHECK(hipnuc_board_stats()->hardware_errors == 1 && stats.overruns == 0);
        CHECK(decoder.crc_error_count == 2 && rx_tail == rx_head);
        CHECK(!(USART2->SR & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)));
        send_byte(73);
        hipnuc_board_poll(&sample);
        CHECK(decoded_bytes == 1 && decoded[0] == 73);
    }

    /* Command TX polling must not silently clear a receive error. */
    decoder.nbyte = 5;
    USART2->SR = USART_SR_TXE | USART_SR_ORE;
    hipnuc_board_send_command("LOG VERSION");
    hipnuc_board_poll(&sample);
    CHECK(hipnuc_board_stats()->hardware_errors == 2 && decoder.nbyte == 0);

    hipnuc_board_init(115200);
    decoded_bytes = 0;
    decoder.nbyte = 5;
    fake_irq_restore_hook = overrun_after_status;
    send_byte(99);
    if (USART2->SR & USART_SR_ORE) USART2_IRQHandler();
    hipnuc_board_poll(&sample);
    CHECK(hipnuc_board_stats()->hardware_errors == 1);
    CHECK(decoded_bytes == 0 && decoder.nbyte == 0 && !fake_irq_mask);
    puts("IRQ: wrap, full-queue drop, UART faults, gap reset and recovery passed");
    return 0;
}
