#include <stdio.h>
#include <stdlib.h>
#include "stubs/stm32f10x.h"
#define __STM32F10x_H
#include "../serial/USER/hipnuc_board.c"

static unsigned decoded_bytes;
static uint8_t decoded[RX_SIZE * 4];
static uint32_t written;
int hipnuc_input(hipnuc_raw_t *raw, uint8_t byte)
{
    (void)raw;
    decoded[decoded_bytes++] = byte;
    return 0;
}
int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *sample)
{ (void)raw; (void)sample; return 0; }

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); exit(1); \
} } while (0)

static void reset(void)
{
    fake_counter_hook = NULL;
    fake_barrier_hook = NULL;
    fake_irq_restore_hook = NULL;
    fake_usart_reads_while_dma_enabled = 0;
    USART2->SR = 0;
    hipnuc_board_init(921600);
    decoded_bytes = written = 0;
}

/* Hardware continues writing even when CPU interrupts are masked. */
static void write_bytes(unsigned count, int service_interrupt)
{
    while (count--) {
        rx_buf[written & (RX_SIZE - 1U)] = (uint8_t)written;
        written++;
        fake_dma.CNDTR = RX_SIZE - (written & (RX_SIZE - 1U));
        if (fake_dma.CNDTR == RX_SIZE) {
            fake_tc = 1;
            if (service_interrupt) DMA1_Channel6_IRQHandler();
        }
    }
}

static void wrap_during_snapshot(void)
{
    CHECK(fake_irq_mask == 1);
    fake_counter_hook = NULL;
    write_bytes(3, 0);
}

static void overwrite_before_copy(void)
{
    fake_barrier_hook = NULL;
    write_bytes(1, 0);
}

static void deliver_error_interrupt(void)
{
    fake_irq_restore_hook = NULL;
    USART2_IRQHandler();
}

static void error_before_copy(void)
{
    fake_barrier_hook = NULL;
    USART2->SR = USART_SR_NE | USART_SR_RXNE;
    fake_irq_restore_hook = deliver_error_interrupt;
}

int main(void)
{
    hipnuc_sample_t sample;
    unsigned i;
    const uint32_t errors[] = {USART_SR_ORE, USART_SR_FE, USART_SR_NE,
                              USART_SR_ORE | USART_SR_FE | USART_SR_NE};
    reset();
    CHECK(fake_usart_error_irq_enabled); /* DMA reception must also observe UART faults. */
    /* A normal DMA wrap must not be mistaken for an overrun on the next poll. */
    write_bytes(RX_SIZE - 100, 1);
    hipnuc_board_poll(&sample);
    write_bytes(120, 1);
    hipnuc_board_poll(&sample);
    write_bytes(1, 1);
    hipnuc_board_poll(&sample);
    CHECK(stats.overruns == 0 && decoded_bytes == RX_SIZE + 21);
    for (i = 0; i < decoded_bytes; ++i) CHECK(decoded[i] == (uint8_t)i);

    /* Count a TC flag whose ISR has not run, without counting it twice later. */
    reset();
    write_bytes(RX_SIZE - 2, 1);
    hipnuc_board_poll(&sample);
    write_bytes(5, 0);
    hipnuc_board_poll(&sample);
    DMA1_Channel6_IRQHandler();
    hipnuc_board_poll(&sample);
    CHECK(stats.overruns == 0 && decoded_bytes == RX_SIZE + 3 && rx_wraps == 1);

    /* TC/CNDTR changes between reads: retry the snapshot, consume each byte once. */
    reset();
    write_bytes(RX_SIZE - 1, 1);
    hipnuc_board_poll(&sample);
    fake_counter_hook = wrap_during_snapshot;
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == RX_SIZE + 2 && stats.overruns == 0);
    CHECK(fake_irq_mask == 0);

    reset();
    write_bytes(RX_SIZE, 1);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == RX_SIZE && stats.overruns == 0); /* exactly full is still intact */

    reset();
    decoder.nbyte = 17;
    decoder.crc_error_count = 3;
    decoder.invalid_count = 4;
    write_bytes(RX_SIZE * 2 + 7, 1);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 0 && stats.overruns == 1 && decoder.nbyte == 0);
    CHECK(decoder.crc_error_count == 3 && decoder.invalid_count == 4);
    write_bytes(2, 1);
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 2 && decoded[0] == 7 && decoded[1] == 8);

    /* If overwritten between availability check and byte copy, never decode it. */
    reset();
    write_bytes(RX_SIZE, 1);
    fake_barrier_hook = overwrite_before_copy;
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 0 && stats.overruns == 1);

    /* Accumulated byte counters wrap modulo 2^32, not as a receive loss. */
    reset();
    rx_wraps = UINT32_MAX / RX_SIZE;
    fake_dma.CNDTR = 1;
    rx_consumed = UINT32_MAX - 2U;
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 2 && rx_consumed == UINT32_MAX);
    rx_wraps++;
    fake_dma.CNDTR = RX_SIZE - 5;
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 8 && stats.overruns == 0 && rx_consumed == 5);
    /* Hardware errors invalidate buffered data even when DMA did not overflow. */
    for (i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
        reset();
        write_bytes(3, 1);
        decoder.nbyte = 7;
        decoder.invalid_count = 2;
        USART2->SR = errors[i] | USART_SR_RXNE;
        USART2_IRQHandler();
        CHECK(fake_usart_dma_enabled && !fake_usart_reads_while_dma_enabled);
        CHECK(!(USART2->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_RXNE)));
        hipnuc_board_poll(&sample);
        CHECK(decoded_bytes == 0 && decoder.nbyte == 0 && decoder.invalid_count == 2);
        CHECK(hipnuc_board_stats()->hardware_errors == 1 && stats.overruns == 0);
        write_bytes(2, 1);
        hipnuc_board_poll(&sample);
        CHECK(decoded_bytes == 2 && decoded[0] == 3 && decoded[1] == 4);
    }

    /* A pending error IRQ after the DMA snapshot must not feed a copied byte. */
    reset();
    write_bytes(3, 1);
    fake_barrier_hook = error_before_copy;
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 0 && hipnuc_board_stats()->hardware_errors == 1);

    /* TXE polling is also an SR read: capture errors even without RXNE. */
    reset();
    write_bytes(3, 1);
    decoder.nbyte = 5;
    USART2->SR = USART_SR_TXE | USART_SR_FE;
    hipnuc_board_send_command("LOG VERSION");
    hipnuc_board_poll(&sample);
    CHECK(decoded_bytes == 0 && decoder.nbyte == 0);
    CHECK(hipnuc_board_stats()->hardware_errors == 1);
    CHECK(fake_usart_dma_enabled && !fake_usart_reads_while_dma_enabled && !fake_irq_mask);
    puts("DMA: wrap, snapshot races, overwrite, UART faults and recovery passed");
    return 0;
}
