/*
 * Board support for the HiPNUC serial example (STM32F103, StdPeriph).
 *
 * Owns USART2 reception (DMA circular buffer or RXNE interrupt), the
 * decoder and sample conversion. main.c calls this interface without
 * touching buffers or interrupts.
 */

#ifndef HIPNUC_BOARD_H
#define HIPNUC_BOARD_H

#include <stdint.h>

#include "hipnuc_sample.h"

/* Receive mode: 1 = DMA circular buffer (recommended),
 * 0 = one interrupt per byte. */
#ifndef HIPNUC_BOARD_USE_DMA
#define HIPNUC_BOARD_USE_DMA 1
#endif

/* Power-of-two buffer size, 2..32768. At 921600 baud, 1024 bytes take 11.1 ms.
 * Both polling and DMA TC interrupt service must run within that time. */
#ifndef HIPNUC_BOARD_RX_BUFFER_SIZE
#define HIPNUC_BOARD_RX_BUFFER_SIZE 1024
#endif

typedef struct {
    uint32_t bytes;          /* bytes received */
    uint32_t frames;         /* frames decoded */
    uint32_t crc_errors;
    uint32_t invalid_frames;
    uint32_t overruns;       /* DMA overwritten spans, or IRQ dropped bytes */
    uint32_t hardware_errors; /* observed UART overrun/framing/noise events, not lost byte count */
} hipnuc_board_stats_t;

/* Initialize once: USART2 (PA3 RX, PA2 TX), USART1 console (PA9, 115200),
 * and a 1 ms SysTick. `baudrate` must match the device. */
void hipnuc_board_init(uint32_t baudrate);

/*
 * Consume received bytes and decode. Returns 1 and fills `sample` when a
 * new measurement is available, 0 otherwise. Call from the main loop; each
 * call returns at most one sample, so call it until it returns 0. After a
 * UART error, buffered bytes and the partial frame are discarded on polling.
 */
int hipnuc_board_poll(hipnuc_sample_t *sample);

const hipnuc_board_stats_t *hipnuc_board_stats(void);

/* Send an ASCII command to the device, e.g. "LOG HI91 ONTIME 0.01". */
void hipnuc_board_send_command(const char *command);

/* Milliseconds since hipnuc_board_init() (SysTick). */
uint32_t hipnuc_board_millis(void);

#endif /* HIPNUC_BOARD_H */
