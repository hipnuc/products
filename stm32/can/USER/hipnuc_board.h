/* STM32F103 CAN1 / StdPeriph. The interrupt receives; the main loop decodes. */
#ifndef HIPNUC_BOARD_H
#define HIPNUC_BOARD_H
#include <stdint.h>
#include "hipnuc_sample.h"

typedef struct {
    uint32_t received;          /* all frames drained from the hardware FIFO */
    uint32_t frames;            /* samples decoded from the selected source */
    uint32_t invalid_frames;
    uint32_t queue_drops;       /* new frames dropped when the software queue is full */
    uint32_t hardware_overruns; /* observed hardware FIFO overflow events */
} hipnuc_board_stats_t;

/* Initialize once: CAN1 PA11/PA12, USART1 console PA9 at 115200, 1 ms SysTick.
 * APB1 must be 36 MHz. Bitrate is 125, 250, 500 or 1000 kbit/s.
 * Returns 1 on success, 0 on invalid bitrate or CAN initialization failure. */
int hipnuc_board_init(uint16_t bitrate_kbps, uint8_t source_address);

/* Main loop only: return 1 with one new frame's fields, or 0 when drained.
 * No cross-frame accumulation. Check sample.valid for each field. */
int hipnuc_board_poll(hipnuc_sample_t *sample);
const hipnuc_board_stats_t *hipnuc_board_stats(void);
uint32_t hipnuc_board_millis(void);
#endif
