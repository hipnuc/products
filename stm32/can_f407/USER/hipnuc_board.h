/* STM32F407IGT6 CAN1 support.  The RX ISR decodes and queues samples. */
#ifndef HIPNUC_BOARD_H
#define HIPNUC_BOARD_H

#include <stdint.h>
#include "hipnuc_sample.h"

typedef struct {
    uint32_t received;
    uint32_t frames;
    uint32_t invalid_frames;
    uint32_t queue_drops;
    uint32_t hardware_overruns;
} hipnuc_board_stats_t;

/* CAN1: PI9 RX, PB9 TX (AF9); USART1 console: PB6 TX at 115200.
 * APB1 must be 42 MHz. Bitrates 125, 250, 500 and 1000 kbit/s are supported. */
int hipnuc_board_init(uint16_t bitrate_kbps, uint8_t source_address);
int hipnuc_board_poll(hipnuc_sample_t *sample);
const hipnuc_board_stats_t *hipnuc_board_stats(void);
uint32_t hipnuc_board_millis(void);

#endif
