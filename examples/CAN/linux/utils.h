// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <linux/can.h>
#include "hipnuc_can_common.h"

// Millisecond timestamp sourced from CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void);

// Sleep for specified milliseconds
void utils_delay_ms(uint32_t ms);

// Convert a HiPNUC helper struct into a Linux CAN frame
void utils_hipnuc_can_to_linux_can(const hipnuc_can_frame_t *hipnuc_frame, struct can_frame *linux_frame);


#endif // UTILS_H
