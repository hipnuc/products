// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <linux/can.h>
#include "hipnuc_can_common.h"

// Millisecond timestamp sourced from CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void);

// Convert a Linux CAN frame into the HiPNUC helper struct
void utils_linux_can_to_hipnuc_can(const struct can_frame *linux_frame, uint64_t hw_ts_us, hipnuc_can_frame_t *hipnuc_frame);

// Convert a HiPNUC helper struct into a Linux CAN frame
void utils_hipnuc_can_to_linux_can(const hipnuc_can_frame_t *hipnuc_frame, struct can_frame *linux_frame);


#endif // UTILS_H
