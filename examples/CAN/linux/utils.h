// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <linux/can.h>
#include "hipnuc_can_parser.h"

// Millisecond timestamp sourced from CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void);

// Convert a Linux CAN frame into the HiPNUC helper struct
void utils_linux_can_to_hipnuc_can(const struct can_frame *linux_frame, uint64_t hw_ts_us, hipnuc_can_frame_t *hipnuc_frame);

// Print quick setup commands when a CAN interface cannot be opened
void utils_print_can_setup_hint(const char *ifname);

#endif // UTILS_H
