// utils.c
#include "utils.h"
#include <time.h>
#include <string.h>

// Millisecond timestamp based on CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Convert a Linux CAN frame into the HiPNUC CAN helper struct
void utils_linux_can_to_hipnuc_can(const struct can_frame *linux_frame, uint64_t hw_ts_us, hipnuc_can_frame_t *hipnuc_frame)
{
    hipnuc_frame->can_id = linux_frame->can_id;
    hipnuc_frame->can_dlc = linux_frame->can_dlc;
    memcpy(hipnuc_frame->data, linux_frame->data, 8);
    hipnuc_frame->hw_ts_us = hw_ts_us;
}

void utils_hipnuc_can_to_linux_can(const hipnuc_can_frame_t *hipnuc_frame, struct can_frame *linux_frame)
{
    linux_frame->can_id = hipnuc_frame->can_id;
    linux_frame->can_dlc = hipnuc_frame->can_dlc;
    memcpy(linux_frame->data, hipnuc_frame->data, 8);
}

