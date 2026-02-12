// utils.c
#include "utils.h"
#include <time.h>
#include <string.h>
#include <unistd.h>

// Millisecond timestamp based on CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void utils_delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}

void utils_hipnuc_can_to_linux_can(const hipnuc_can_frame_t *hipnuc_frame, struct can_frame *linux_frame)
{
    linux_frame->can_id = hipnuc_frame->can_id;
    linux_frame->can_dlc = hipnuc_frame->can_dlc;
    memcpy(linux_frame->data, hipnuc_frame->data, 8);
}

