#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

/* Milliseconds from CLOCK_MONOTONIC; does not wrap. */
uint64_t utils_now_ms(void);

void utils_delay_ms(uint32_t ms);

#endif /* UTILS_H */
