#ifndef J1939_REG_API_H
#define J1939_REG_API_H

#include <stdint.h>

typedef struct {
    uint32_t value;
    uint8_t status;
} j1939_reg_result_t;

int j1939_reg_read(int fd, uint8_t da, uint8_t sa, uint16_t addr, int timeout_ms, j1939_reg_result_t *out);
int j1939_reg_write(int fd, uint8_t da, uint8_t sa, uint16_t addr, uint32_t val, int timeout_ms, j1939_reg_result_t *out);

#endif
