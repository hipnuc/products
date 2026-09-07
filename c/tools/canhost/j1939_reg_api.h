/* Register read/write over J1939 configuration frames (PGN 0xEF00). */
#ifndef J1939_REG_API_H
#define J1939_REG_API_H

#include <stdint.h>

typedef struct {
    uint32_t value;
    uint8_t status;     /* 0 = ok, otherwise device error code */
} j1939_reg_result_t;

/* Send a request to node `da` from host address `sa` and wait up to
 * timeout_ms for the matching reply from `da`. Returns 1 when a reply was
 * received (out filled), 0 on timeout, -1 on a socket error. */
int j1939_reg_read(int fd, uint8_t da, uint8_t sa, uint16_t addr, int timeout_ms, j1939_reg_result_t *out);
int j1939_reg_write(int fd, uint8_t da, uint8_t sa, uint16_t addr, uint32_t val, int timeout_ms, j1939_reg_result_t *out);

#endif /* J1939_REG_API_H */
