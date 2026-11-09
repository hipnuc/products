/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC I2C host (the -I2C models): register access and the newest sample,
 * from an MCU acting as I2C master.
 *
 * Portable C99: no heap, no stdio, no global state. Copy hipnuc_i2c.c/.h with
 * hipnuc_decoder.c/.h (it decodes the frames), hipnuc_data.h/.c and
 * hipnuc_internal.h, and supply the bus in hipnuc_i2c_t.
 *
 * Use separate transfers, each with its own START and STOP (no repeated
 * START). The module answers NACK while it works on a request or saves to
 * flash; every call retries a NACKed transfer every 1 ms for up to
 * HIPNUC_I2C_RETRY_MS. It may also hold SCL low for about 20 ms while flash is
 * busy, so the bus must allow clock stretching. Register words are big-endian on the bus.
 */

#ifndef HIPNUC_I2C_H
#define HIPNUC_I2C_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_I2C_ADDRESS_DEFAULT 0x50U   /* 7-bit; the MODBUS_ID register */
#define HIPNUC_I2C_REG_MODBUS_ID   0x0005U
#define HIPNUC_I2C_REG_HI83_MAP    0x00B8U   /* two words, high word first: the fields of the sample */
#define HIPNUC_I2C_MAX_REGS        32U       /* registers per read or write */
#ifndef HIPNUC_I2C_RETRY_MS
#define HIPNUC_I2C_RETRY_MS        1000U     /* longest wait for a NACKing module */
#endif

/* Return codes */
#define HIPNUC_I2C_OK          0
#define HIPNUC_I2C_ERR_ARG     (-1)
#define HIPNUC_I2C_ERR_BUS     (-2)          /* the bus hook reported an error */
#define HIPNUC_I2C_ERR_TIMEOUT (-3)          /* NACK for longer than HIPNUC_I2C_RETRY_MS */
#define HIPNUC_I2C_ERR_ID      (-4)          /* MODBUS_ID is not the bus address */
#define HIPNUC_I2C_ERR_FRAME   (-5)          /* the sample frame is too long, damaged or not decodable */

typedef struct {
    /* One whole transfer: START, address, data, STOP. Return 0 when done, 1
     * when the address or a byte was NACKed, below 0 on a bus error. */
    int (*write)(void *ctx, uint8_t address, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t address, uint8_t *data, size_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
    void *ctx;
    uint8_t address;                         /* 7-bit */

    /* Private: the last sample returned by hipnuc_i2c_read_data() */
    uint8_t have_last;
    uint16_t last_crc, last_len;
} hipnuc_i2c_t;

/* Waits out the module's start-up window and checks that MODBUS_ID reads back
 * as the bus address. Call it again after writing a reset (REBOOT, FRESET) to
 * the CTRL register: the module drives these pins as CAN for about 100 ms
 * while it restarts. */
int hipnuc_i2c_init(hipnuc_i2c_t *dev);

/* Registers, 1..HIPNUC_I2C_MAX_REGS words at reg (below 0x1000). A write the
 * module refuses is not reported: read it back to confirm. */
int hipnuc_i2c_read_regs(hipnuc_i2c_t *dev, uint16_t reg, uint16_t *value, uint8_t count);
int hipnuc_i2c_write_regs(hipnuc_i2c_t *dev, uint16_t reg, const uint16_t *value, uint8_t count);

/* Reads and decodes the newest sample (its fields follow HI83_MAP). Returns
 * 1 for a new sample, 0 when the module has none yet or it is the sample
 * already returned (reading faster than the module samples), or an error
 * code. out is written only on 1. */
int hipnuc_i2c_read_data(hipnuc_i2c_t *dev, hipnuc_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_I2C_H */
