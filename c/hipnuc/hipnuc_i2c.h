/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC I2C host link (HI05 I2C models): register access and the newest
 * HI83 frame, from an MCU acting as I2C master.
 *
 * Portable C99, no dynamic memory, no stdio, no global state. Copy
 * hipnuc_i2c.c/.h into your project and supply the bus in hipnuc_i2c_t. To
 * decode the frames, also copy hipnuc_dec.c/.h and hipnuc_sample.c/.h and feed
 * each frame to hipnuc_input_buffer().
 *
 * The module answers NACK while it runs a request (a register access, or
 * encoding the frame); every call retries a NACKed transfer every 1 ms for up
 * to 1 s. While it writes flash it holds
 * SCL low instead, for up to about 20 ms, so the bus must allow clock
 * stretching. Register words are big-endian on the bus; the frames inside are
 * the same little-endian HiPNUC frames as on the UART.
 */

#ifndef HIPNUC_I2C_H
#define HIPNUC_I2C_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_I2C_ADDRESS_DEFAULT 0x50U      /* 7-bit; the MODBUS_ID register */
#define HIPNUC_I2C_REG_MODBUS_ID   0x0005U
#define HIPNUC_I2C_REG_HI83        0x1000U    /* the newest sample as an HI83 frame */
#define HIPNUC_I2C_MAX_REGS        32U        /* registers per read or write */
#define HIPNUC_I2C_RETRY_MS        1000U

/* Return codes */
#define HIPNUC_I2C_OK              0
#define HIPNUC_I2C_ERR_ARG         (-1)
#define HIPNUC_I2C_ERR_BUS         (-2)       /* the bus hook reported an error */
#define HIPNUC_I2C_ERR_TIMEOUT     (-3)       /* NACK for longer than HIPNUC_I2C_RETRY_MS */
#define HIPNUC_I2C_ERR_ID          (-4)       /* MODBUS_ID is not the bus address */
#define HIPNUC_I2C_ERR_SPACE       (-5)       /* the frame does not fit the buffer */

typedef struct {
    /* One whole transfer with START, address, data and STOP. Return 0 when
     * done, 1 when the address or a byte was NACKed, below 0 on a bus error. */
    int (*write)(void *ctx, uint8_t address, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t address, uint8_t *data, size_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
    void *ctx;
    uint8_t address;                          /* 7-bit */
} hipnuc_i2c_t;

/* Waits out the module's reset window and checks that MODBUS_ID reads back as
 * the bus address. Call it again after writing a reset (REBOOT, FRESET) to the
 * CTRL register, before any other transfer: the module drives these pins as
 * CAN for about 100 ms while it restarts. */
int hipnuc_i2c_init(hipnuc_i2c_t *dev);

/* HIREG registers, 1..HIPNUC_I2C_MAX_REGS words at reg (below 0x1000). A
 * write that the module refuses is not reported: read it back to confirm. */
int hipnuc_i2c_read_regs(hipnuc_i2c_t *dev, uint16_t reg, uint16_t *value, uint8_t count);
int hipnuc_i2c_write_regs(hipnuc_i2c_t *dev, uint16_t reg, const uint16_t *value, uint8_t count);

/* Reads the newest sample as an HI83 frame into buf; the fields follow
 * HI83_MAP. *len is 0 before the module's first sample. Reading faster than
 * the module samples returns the same sample again (same system time). */
int hipnuc_i2c_read_frame(hipnuc_i2c_t *dev, uint8_t *buf, size_t cap, size_t *len);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_I2C_H */
