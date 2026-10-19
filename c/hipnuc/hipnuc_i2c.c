/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC I2C host link; see hipnuc_i2c.h.
 */

#include "hipnuc_i2c.h"

#define FRAME_HEADER 6U          /* 5A A5, payload length (LE), CRC */
#define RESET_WINDOW_MS 150U

/* A NACKed transfer is retried every 1 ms until the module answers. */
static int with_retry(hipnuc_i2c_t *dev, int is_read, uint8_t *data, size_t len)
{
    uint32_t waited;

    for (waited = 0; ; waited++) {
        int rc = is_read ? dev->read(dev->ctx, dev->address, data, len)
                         : dev->write(dev->ctx, dev->address, data, len);
        if (rc == 0)
            return HIPNUC_I2C_OK;
        if (rc < 0)
            return HIPNUC_I2C_ERR_BUS;
        if (waited >= HIPNUC_I2C_RETRY_MS)
            return HIPNUC_I2C_ERR_TIMEOUT;
        dev->delay_ms(dev->ctx, 1);
    }
}

static int set_pointer(hipnuc_i2c_t *dev, uint16_t reg)
{
    uint8_t cmd[2];
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    return with_retry(dev, 0, cmd, sizeof(cmd));
}

int hipnuc_i2c_init(hipnuc_i2c_t *dev)
{
    uint16_t id;
    int rc;

    dev->delay_ms(dev->ctx, RESET_WINDOW_MS);
    rc = hipnuc_i2c_read_regs(dev, HIPNUC_I2C_REG_MODBUS_ID, &id, 1);
    if (rc != HIPNUC_I2C_OK)
        return rc;
    return id == dev->address ? HIPNUC_I2C_OK : HIPNUC_I2C_ERR_ID;
}

int hipnuc_i2c_read_regs(hipnuc_i2c_t *dev, uint16_t reg, uint16_t *value, uint8_t count)
{
    uint8_t cmd[3];
    uint8_t data[2 * HIPNUC_I2C_MAX_REGS];
    uint8_t i;
    int rc;

    if (count == 0 || count > HIPNUC_I2C_MAX_REGS)
        return HIPNUC_I2C_ERR_ARG;
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    cmd[2] = count;
    rc = with_retry(dev, 0, cmd, sizeof(cmd));
    if (rc == HIPNUC_I2C_OK)
        rc = with_retry(dev, 1, data, 2U * count);  /* NACKed until the module has read them */
    if (rc != HIPNUC_I2C_OK)
        return rc;
    for (i = 0; i < count; i++)
        value[i] = (uint16_t)(data[2 * i] << 8 | data[2 * i + 1]);
    return HIPNUC_I2C_OK;
}

int hipnuc_i2c_write_regs(hipnuc_i2c_t *dev, uint16_t reg, const uint16_t *value, uint8_t count)
{
    uint8_t cmd[2 + 2 * HIPNUC_I2C_MAX_REGS];
    uint8_t i;

    if (count == 0 || count > HIPNUC_I2C_MAX_REGS)
        return HIPNUC_I2C_ERR_ARG;
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    for (i = 0; i < count; i++) {
        cmd[2 + 2 * i] = (uint8_t)(value[i] >> 8);
        cmd[3 + 2 * i] = (uint8_t)value[i];
    }
    return with_retry(dev, 0, cmd, 2U + 2U * count);
}

int hipnuc_i2c_read_frame(hipnuc_i2c_t *dev, uint8_t *buf, size_t cap, size_t *len)
{
    size_t frame_len;
    int rc;

    *len = 0;
    if (cap < FRAME_HEADER)
        return HIPNUC_I2C_ERR_ARG;
    /* The pointer at HIPNUC_I2C_REG_HI83 asks for the newest sample; the
     * header is read once it is encoded, the rest from word 3 on. */
    rc = set_pointer(dev, HIPNUC_I2C_REG_HI83);
    if (rc == HIPNUC_I2C_OK)
        rc = with_retry(dev, 1, buf, FRAME_HEADER);
    if (rc != HIPNUC_I2C_OK)
        return rc;
    if (buf[0] != 0x5AU || buf[1] != 0xA5U)
        return HIPNUC_I2C_OK;                   /* no sample yet */
    frame_len = FRAME_HEADER + ((size_t)buf[2] | (size_t)buf[3] << 8);
    if (frame_len > cap)
        return HIPNUC_I2C_ERR_SPACE;
    rc = set_pointer(dev, HIPNUC_I2C_REG_HI83 + FRAME_HEADER / 2U);
    if (rc == HIPNUC_I2C_OK)
        rc = with_retry(dev, 1, buf + FRAME_HEADER, frame_len - FRAME_HEADER);
    if (rc != HIPNUC_I2C_OK)
        return rc;
    *len = frame_len;
    return HIPNUC_I2C_OK;
}
