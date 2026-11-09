/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC I2C host; see hipnuc_i2c.h.
 */

#include "hipnuc_i2c.h"
#include "hipnuc_decoder.h"

#include <string.h>

#define REG_SAMPLE 0x1000U   /* a pointer write here asks for the newest sample as a frame */
#define HEADER 6U            /* 5A A5, payload length (LE), CRC (LE) */
#define MAX_FRAME 254U       /* the longest frame this SDK decodes (HI83, every supported field) */
#define START_MS 150U        /* the module does not answer this long after power-up or reset */

/* A NACKed transfer is retried every 1 ms until the module answers. */
static int transfer(hipnuc_i2c_t *dev, int is_read, uint8_t *data, size_t len)
{
    uint32_t waited;
    for (waited = 0;; waited++) {
        int rc = is_read ? dev->read(dev->ctx, dev->address, data, len)
                         : dev->write(dev->ctx, dev->address, data, len);
        if (rc == 0) return HIPNUC_I2C_OK;
        if (rc < 0) return HIPNUC_I2C_ERR_BUS;
        if (waited >= HIPNUC_I2C_RETRY_MS) return HIPNUC_I2C_ERR_TIMEOUT;
        dev->delay_ms(dev->ctx, 1);
    }
}

static int set_pointer(hipnuc_i2c_t *dev, uint16_t reg)
{
    uint8_t cmd[2];
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    return transfer(dev, 0, cmd, sizeof(cmd));
}

static int bus_ok(const hipnuc_i2c_t *dev)
{
    return dev && dev->write && dev->read && dev->delay_ms;
}

int hipnuc_i2c_init(hipnuc_i2c_t *dev)
{
    uint16_t id;
    int rc;
    if (!bus_ok(dev)) return HIPNUC_I2C_ERR_ARG;
    dev->have_last = 0;
    dev->delay_ms(dev->ctx, START_MS);
    rc = hipnuc_i2c_read_regs(dev, HIPNUC_I2C_REG_MODBUS_ID, &id, 1);
    if (rc != HIPNUC_I2C_OK) return rc;
    return id == dev->address ? HIPNUC_I2C_OK : HIPNUC_I2C_ERR_ID;
}

int hipnuc_i2c_read_regs(hipnuc_i2c_t *dev, uint16_t reg, uint16_t *value, uint8_t count)
{
    uint8_t cmd[3];
    uint8_t data[2 * HIPNUC_I2C_MAX_REGS];
    uint8_t i;
    int rc;

    if (!bus_ok(dev) || !value || count == 0 || count > HIPNUC_I2C_MAX_REGS) return HIPNUC_I2C_ERR_ARG;
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    cmd[2] = count;
    rc = transfer(dev, 0, cmd, sizeof(cmd));
    if (rc == HIPNUC_I2C_OK) rc = transfer(dev, 1, data, 2U * count);   /* NACKed until the module has them */
    if (rc != HIPNUC_I2C_OK) return rc;
    for (i = 0; i < count; i++) value[i] = (uint16_t)(((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
    return HIPNUC_I2C_OK;
}

int hipnuc_i2c_write_regs(hipnuc_i2c_t *dev, uint16_t reg, const uint16_t *value, uint8_t count)
{
    uint8_t cmd[2 + 2 * HIPNUC_I2C_MAX_REGS];
    uint8_t i;

    if (!bus_ok(dev) || !value || count == 0 || count > HIPNUC_I2C_MAX_REGS) return HIPNUC_I2C_ERR_ARG;
    cmd[0] = (uint8_t)(reg >> 8);
    cmd[1] = (uint8_t)reg;
    for (i = 0; i < count; i++) {
        cmd[2 + 2 * i] = (uint8_t)(value[i] >> 8);
        cmd[3 + 2 * i] = (uint8_t)value[i];
    }
    return transfer(dev, 0, cmd, 2U + 2U * count);
}

int hipnuc_i2c_read_data(hipnuc_i2c_t *dev, hipnuc_data_t *out)
{
    uint8_t frame[MAX_FRAME];
    size_t total;
    uint16_t crc;
    int rc;

    if (!bus_ok(dev)) return HIPNUC_I2C_ERR_ARG;
    /* The header is read once the module has encoded the sample, the rest from word 3 on. */
    rc = set_pointer(dev, REG_SAMPLE);
    if (rc == HIPNUC_I2C_OK) rc = transfer(dev, 1, frame, HEADER);
    if (rc != HIPNUC_I2C_OK) return rc;
    if (frame[0] != 0x5AU || frame[1] != 0xA5U) return 0;   /* no sample yet */
    total = HEADER + ((size_t)frame[2] | ((size_t)frame[3] << 8));
    if (total == HEADER || total > MAX_FRAME) return HIPNUC_I2C_ERR_FRAME;
    crc = (uint16_t)(frame[4] | ((uint16_t)frame[5] << 8));
    /* The same sample again has the same checksum and length. */
    if (dev->have_last && crc == dev->last_crc && total == dev->last_len) return 0;
    rc = set_pointer(dev, REG_SAMPLE + HEADER / 2U);
    if (rc == HIPNUC_I2C_OK) rc = transfer(dev, 1, frame + HEADER, total - HEADER);
    if (rc != HIPNUC_I2C_OK) return rc;
    if (hipnuc_frame_decode(frame, total, out) != 1) return HIPNUC_I2C_ERR_FRAME;
    dev->have_last = 1;
    dev->last_crc = crc;
    dev->last_len = (uint16_t)total;
    return 1;
}
