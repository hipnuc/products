/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN firmware update client. See hipnuc_can_update.h.
 *
 * Server behaviour (HiPNUC CAN bootloader):
 *   - Expedited write (cs 0x23) to 0x1F51:05/06/09 is answered with cs 0x60
 *     and the same index/subindex. The application answers 0x1F51:05 and
 *     then reboots into the bootloader.
 *   - Initiate download (cs 0x21) to 0x1F51:01 carries the image size; the
 *     bootloader erases flash before answering 0x60.
 *   - Segments (cs 0x00/0x10 alternating, 7 data bytes) are answered with
 *     cs 0x20/0x30. The last segment sets the low nibble so the bootloader
 *     can compute the byte count, see hipnuc_can_update_last_segment_cs().
 *   - SDO abort (cs 0x80) ends the transfer.
 */

#include "hipnuc_can_update.h"

#include <string.h>

#define SDO_CS_EXPEDITED_WRITE   0x23U
#define SDO_CS_INITIATE_DOWNLOAD 0x21U
#define SDO_CS_WRITE_ACK         0x60U
#define SDO_CS_ABORT             0x80U
#define SDO_CS_SEGMENT_REQ0      0x00U
#define SDO_CS_SEGMENT_REQ1      0x10U
#define SDO_CS_SEGMENT_RESP0     0x20U
#define SDO_CS_SEGMENT_RESP1     0x30U

#define SDO_SEGMENT_DATA_BYTES   7U
#define SDO_FRAME_LEN            8U

#define CAN_UPDATE_DEFAULT_EXPEDITED_TIMEOUT_MS  100U
#define CAN_UPDATE_DEFAULT_INITIATE_TIMEOUT_MS   8000U
#define CAN_UPDATE_DEFAULT_SEGMENT_TIMEOUT_MS    4000U
#define CAN_UPDATE_DEFAULT_BOOT_DELAY_MS         20U
#define CAN_UPDATE_DEFAULT_RETRY_DELAY_MS        50U
#define CAN_UPDATE_DEFAULT_HANDSHAKE_RETRIES     5U

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void cu_log(const hipnuc_can_update_ctx_t *ctx, uint8_t node_id, const char *message)
{
    if (ctx->port.log) {
        ctx->port.log(ctx->port.user, node_id, message);
    }
}

static void cu_delay(const hipnuc_can_update_ctx_t *ctx, uint32_t ms)
{
    if (ctx->port.delay_ms) {
        ctx->port.delay_ms(ctx->port.user, ms);
    }
}

static void cu_progress(const hipnuc_can_update_ctx_t *ctx, uint8_t node_id, uint8_t percent)
{
    if (ctx->port.progress) {
        ctx->port.progress(ctx->port.user, node_id, percent);
    }
}

static int cu_check(const hipnuc_can_update_ctx_t *ctx, uint8_t node_id)
{
    if (!ctx || !ctx->port.send || !ctx->port.wait) {
        return HIPNUC_CAN_UPDATE_ERR_PARAM;
    }
    if (node_id == 0U || node_id > 127U) {
        return HIPNUC_CAN_UPDATE_ERR_PARAM;
    }
    return HIPNUC_CAN_UPDATE_OK;
}

/* Prepare a standard 8-byte SDO request frame addressed to `node_id`. */
static void cu_prepare_request(hipnuc_can_frame_t *frame, uint8_t node_id, uint8_t cs)
{
    memset(frame, 0, sizeof(*frame));
    frame->id = HIPNUC_CAN_UPDATE_SDO_REQUEST_BASE + node_id;
    frame->len = SDO_FRAME_LEN;
    frame->data[0] = cs;
}

/* Send `tx` and wait for the SDO response of `node_id`. */
static int cu_exchange(const hipnuc_can_update_ctx_t *ctx, uint8_t node_id,
                       const hipnuc_can_frame_t *tx, hipnuc_can_frame_t *rx, uint32_t timeout_ms)
{
    uint32_t response_id = HIPNUC_CAN_UPDATE_SDO_RESPONSE_BASE + node_id;
    int result;

    if (ctx->port.send(ctx->port.user, tx) != 0) {
        cu_log(ctx, node_id, "CAN send failed");
        return HIPNUC_CAN_UPDATE_ERR_SEND;
    }
    memset(rx, 0, sizeof(*rx));
    result = ctx->port.wait(ctx->port.user, response_id, rx, timeout_ms);
    if (result > 0) return HIPNUC_CAN_UPDATE_ERR_TIMEOUT;
    if (result < 0) return HIPNUC_CAN_UPDATE_ERR_RECEIVE;
    if (rx->id != response_id || rx->is_extended || rx->is_remote || rx->is_error || rx->len < 1U) {
        return HIPNUC_CAN_UPDATE_ERR_ACK;
    }
    return HIPNUC_CAN_UPDATE_OK;
}

/* Check a reply that must echo index/subindex with command specifier `cs`. */
static int cu_check_indexed_reply(const hipnuc_can_frame_t *rx, uint16_t index, uint8_t subindex, uint8_t cs)
{
    if (rx->data[0] == SDO_CS_ABORT) {
        return HIPNUC_CAN_UPDATE_ERR_ABORT;
    }
    if (rx->len < 4U || rx->data[0] != cs) {
        return HIPNUC_CAN_UPDATE_ERR_ACK;
    }
    if (rx->data[1] != (uint8_t)(index & 0xFFU) ||
        rx->data[2] != (uint8_t)((index >> 8) & 0xFFU) ||
        rx->data[3] != subindex) {
        return HIPNUC_CAN_UPDATE_ERR_ACK;
    }
    return HIPNUC_CAN_UPDATE_OK;
}

/*
 * Command specifier of the last segment carrying `n` data bytes (1..7).
 *
 * The HiPNUC bootloader derives the byte count as
 *   n = ((15 - (cs & 0x0F)) / 2) + 1
 * so the low nibble is 17 - 2 * n: 7 bytes -> 0x03, 6 -> 0x05, ... 1 -> 0x0F.
 * This is the bootloader's convention (it differs from the CiA 301 encoding
 * where 7 bytes would give 0x01); keep it as is.
 */
static uint8_t cu_last_segment_cs(uint8_t toggle_cs, uint32_t n)
{
    return (uint8_t)(toggle_cs | (uint8_t)(17U - 2U * n));
}

void hipnuc_can_update_init(hipnuc_can_update_ctx_t *ctx, const hipnuc_can_update_port_t *port)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (port) {
        ctx->port = *port;
    }
    ctx->expedited_timeout_ms = CAN_UPDATE_DEFAULT_EXPEDITED_TIMEOUT_MS;
    ctx->initiate_timeout_ms = CAN_UPDATE_DEFAULT_INITIATE_TIMEOUT_MS;
    ctx->segment_timeout_ms = CAN_UPDATE_DEFAULT_SEGMENT_TIMEOUT_MS;
    ctx->boot_delay_ms = CAN_UPDATE_DEFAULT_BOOT_DELAY_MS;
    ctx->retry_delay_ms = CAN_UPDATE_DEFAULT_RETRY_DELAY_MS;
    ctx->handshake_retries = CAN_UPDATE_DEFAULT_HANDSHAKE_RETRIES;
}

int hipnuc_can_update_sdo_write(hipnuc_can_update_ctx_t *ctx, uint8_t node_id, uint16_t index,
                                uint8_t subindex, uint32_t value, uint32_t timeout_ms)
{
    hipnuc_can_frame_t tx;
    hipnuc_can_frame_t rx;
    int ret;

    ret = cu_check(ctx, node_id);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }

    cu_prepare_request(&tx, node_id, SDO_CS_EXPEDITED_WRITE);
    wr_u16(&tx.data[1], index);
    tx.data[3] = subindex;
    wr_u32(&tx.data[4], value);

    ret = cu_exchange(ctx, node_id, &tx, &rx, timeout_ms);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }
    return cu_check_indexed_reply(&rx, index, subindex, SDO_CS_WRITE_ACK);
}

int hipnuc_can_update_connect(hipnuc_can_update_ctx_t *ctx, uint8_t node_id)
{
    uint8_t attempts;
    uint8_t i;
    int ret;

    ret = cu_check(ctx, node_id);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }

    attempts = ctx->handshake_retries ? ctx->handshake_retries : 1U;
    cu_log(ctx, node_id, "connecting");

    for (i = 0; i < attempts; ++i) {
        /* The application answers this write with 0x60, then reboots. */
        ret = hipnuc_can_update_sdo_write(ctx, node_id, HIPNUC_CAN_UPDATE_OD_INDEX,
                                          HIPNUC_CAN_UPDATE_SUB_ENTER_BL, 0, ctx->expedited_timeout_ms);
        if (ret == HIPNUC_CAN_UPDATE_OK) {
            cu_delay(ctx, ctx->boot_delay_ms);
            ret = hipnuc_can_update_sdo_write(ctx, node_id, HIPNUC_CAN_UPDATE_OD_INDEX,
                                              HIPNUC_CAN_UPDATE_SUB_CONFIRM, 0, ctx->expedited_timeout_ms);
            if (ret == HIPNUC_CAN_UPDATE_OK) {
                cu_log(ctx, node_id, "device entered bootloader");
                return HIPNUC_CAN_UPDATE_OK;
            }
        }
        if (ret != HIPNUC_CAN_UPDATE_ERR_TIMEOUT) return ret;
        if (i + 1U < attempts) {
            cu_delay(ctx, ctx->retry_delay_ms);
        }
    }

    cu_log(ctx, node_id, "no bootloader handshake");
    return HIPNUC_CAN_UPDATE_ERR_TIMEOUT;
}

int hipnuc_can_update_download(hipnuc_can_update_ctx_t *ctx, uint8_t node_id,
                               const uint8_t *image, uint32_t size)
{
    hipnuc_can_frame_t tx;
    hipnuc_can_frame_t rx;
    uint32_t offset = 0;
    uint8_t toggle = 0;
    uint8_t last_percent = 0;
    int ret;

    ret = cu_check(ctx, node_id);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }
    if (!image || size == 0U) {
        return HIPNUC_CAN_UPDATE_ERR_PARAM;
    }

    /* Initiate: the bootloader erases `size` bytes before answering. */
    cu_prepare_request(&tx, node_id, SDO_CS_INITIATE_DOWNLOAD);
    wr_u16(&tx.data[1], HIPNUC_CAN_UPDATE_OD_INDEX);
    tx.data[3] = HIPNUC_CAN_UPDATE_SUB_DOWNLOAD;
    wr_u32(&tx.data[4], size);

    cu_log(ctx, node_id, "erasing flash");
    ret = cu_exchange(ctx, node_id, &tx, &rx, ctx->initiate_timeout_ms);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        cu_log(ctx, node_id, "download initiate failed");
        return ret;
    }
    ret = cu_check_indexed_reply(&rx, HIPNUC_CAN_UPDATE_OD_INDEX, HIPNUC_CAN_UPDATE_SUB_DOWNLOAD, SDO_CS_WRITE_ACK);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        cu_log(ctx, node_id, "download initiate rejected");
        return ret;
    }

    cu_log(ctx, node_id, "downloading firmware");
    while (offset < size) {
        uint32_t remaining = size - offset;
        uint32_t n = remaining > SDO_SEGMENT_DATA_BYTES ? SDO_SEGMENT_DATA_BYTES : remaining;
        uint8_t req_cs = toggle ? SDO_CS_SEGMENT_REQ1 : SDO_CS_SEGMENT_REQ0;
        uint8_t resp_cs = toggle ? SDO_CS_SEGMENT_RESP1 : SDO_CS_SEGMENT_RESP0;
        uint8_t percent;

        if (remaining <= SDO_SEGMENT_DATA_BYTES) {
            req_cs = cu_last_segment_cs(req_cs, n);
        }
        cu_prepare_request(&tx, node_id, req_cs);
        memcpy(&tx.data[1], image + offset, n);

        ret = cu_exchange(ctx, node_id, &tx, &rx, ctx->segment_timeout_ms);
        if (ret != HIPNUC_CAN_UPDATE_OK) {
            cu_log(ctx, node_id, "segment not acknowledged");
            return ret;
        }
        if (rx.data[0] == SDO_CS_ABORT) {
            cu_log(ctx, node_id, "device aborted the transfer");
            return HIPNUC_CAN_UPDATE_ERR_ABORT;
        }
        if (rx.data[0] != resp_cs) {
            cu_log(ctx, node_id, "unexpected segment reply");
            return HIPNUC_CAN_UPDATE_ERR_ACK;
        }

        offset += n;
        toggle ^= 1U;

        percent = (uint8_t)(((uint64_t)offset * 100U) / size);
        if (percent != last_percent) {
            last_percent = percent;
            cu_progress(ctx, node_id, percent);
        }
    }
    cu_log(ctx, node_id, "download complete");

    /* Some bootloaders reset before replying. Only that timeout is allowed;
     * a send failure, explicit abort or malformed ACK remains a failure. */
    ret = hipnuc_can_update_sdo_write(ctx, node_id, HIPNUC_CAN_UPDATE_OD_INDEX,
                                      HIPNUC_CAN_UPDATE_SUB_GOTO_APP, 0, ctx->expedited_timeout_ms);
    if (ret == HIPNUC_CAN_UPDATE_OK) {
        cu_log(ctx, node_id, "application start request acknowledged; startup not verified");
    } else if (ret == HIPNUC_CAN_UPDATE_ERR_TIMEOUT) {
        cu_log(ctx, node_id, "image transfer confirmed; application start request had no reply");
    } else {
        cu_log(ctx, node_id, "image transfer confirmed; application start request failed");
        return ret;
    }
    return HIPNUC_CAN_UPDATE_OK;
}

int hipnuc_can_update_node(hipnuc_can_update_ctx_t *ctx, uint8_t node_id,
                           const uint8_t *image, uint32_t size)
{
    int ret;

    ret = cu_check(ctx, node_id);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }
    if (!image || size == 0U) {
        return HIPNUC_CAN_UPDATE_ERR_PARAM;
    }
    ret = hipnuc_can_update_connect(ctx, node_id);
    if (ret != HIPNUC_CAN_UPDATE_OK) {
        return ret;
    }
    return hipnuc_can_update_download(ctx, node_id, image, size);
}

const char *hipnuc_can_update_strerror(int status)
{
    switch (status) {
    case HIPNUC_CAN_UPDATE_OK:          return "ok";
    case HIPNUC_CAN_UPDATE_ERR_PARAM:   return "invalid argument";
    case HIPNUC_CAN_UPDATE_ERR_SEND:    return "CAN send failed";
    case HIPNUC_CAN_UPDATE_ERR_TIMEOUT: return "no reply from device";
    case HIPNUC_CAN_UPDATE_ERR_ACK:     return "unexpected SDO reply";
    case HIPNUC_CAN_UPDATE_ERR_ABORT:   return "SDO abort from device";
    case HIPNUC_CAN_UPDATE_ERR_SIZE:    return "image too large";
    case HIPNUC_CAN_UPDATE_ERR_RECEIVE: return "CAN receive failed";
    default:                            return "unknown error";
    }
}
