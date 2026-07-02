#include "hipnuc_can_update.h"

#include <string.h>

#define CANOPEN_SDO_TX_BASE       0x600U
#define CANOPEN_SDO_RX_BASE       0x580U
#define CO_SDO_CS_WRITE_ACK       0x60U
#define CO_SDO_CS_ABORT           0x80U
#define CO_SDO_WRITE_4BYTE        0x23U
#define CO_SDO_SEG_INIT_REQ       0x21U
#define CO_SDO_SEG_DATA0_REQ      0x00U
#define CO_SDO_SEG_DATA1_REQ      0x10U
#define CO_SDO_SEG_DATA0_RESP     0x20U
#define CO_SDO_SEG_DATA1_RESP     0x30U

#define CAN_UPDATE_DEFAULT_EXPEDITED_TIMEOUT_MS  100U
#define CAN_UPDATE_DEFAULT_SEGMENT_TIMEOUT_MS    4000U
#define CAN_UPDATE_DEFAULT_BOOT_DELAY_MS         20U
#define CAN_UPDATE_DEFAULT_RETRY_DELAY_MS        50U
#define CAN_UPDATE_DEFAULT_HANDSHAKE_RETRIES     5U

static void le16_write(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void le32_write(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
    out[2] = (uint8_t)((value >> 16) & 0xFFU);
    out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void can_update_delay(can_update_ctx_t *ctx, uint32_t delay_ms)
{
    if (ctx && ctx->port.delay_ms) {
        ctx->port.delay_ms(ctx->port.user, delay_ms);
    }
}

static void can_update_log(can_update_ctx_t *ctx, uint8_t node_id,
                           const char *msg)
{
    if (ctx && ctx->port.log) {
        ctx->port.log(ctx->port.user, node_id, msg);
    }
}

static void can_update_progress(can_update_ctx_t *ctx, uint8_t node_id,
                                uint8_t progress)
{
    if (ctx && ctx->port.progress) {
        ctx->port.progress(ctx->port.user, node_id, progress);
    }
}

static int can_update_check_ctx(const can_update_ctx_t *ctx)
{
    if (!ctx || !ctx->port.send || !ctx->port.wait) {
        return CAN_UPDATE_ERR_PARAM;
    }
    return CAN_UPDATE_OK;
}

static int can_update_send_wait(can_update_ctx_t *ctx,
                                const hipnuc_can_frame_t *tx,
                                uint32_t expected_id,
                                hipnuc_can_frame_t *rx,
                                uint32_t timeout_ms)
{
    int ret = can_update_check_ctx(ctx);
    if (ret != CAN_UPDATE_OK) {
        return ret;
    }
    if (!tx || !rx) {
        return CAN_UPDATE_ERR_PARAM;
    }
    if (ctx->port.send(ctx->port.user, tx) != 0) {
        return CAN_UPDATE_ERR_SEND;
    }
    if (ctx->port.wait(ctx->port.user, expected_id, rx, timeout_ms) != 0) {
        return CAN_UPDATE_ERR_TIMEOUT;
    }
    return CAN_UPDATE_OK;
}

static int can_update_validate_sdo_ack(const hipnuc_can_frame_t *rx,
                                       uint8_t node_id, uint16_t index,
                                       uint8_t subindex, uint8_t expected_cmd)
{
    uint32_t expected_id = CANOPEN_SDO_RX_BASE + node_id;

    if (!rx || rx->can_dlc < 4U) {
        return CAN_UPDATE_ERR_ACK;
    }
    if ((rx->can_id & HIPNUC_CAN_SFF_MASK) != expected_id) {
        return CAN_UPDATE_ERR_ACK;
    }
    if (rx->data[0] == CO_SDO_CS_ABORT) {
        return CAN_UPDATE_ERR_ABORT;
    }
    if (rx->data[0] != expected_cmd) {
        return CAN_UPDATE_ERR_ACK;
    }
    if (rx->data[1] != (uint8_t)(index & 0xFFU) ||
        rx->data[2] != (uint8_t)((index >> 8) & 0xFFU) ||
        rx->data[3] != subindex) {
        return CAN_UPDATE_ERR_ACK;
    }
    return CAN_UPDATE_OK;
}

static uint8_t can_update_last_segment_mask(uint32_t remaining)
{
    switch (remaining) {
    case 1U: return 0x0FU;
    case 2U: return 0x0DU;
    case 3U: return 0x0BU;
    case 4U: return 0x09U;
    case 5U: return 0x07U;
    case 6U: return 0x05U;
    case 7U: return 0x03U;
    default: return 0x00U;
    }
}

void can_update_init(can_update_ctx_t *ctx, const can_update_port_t *port)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (port) {
        ctx->port = *port;
    }
    ctx->expedited_timeout_ms = CAN_UPDATE_DEFAULT_EXPEDITED_TIMEOUT_MS;
    ctx->segment_timeout_ms = CAN_UPDATE_DEFAULT_SEGMENT_TIMEOUT_MS;
    ctx->boot_delay_ms = CAN_UPDATE_DEFAULT_BOOT_DELAY_MS;
    ctx->retry_delay_ms = CAN_UPDATE_DEFAULT_RETRY_DELAY_MS;
    ctx->handshake_retries = CAN_UPDATE_DEFAULT_HANDSHAKE_RETRIES;
}

void can_update_build_sdo_write(uint8_t node_id, uint16_t index,
                                uint8_t subindex, int32_t value,
                                hipnuc_can_frame_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->can_id = CANOPEN_SDO_TX_BASE + node_id;
    out->can_dlc = 8U;
    out->data[0] = CO_SDO_WRITE_4BYTE;
    le16_write(&out->data[1], index);
    out->data[3] = subindex;
    le32_write(&out->data[4], (uint32_t)value);
}

static int can_update_fsdo_write_expect(can_update_ctx_t *ctx,
                                        uint8_t node_id,
                                        uint16_t index,
                                        uint8_t subindex,
                                        int32_t value,
                                        uint32_t timeout_ms,
                                        uint8_t expected_cmd)
{
    hipnuc_can_frame_t tx;
    hipnuc_can_frame_t rx;
    int ret;

    if (node_id == 0U || node_id > 127U) {
        return CAN_UPDATE_ERR_PARAM;
    }

    can_update_build_sdo_write(node_id, index, subindex, value, &tx);
    ret = can_update_send_wait(ctx, &tx, CANOPEN_SDO_RX_BASE + node_id,
                               &rx, timeout_ms);
    if (ret != CAN_UPDATE_OK) {
        return ret;
    }
    return can_update_validate_sdo_ack(&rx, node_id, index, subindex,
                                       expected_cmd);
}

int can_update_fsdo_write(can_update_ctx_t *ctx, uint8_t node_id,
                          uint16_t index, uint8_t subindex, int32_t value,
                          uint32_t timeout_ms)
{
    return can_update_fsdo_write_expect(ctx, node_id, index, subindex, value,
                                        timeout_ms, CO_SDO_CS_WRITE_ACK);
}

int can_update_connect_node(can_update_ctx_t *ctx, uint8_t node_id)
{
    uint8_t attempt;
    uint8_t retries;

    if (can_update_check_ctx(ctx) != CAN_UPDATE_OK ||
        node_id == 0U || node_id > 127U) {
        return CAN_UPDATE_ERR_PARAM;
    }

    retries = ctx->handshake_retries ? ctx->handshake_retries : 1U;
    can_update_log(ctx, node_id, "Connecting...");

    for (attempt = 0U; attempt < retries; ++attempt) {
        int ret;

        ret = can_update_fsdo_write_expect(ctx, node_id, CANOPEN_OD_IDX_BL,
                                           CANOPEN_OD_SUBIDX_ENTER_BL, 0,
                                           ctx->expedited_timeout_ms, 0U);
        if (ret == CAN_UPDATE_OK) {
            can_update_delay(ctx, ctx->boot_delay_ms);
            ret = can_update_fsdo_write(ctx, node_id, CANOPEN_OD_IDX_BL,
                                        CANOPEN_OD_SUBIDX_CONFIRM, 0,
                                        ctx->expedited_timeout_ms);
            if (ret == CAN_UPDATE_OK) {
                can_update_log(ctx, node_id,
                               "Connected. Device entered bootloader.");
                return CAN_UPDATE_OK;
            }
        }
        can_update_delay(ctx, ctx->retry_delay_ms);
    }

    can_update_log(ctx, node_id, "Connection failed.");
    return CAN_UPDATE_ERR_TIMEOUT;
}

int can_update_sdo_write_segmented(can_update_ctx_t *ctx, uint8_t node_id,
                                   uint16_t index, uint8_t subindex,
                                   const uint8_t *data, uint32_t size,
                                   uint32_t timeout_ms)
{
    hipnuc_can_frame_t tx;
    hipnuc_can_frame_t rx;
    uint32_t offset = 0U;
    uint8_t toggle = 0U;
    uint8_t last_progress = 0U;
    int ret;

    if (node_id == 0U || node_id > 127U || !data || size == 0U) {
        return CAN_UPDATE_ERR_PARAM;
    }
    if (can_update_check_ctx(ctx) != CAN_UPDATE_OK) {
        return CAN_UPDATE_ERR_PARAM;
    }

    memset(&tx, 0, sizeof(tx));
    tx.can_id = CANOPEN_SDO_TX_BASE + node_id;
    tx.can_dlc = 8U;
    tx.data[0] = CO_SDO_SEG_INIT_REQ;
    le16_write(&tx.data[1], index);
    tx.data[3] = subindex;
    le32_write(&tx.data[4], size);

    ret = can_update_send_wait(ctx, &tx, CANOPEN_SDO_RX_BASE + node_id,
                               &rx, 8000U);
    if (ret != CAN_UPDATE_OK) {
        return ret;
    }
    ret = can_update_validate_sdo_ack(&rx, node_id, index, subindex,
                                      CO_SDO_CS_WRITE_ACK);
    if (ret != CAN_UPDATE_OK) {
        return ret;
    }

    while (offset < size) {
        uint32_t remaining = size - offset;
        uint8_t send_len = remaining > 7U ? 7U : (uint8_t)remaining;
        uint8_t expected_resp = toggle ? CO_SDO_SEG_DATA1_RESP
                                       : CO_SDO_SEG_DATA0_RESP;

        memset(&tx, 0, sizeof(tx));
        tx.can_id = CANOPEN_SDO_TX_BASE + node_id;
        tx.can_dlc = 8U;
        tx.data[0] = toggle ? CO_SDO_SEG_DATA1_REQ : CO_SDO_SEG_DATA0_REQ;
        if (remaining <= 7U) {
            tx.data[0] |= can_update_last_segment_mask(remaining);
        }
        memcpy(&tx.data[1], data + offset, send_len);

        ret = can_update_send_wait(ctx, &tx, CANOPEN_SDO_RX_BASE + node_id,
                                   &rx, timeout_ms);
        if (ret != CAN_UPDATE_OK) {
            return ret;
        }
        if (rx.can_dlc < 1U || rx.data[0] == CO_SDO_CS_ABORT) {
            return rx.can_dlc >= 1U ? CAN_UPDATE_ERR_ABORT
                                    : CAN_UPDATE_ERR_ACK;
        }
        if ((rx.can_id & HIPNUC_CAN_SFF_MASK) !=
            (CANOPEN_SDO_RX_BASE + node_id) ||
            rx.data[0] != expected_resp) {
            return CAN_UPDATE_ERR_ACK;
        }

        offset += send_len;
        toggle ^= 1U;

        if (size > 0U) {
            uint8_t progress = (uint8_t)((offset * 100U) / size);
            if (progress != last_progress) {
                can_update_progress(ctx, node_id, progress);
                last_progress = progress;
            }
        }
    }

    return CAN_UPDATE_OK;
}

int can_update_download_node(can_update_ctx_t *ctx, uint8_t node_id,
                             const uint8_t *image, uint32_t image_size)
{
    int ret;

    if (!image || image_size == 0U) {
        return CAN_UPDATE_ERR_PARAM;
    }

    can_update_log(ctx, node_id, "Downloading firmware...");
    ret = can_update_sdo_write_segmented(ctx, node_id, CANOPEN_OD_IDX_BL,
                                         CANOPEN_OD_SUBIDX_DOWNLOAD,
                                         image, image_size,
                                         ctx ? ctx->segment_timeout_ms : 0U);
    if (ret != CAN_UPDATE_OK) {
        can_update_log(ctx, node_id, "Firmware download failed.");
        return ret;
    }

    can_update_log(ctx, node_id, "Firmware download completed.");
    can_update_log(ctx, node_id, "Jumping to the application...");
    (void)can_update_fsdo_write(ctx, node_id, CANOPEN_OD_IDX_BL,
                                CANOPEN_OD_SUBIDX_GOTO_APP, 0,
                                ctx->expedited_timeout_ms);
    can_update_log(ctx, node_id,
                   "Update completed. Please power-cycle the device.");
    return CAN_UPDATE_OK;
}

int can_update_update_node(can_update_ctx_t *ctx, uint8_t node_id,
                           const uint8_t *image, uint32_t image_size)
{
    int ret = can_update_connect_node(ctx, node_id);
    if (ret != CAN_UPDATE_OK) {
        return ret;
    }
    return can_update_download_node(ctx, node_id, image, image_size);
}

int can_update_update_range(can_update_ctx_t *ctx, uint8_t start_node_id,
                            uint8_t end_node_id, const uint8_t *image,
                            uint32_t image_size, int continue_on_error)
{
    uint8_t node_id;
    int last_error = CAN_UPDATE_OK;

    if (!image || image_size == 0U ||
        start_node_id == 0U || end_node_id == 0U ||
        start_node_id > 127U || end_node_id > 127U) {
        return CAN_UPDATE_ERR_PARAM;
    }

    if (start_node_id > end_node_id) {
        uint8_t tmp = start_node_id;
        start_node_id = end_node_id;
        end_node_id = tmp;
    }

    for (node_id = start_node_id; node_id <= end_node_id; ++node_id) {
        int ret;

        can_update_log(ctx, node_id, "Starting update.");
        ret = can_update_update_node(ctx, node_id, image, image_size);
        if (ret != CAN_UPDATE_OK) {
            last_error = ret;
            can_update_log(ctx, node_id, "Update failed.");
            if (!continue_on_error) {
                return ret;
            }
        }
        can_update_delay(ctx, 100U);

        if (node_id == 127U) {
            break;
        }
    }

    return last_error;
}
