/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * CAN firmware update client. The HiPNUC bootloader is driven with CANopen
 * SDO transfers on object 0x1F51; this is the only CANopen traffic the SDK
 * uses, and it is kept apart from J1939 measurement decoding.
 *
 * Portable C99: the caller supplies CAN send/wait callbacks. Copy
 * hipnuc_can_update.c, hipnuc_can_update.h and hipnuc_can_frame.h.
 *
 * Flow: enter bootloader (0x1F51:05, answered by the application) ->
 * confirm (0x1F51:06, answered by the bootloader) -> segmented download
 * (0x1F51:01) -> jump to application (0x1F51:09).
 */

#ifndef HIPNUC_CAN_UPDATE_H
#define HIPNUC_CAN_UPDATE_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_can_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_CAN_UPDATE_SDO_REQUEST_BASE   0x600U   /* + node id */
#define HIPNUC_CAN_UPDATE_SDO_RESPONSE_BASE  0x580U   /* + node id */
#define HIPNUC_CAN_UPDATE_OD_INDEX           0x1F51U
#define HIPNUC_CAN_UPDATE_SUB_DOWNLOAD       0x01U
#define HIPNUC_CAN_UPDATE_SUB_ENTER_BL       0x05U
#define HIPNUC_CAN_UPDATE_SUB_CONFIRM        0x06U
#define HIPNUC_CAN_UPDATE_SUB_GOTO_APP       0x09U

typedef enum {
    HIPNUC_CAN_UPDATE_OK          = 0,
    HIPNUC_CAN_UPDATE_ERR_PARAM   = -1,
    HIPNUC_CAN_UPDATE_ERR_SEND    = -2,
    HIPNUC_CAN_UPDATE_ERR_TIMEOUT = -3,
    HIPNUC_CAN_UPDATE_ERR_ACK     = -4,   /* unexpected reply */
    HIPNUC_CAN_UPDATE_ERR_ABORT   = -5,   /* SDO abort from the device */
    HIPNUC_CAN_UPDATE_ERR_SIZE    = -6,
    HIPNUC_CAN_UPDATE_ERR_RECEIVE = -7
} hipnuc_can_update_status_t;

typedef struct {
    /* Send one frame; return 0 on success. */
    int  (*send)(void *user, const hipnuc_can_frame_t *frame);
    /* Wait up to timeout_ms for a standard frame with identifier `id`;
     * return 0 and fill `frame` on success, positive on timeout, negative
     * on a receive/transport failure. Frames with
     * other identifiers must be skipped (they may be measurement traffic). */
    int  (*wait)(void *user, uint32_t id, hipnuc_can_frame_t *frame, uint32_t timeout_ms);
    void (*delay_ms)(void *user, uint32_t ms);                       /* optional */
    void (*progress)(void *user, uint8_t node_id, uint8_t percent);  /* optional */
    void (*log)(void *user, uint8_t node_id, const char *message);   /* optional */
    void *user;
} hipnuc_can_update_port_t;

typedef struct {
    hipnuc_can_update_port_t port;
    uint32_t expedited_timeout_ms;   /* default 100 */
    uint32_t initiate_timeout_ms;    /* default 8000: the bootloader erases flash before replying */
    uint32_t segment_timeout_ms;     /* default 4000 */
    uint32_t boot_delay_ms;          /* default 20, between enter-bootloader and confirm */
    uint32_t retry_delay_ms;         /* default 50 */
    uint8_t  handshake_retries;      /* default 5 */
} hipnuc_can_update_ctx_t;

void hipnuc_can_update_init(hipnuc_can_update_ctx_t *ctx, const hipnuc_can_update_port_t *port);

/* Expedited SDO write of a 32-bit value; waits for the 0x60 confirmation. */
int hipnuc_can_update_sdo_write(hipnuc_can_update_ctx_t *ctx, uint8_t node_id, uint16_t index,
                                uint8_t subindex, uint32_t value, uint32_t timeout_ms);

/* Enter the bootloader and confirm. Only missing handshake replies are
 * retried; transport failures and explicit rejection stop immediately. */
int hipnuc_can_update_connect(hipnuc_can_update_ctx_t *ctx, uint8_t node_id);

/* Segmented download and one application-start request. OK means all image
 * segments were acknowledged; it does not verify the running application.
 * A missing start reply is allowed because the bootloader may already reset;
 * an explicit rejection or a send failure is still returned as an error. */
int hipnuc_can_update_download(hipnuc_can_update_ctx_t *ctx, uint8_t node_id,
                               const uint8_t *image, uint32_t size);

/* connect + download. */
int hipnuc_can_update_node(hipnuc_can_update_ctx_t *ctx, uint8_t node_id,
                           const uint8_t *image, uint32_t size);

const char *hipnuc_can_update_strerror(int status);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_CAN_UPDATE_H */
