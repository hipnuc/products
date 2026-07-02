#ifndef DIALOG_CAN_UPDATE_H
#define DIALOG_CAN_UPDATE_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_can_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CANOPEN_OD_IDX_BL             0x1F51U
#define CANOPEN_OD_SUBIDX_DOWNLOAD    0x01U
#define CANOPEN_OD_SUBIDX_ENTER_BL    0x05U
#define CANOPEN_OD_SUBIDX_CONFIRM     0x06U
#define CANOPEN_OD_SUBIDX_GOTO_APP    0x09U

typedef enum {
    CAN_UPDATE_OK          = 0,
    CAN_UPDATE_ERR_PARAM   = -1,
    CAN_UPDATE_ERR_SEND    = -2,
    CAN_UPDATE_ERR_TIMEOUT = -3,
    CAN_UPDATE_ERR_ACK     = -4,
    CAN_UPDATE_ERR_ABORT   = -5,
    CAN_UPDATE_ERR_SIZE    = -6
} can_update_status_t;

typedef int  (*can_update_send_fn)(void *user, const hipnuc_can_frame_t *frame);
typedef int  (*can_update_wait_fn)(void *user, uint32_t can_id,
                                   hipnuc_can_frame_t *frame,
                                   uint32_t timeout_ms);
typedef void (*can_update_delay_fn)(void *user, uint32_t delay_ms);
typedef void (*can_update_progress_fn)(void *user, uint8_t node_id,
                                       uint8_t progress);
typedef void (*can_update_log_fn)(void *user, uint8_t node_id,
                                  const char *msg);

typedef struct {
    can_update_send_fn     send;
    can_update_wait_fn     wait;
    can_update_delay_fn    delay_ms;
    can_update_progress_fn progress;
    can_update_log_fn      log;
    void                  *user;
} can_update_port_t;

typedef struct {
    can_update_port_t port;
    uint32_t expedited_timeout_ms;
    uint32_t segment_timeout_ms;
    uint32_t boot_delay_ms;
    uint32_t retry_delay_ms;
    uint8_t  handshake_retries;
} can_update_ctx_t;

void can_update_init(can_update_ctx_t *ctx, const can_update_port_t *port);

int can_update_connect_node(can_update_ctx_t *ctx, uint8_t node_id);
int can_update_download_node(can_update_ctx_t *ctx, uint8_t node_id,
                             const uint8_t *image, uint32_t image_size);
int can_update_update_node(can_update_ctx_t *ctx, uint8_t node_id,
                           const uint8_t *image, uint32_t image_size);
int can_update_update_range(can_update_ctx_t *ctx, uint8_t start_node_id,
                            uint8_t end_node_id, const uint8_t *image,
                            uint32_t image_size, int continue_on_error);

void can_update_build_sdo_write(uint8_t node_id, uint16_t index,
                                uint8_t subindex, int32_t value,
                                hipnuc_can_frame_t *out);
int can_update_fsdo_write(can_update_ctx_t *ctx, uint8_t node_id,
                          uint16_t index, uint8_t subindex, int32_t value,
                          uint32_t timeout_ms);
int can_update_sdo_write_segmented(can_update_ctx_t *ctx, uint8_t node_id,
                                   uint16_t index, uint8_t subindex,
                                   const uint8_t *data, uint32_t size,
                                   uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
