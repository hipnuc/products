/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC CAN messages: J1939 data messages and CANFD83 into hipnuc_data_t,
 * and the register read/write frames (PGN 0xEF00) for configuration.
 *
 * Portable C99: no heap, no stdio, no global state. Copy hipnuc_can.c/.h with
 * hipnuc_data.h/.c and hipnuc_internal.h, convert each frame your CAN driver
 * receives into hipnuc_can_frame_t and pass it to hipnuc_can_decode().
 *
 * A J1939 message carries one quantity (acceleration, angular rate, ...);
 * use hipnuc_data_merge() to collect the messages of one epoch.
 */

#ifndef HIPNUC_CAN_H
#define HIPNUC_CAN_H

#include <stdint.h>

#include "hipnuc_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_CAN_NODE_DEFAULT 0x08U     /* default source address of the device */
#define HIPNUC_CAN_HOST_DEFAULT 0x55U     /* host address used in the manual; any free address works */

/* Register: write a PGN number and the device sends that message once */
#define HIPNUC_CAN_REG_TRIGGER  0x0096U

/* hipnuc_can_reply() results */
#define HIPNUC_CAN_READ  3                /* reply to a read: *value is the register */
#define HIPNUC_CAN_WRITE 6                /* reply to a write: the device received it (read back to confirm) */

/* A classic or FD data frame. Pass data frames only; drop the error frames
 * your driver reports. */
typedef struct {
    uint32_t id;          /* 11-bit or 29-bit identifier, without flag bits */
    uint8_t  extended;    /* 1: 29-bit identifier (all HiPNUC messages are) */
    uint8_t  remote;      /* 1: remote transmission request */
    uint8_t  len;         /* payload bytes 0..64 (a count, not a DLC code) */
    uint8_t  data[64];
} hipnuc_can_frame_t;

/* Returns 1 when frame is a HiPNUC data message (out is cleared, then
 * filled), 0 when it is some other frame, and -1 when it uses a HiPNUC data
 * identifier but its payload is malformed. out is written only on 1. The
 * J1939 identifiers are proprietary PGNs that other devices may use too: on a
 * shared bus, keep only the messages whose out->node_id is your device's. */
int hipnuc_can_decode(const hipnuc_can_frame_t *frame, hipnuc_data_t *out);

/*
 * Register access. Build the request, send it, then pass every received frame
 * to hipnuc_can_reply() until it returns the reply (or give up after about
 * 1 s). The device:
 *  - handles one request at a time and drops a request that arrives meanwhile;
 *  - does not answer a refused request, a reset (REBOOT, FRESET) or a trigger;
 *  - answers a write when it has received it, which is not proof that the
 *    value was taken: read it back;
 *  - returns 16-bit registers sign-extended to 32 bits; write them with the
 *    upper 16 bits zero;
 *  - can take several hundred milliseconds for a write that is saved to flash.
 */
void hipnuc_can_read_request(hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t reg);
void hipnuc_can_write_request(hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t reg, uint32_t value);

/* Returns HIPNUC_CAN_READ or HIPNUC_CAN_WRITE when frame is node's reply to
 * host (reg and value are filled), else 0. Any reply of node matches: compare
 * reg with the register you asked for (a late reply can still arrive). */
int hipnuc_can_reply(const hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t *reg, uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_CAN_H */
