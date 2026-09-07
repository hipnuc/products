/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal CAN frame shared by the J1939 decoder and the CAN firmware update
 * client. Independent of any operating system driver: convert your driver's
 * frame into this structure before calling the SDK.
 */

#ifndef HIPNUC_CAN_FRAME_H
#define HIPNUC_CAN_FRAME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;            /* 11-bit or 29-bit identifier without flag bits */
    uint8_t  is_extended;   /* 1: 29-bit identifier (J1939) */
    uint8_t  is_remote;     /* 1: remote transmission request; never decoded */
    uint8_t  is_error;      /* 1: error frame reported by the driver; never decoded */
    uint8_t  len;           /* payload bytes 0..64 (a byte count, not a DLC code) */
    uint8_t  data[64];
} hipnuc_can_frame_t;

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_CAN_FRAME_H */
