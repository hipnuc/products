/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC CAN protocol: J1939 measurement PGNs, the CANFD83 frame and the
 * J1939 register access (configuration) frames.
 *
 * Portable C99, no dynamic memory, no stdio, no global state. Copy
 * hipnuc_j1939.c, hipnuc_j1939.h, hipnuc_can_frame.h and hipnuc_sample.h
 * (with hipnuc_dec.h and nmea_dec.h, which it includes) into your project.
 *
 * Every data PGN is decoded into a hipnuc_sample_t whose `valid` bits name
 * only the fields carried by that frame; the caller merges frames as it sees
 * fit (see hipnuc_j1939_merge()). Remote and error frames are rejected, and
 * the full 8-bit source address is reported so several devices on one bus
 * stay apart.
 */

#ifndef HIPNUC_J1939_H
#define HIPNUC_J1939_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_can_frame.h"
#include "hipnuc_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

/* J1939 constants used by HiPNUC devices */
#define HIPNUC_J1939_PRIORITY        3U
#define HIPNUC_J1939_BROADCAST       0xFFU
#define HIPNUC_J1939_HOST_ADDRESS    0x55U        /* recommended host source address */
#define HIPNUC_J1939_DEFAULT_NODE    0x08U        /* factory device source address */
#define HIPNUC_J1939_PGN_CONFIG      0xEF00U      /* register read/write, PDU1 */

/* Data PGNs (PF = 0xFF, PS = HIREG address) */
#define HIPNUC_J1939_PGN_POSITION    0xFF10U      /* i32 lat, i32 lon x1e-7 deg */
#define HIPNUC_J1939_PGN_ALTITUDE    0xFF14U      /* i32 msl cm, i16 undulation cm, i16 diff age x0.01 s */
#define HIPNUC_J1939_PGN_GNSS_STATUS 0xFF18U      /* solq, solq heading, nv, nv heading, ins status */
#define HIPNUC_J1939_PGN_VELOCITY    0xFF26U      /* i16 e,n,u,ground speed x0.01 m/s */
#define HIPNUC_J1939_PGN_TIME        0xFF2FU      /* y-2000, m, d, h, min, s, u16 ms */
#define HIPNUC_J1939_PGN_ACC         0xFF34U      /* i16 x,y,z, LSB = 1/2048 G */
#define HIPNUC_J1939_PGN_GYR         0xFF37U      /* i16 x,y,z, LSB = 2000/32768 deg/s */
#define HIPNUC_J1939_PGN_MAG         0xFF3AU      /* i16 x,y,z, LSB = 1000/32768 uT */
#define HIPNUC_J1939_PGN_ROLL_PITCH  0xFF3DU      /* i32 roll, i32 pitch x0.001 deg */
#define HIPNUC_J1939_PGN_YAW         0xFF41U      /* i32 heading 0..360 CW x0.001 deg, i32 yaw CCW x0.001 deg */
#define HIPNUC_J1939_PGN_TEMP        0xFF43U      /* i16 temp x0.01 degC; bytes 4..7 reserved */
#define HIPNUC_J1939_PGN_QUAT        0xFF46U      /* i16 w,x,y,z x0.0001 */
#define HIPNUC_J1939_PGN_INCLINATION 0xFF4AU      /* i32 x, i32 y x0.001 deg */
#define HIPNUC_J1939_PGN_CANFD83     0xFF5BU      /* CAN FD, bitmap selected float fields */

/* CANFD83 bitmap (same bit positions as HI83 for the shared fields) */
#define CANFD83_MAP_ACC_B            (UINT32_C(1) << 0)   /* 3 x f32 m/s^2 */
#define CANFD83_MAP_GYR_B            (UINT32_C(1) << 1)   /* 3 x f32 rad/s */
#define CANFD83_MAP_MAG_B            (UINT32_C(1) << 2)   /* 3 x f32 uT */
#define CANFD83_MAP_RPY              (UINT32_C(1) << 3)   /* 3 x f32 deg */
#define CANFD83_MAP_QUAT             (UINT32_C(1) << 4)   /* 4 x f32 */
#define CANFD83_MAP_SYSTEM_TIME      (UINT32_C(1) << 5)   /* u64 us */
#define CANFD83_MAP_UTC              (UINT32_C(1) << 6)   /* 8 bytes, HI83 layout */
#define CANFD83_MAP_TEMPERATURE      (UINT32_C(1) << 8)   /* f32 degC */
#define CANFD83_MAP_SUPPORTED        UINT32_C(0x0000017F)
#define CANFD83_MAP_DEFAULT          UINT32_C(0x0000012B)
#define CANFD83_HEADER_SIZE          8    /* u32 bitmap, u16 main_status, u8 ins_status, u8 sequence */

/* Result of hipnuc_j1939_parse() */
typedef enum {
    HIPNUC_J1939_MSG_NONE = 0,     /* not a HiPNUC data frame */
    HIPNUC_J1939_MSG_ACC,
    HIPNUC_J1939_MSG_GYR,
    HIPNUC_J1939_MSG_MAG,
    HIPNUC_J1939_MSG_ROLL_PITCH,
    HIPNUC_J1939_MSG_YAW,
    HIPNUC_J1939_MSG_TEMP,
    HIPNUC_J1939_MSG_QUAT,
    HIPNUC_J1939_MSG_INCLINATION,
    HIPNUC_J1939_MSG_TIME,
    HIPNUC_J1939_MSG_POSITION,
    HIPNUC_J1939_MSG_ALTITUDE,
    HIPNUC_J1939_MSG_GNSS_STATUS,
    HIPNUC_J1939_MSG_VELOCITY,
    HIPNUC_J1939_MSG_CANFD83
} hipnuc_j1939_msg_t;

/**
 * Decode one received frame into a sample.
 *
 * On success the sample is cleared, `source` is HIPNUC_SOURCE_J1939 (or
 * HIPNUC_SOURCE_CANFD83), `node_id` is the 8-bit source address and `valid`
 * names the fields carried by this frame. CANFD83 also fills main_status and
 * ins_status; `canfd83_sequence` receives the frame counter when not NULL.
 *
 * @return message type > 0, HIPNUC_J1939_MSG_NONE for frames that are not
 *         HiPNUC data (standard frames, other PGNs, config frames), or -1
 *         for a HiPNUC PGN with an invalid payload (short, remote, error,
 *         unsupported CANFD83 bitmap).
 */
int hipnuc_j1939_parse(const hipnuc_can_frame_t *frame, hipnuc_sample_t *sample, uint8_t *canfd83_sequence);

/**
 * Copy the valid fields of `part` into `into` (same source address) and
 * merge the valid bits. Used to assemble one sample from several PGNs.
 */
void hipnuc_j1939_merge(hipnuc_sample_t *into, const hipnuc_sample_t *part);

/* Identifier helpers */
uint32_t hipnuc_j1939_pgn(uint32_t id);              /* PF/PS of a 29-bit identifier */
uint8_t  hipnuc_j1939_source_address(uint32_t id);   /* low 8 bits */
uint32_t hipnuc_j1939_data_id(uint32_t pgn, uint8_t source);  /* priority 3 data frame id */

/* Register access over CAN: PGN 0xEF00, payload addr(u16 LE), cmd, status, value(u32 LE). */
typedef enum {
    HIPNUC_J1939_CMD_READ = 0x03,
    HIPNUC_J1939_CMD_WRITE = 0x06
} hipnuc_j1939_cmd_t;

void hipnuc_j1939_build_reg_write(uint8_t dest, uint8_t source, uint16_t addr, uint32_t value, hipnuc_can_frame_t *out);
void hipnuc_j1939_build_reg_read(uint8_t dest, uint8_t source, uint16_t addr, hipnuc_can_frame_t *out);
/* Trigger one data PGN immediately (register 0x0096). */
void hipnuc_j1939_build_trigger(uint8_t dest, uint8_t source, uint32_t pgn, hipnuc_can_frame_t *out);

/** 1 when the frame is a register access frame (request or reply). */
int hipnuc_j1939_is_config(const hipnuc_can_frame_t *frame);

/**
 * Parse a register access frame. Returns 0 on success, -1 otherwise.
 * Any output pointer may be NULL.
 */
int hipnuc_j1939_parse_config(const hipnuc_can_frame_t *frame, uint8_t *source, uint16_t *addr,
                              hipnuc_j1939_cmd_t *cmd, uint8_t *status, uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_J1939_H */
