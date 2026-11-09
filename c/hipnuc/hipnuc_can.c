/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC CAN messages; see hipnuc_can.h.
 */

#include "hipnuc_can.h"
#include "hipnuc_internal.h"

#include <string.h>

#define PRIORITY   UINT32_C(3)
#define PGN_CONFIG 0xEF00U   /* register access, PDU1: PS is the destination */

/* Data PGNs (PDU2, priority 3, source = device address) */
#define PGN_POSITION    0xFF10U   /* i32 lat, i32 lon, x1e-7 deg */
#define PGN_ALTITUDE    0xFF14U   /* i32 msl cm, i16 undulation cm, i16 diff age x0.01 s */
#define PGN_GNSS_STATUS 0xFF18U   /* position quality, heading quality, satellites x2, ins status */
#define PGN_VELOCITY    0xFF26U   /* i16 e, n, u, ground speed, x0.01 m/s */
#define PGN_TIME        0xFF2FU   /* year-2000, month, day, hour, min, s, u16 ms */
#define PGN_ACC         0xFF34U   /* i16 x, y, z; 1/2048 G */
#define PGN_GYR         0xFF37U   /* i16 x, y, z; 2000/32768 deg/s */
#define PGN_MAG         0xFF3AU   /* i16 x, y, z; 1000/32768 uT */
#define PGN_ROLL_PITCH  0xFF3DU   /* i32 roll, i32 pitch, x0.001 deg */
#define PGN_YAW         0xFF41U   /* i32 heading (clockwise), i32 yaw (counter-clockwise), x0.001 deg */
#define PGN_TEMP        0xFF43U   /* i16 x0.01 degC */
#define PGN_QUAT        0xFF46U   /* i16 w, x, y, z, x0.0001 */
#define PGN_INCLINATION 0xFF4AU   /* i32 x, i32 y, x0.001 deg */
#define PGN_CANFD83     0xFF5BU   /* CAN FD: bitmap-selected float fields */

#define ACC_SCALE   ((16.0f / 32768.0f) * HIPNUC_GRAVITY)     /* -> m/s^2 */
#define GYR_SCALE   ((2000.0f / 32768.0f) * HIPNUC_DEG2RAD)   /* -> rad/s */
#define MAG_SCALE   ((1000.0f / 32768.0f) * 1e-6f)            /* -> T */
#define MDEG2RAD    (0.001f * HIPNUC_DEG2RAD)

/* CANFD83 header: u32 bitmap, u16 main_status, u8 ins_status, u8 sequence;
 * then the fields of bits 0-6 and 8 in ascending order (HI83 encodings). */
#define FD83_HEADER    8U
#define FD83_SUPPORTED UINT32_C(0x0000017F)

static uint32_t pgn_of(uint32_t id)
{
    uint32_t pgn = (id >> 8) & 0x3FFFFU;
    if (((pgn >> 8) & 0xFFU) < 0xF0U) pgn &= 0x3FF00U;   /* PDU1: PS is an address */
    return pgn;
}

/* Smallest payload of a HiPNUC data PGN; 0 when the PGN is not one. */
static uint8_t min_len(uint32_t pgn)
{
    switch (pgn) {
    case PGN_ACC: case PGN_GYR: case PGN_MAG: case PGN_VELOCITY: return 6;
    case PGN_ROLL_PITCH: case PGN_QUAT: case PGN_INCLINATION: case PGN_TIME:
    case PGN_POSITION: case PGN_ALTITUDE: return 8;
    case PGN_YAW: return 4;
    case PGN_TEMP: return 2;
    case PGN_GNSS_STATUS: return 5;
    case PGN_CANFD83: return FD83_HEADER;
    default: return 0;
    }
}

static size_t fd83_size(uint32_t bitmap)
{
    static const uint8_t field_size[9] = {12, 12, 12, 12, 16, 8, 8, 0, 4};
    size_t size = FD83_HEADER;
    unsigned bit;
    for (bit = 0; bit < 9; ++bit) {
        if (bitmap & (UINT32_C(1) << bit)) size += field_size[bit];
    }
    return size;
}

static void decode_fd83(const uint8_t *p, hipnuc_data_t *d)
{
    uint32_t bm = hn_u32(p);
    uint16_t status = hn_u16(p + 4);
    const uint8_t *f = p + FD83_HEADER;
    int i;

    d->source = HIPNUC_SOURCE_CANFD83;
    hn_set_status(d, status);
    d->sequence = p[7];
    d->valid |= HIPNUC_VALID_SEQUENCE;
    if (bm & (1UL << 0)) { hn_f32n(d->acc, f, 3); f += 12; d->valid |= HIPNUC_VALID_ACC; }
    if (bm & (1UL << 1)) { hn_f32n(d->gyr, f, 3); f += 12; d->valid |= HIPNUC_VALID_GYR; }
    if (bm & (1UL << 2)) {
        for (i = 0; i < 3; ++i) d->mag[i] = hn_f32(f + 4 * i) * 1e-6f;
        f += 12;
        d->valid |= HIPNUC_VALID_MAG;
    }
    if (bm & (1UL << 3)) {
        d->roll = hn_f32(f) * HIPNUC_DEG2RAD;
        d->pitch = hn_f32(f + 4) * HIPNUC_DEG2RAD;
        d->yaw = hn_f32(f + 8) * HIPNUC_DEG2RAD;
        f += 12;
        d->valid |= HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_YAW;
    }
    if (bm & (1UL << 4)) { hn_f32n(d->quat, f, 4); f += 16; d->valid |= HIPNUC_VALID_QUAT; }
    if (bm & (1UL << 5)) { d->device_time_us = hn_u64(f); f += 8; d->valid |= HIPNUC_VALID_DEVICE_TIME; }
    if (bm & (1UL << 6)) {
        if (!(status & HIPNUC_STATUS_UTC_UNSYNC)) hn_set_utc(d, f[0], f[1], f[2], f[3], f[4], hn_u16(f + 5));
        f += 8;
    }
    if (bm & (1UL << 8)) { d->temperature = hn_f32(f); d->valid |= HIPNUC_VALID_TEMPERATURE; }
}

static void decode_j1939(uint32_t pgn, const uint8_t *p, uint8_t len, hipnuc_data_t *d)
{
    int i;
    d->source = HIPNUC_SOURCE_J1939;
    switch (pgn) {
    case PGN_ACC:
        for (i = 0; i < 3; ++i) d->acc[i] = hn_i16(p + 2 * i) * ACC_SCALE;
        d->valid |= HIPNUC_VALID_ACC;
        break;
    case PGN_GYR:
        for (i = 0; i < 3; ++i) d->gyr[i] = hn_i16(p + 2 * i) * GYR_SCALE;
        d->valid |= HIPNUC_VALID_GYR;
        break;
    case PGN_MAG:
        for (i = 0; i < 3; ++i) d->mag[i] = hn_i16(p + 2 * i) * MAG_SCALE;
        d->valid |= HIPNUC_VALID_MAG;
        break;
    case PGN_ROLL_PITCH:
        d->roll = (float)hn_i32(p) * MDEG2RAD;
        d->pitch = (float)hn_i32(p + 4) * MDEG2RAD;
        d->valid |= HIPNUC_VALID_ROLL_PITCH;
        break;
    case PGN_YAW:
        d->heading = (float)hn_i32(p) * MDEG2RAD;
        d->valid |= HIPNUC_VALID_HEADING;
        if (len >= 8) {
            d->yaw = (float)hn_i32(p + 4) * MDEG2RAD;
            d->valid |= HIPNUC_VALID_YAW;
        }
        break;
    case PGN_TEMP:   /* bytes 4..7 are a placeholder, not a pressure */
        d->temperature = hn_i16(p) * 0.01f;
        d->valid |= HIPNUC_VALID_TEMPERATURE;
        break;
    case PGN_QUAT:
        for (i = 0; i < 4; ++i) d->quat[i] = hn_i16(p + 2 * i) * 0.0001f;
        d->valid |= HIPNUC_VALID_QUAT;
        break;
    case PGN_INCLINATION:
        d->inclination[0] = (float)hn_i32(p) * MDEG2RAD;
        d->inclination[1] = (float)hn_i32(p + 4) * MDEG2RAD;
        d->valid |= HIPNUC_VALID_INCLINATION;
        break;
    case PGN_TIME:   /* a zero date (no time yet) is not UTC */
        if (p[0] || p[1] || p[2]) {
            hipnuc_utc_t u;
            u.year = (uint16_t)(2000U + p[0]);
            u.month = p[1];
            u.day = p[2];
            u.hour = p[3];
            u.minute = p[4];
            u.second = p[5];
            u.millisecond = hn_u16(p + 6);
            if (hn_utc_valid(&u)) {
                d->utc = u;
                d->valid |= HIPNUC_VALID_UTC | HIPNUC_VALID_UTC_TIME_OF_DAY;
            }
        }
        break;
    case PGN_POSITION:
        d->latitude = hn_i32(p) * 1e-7;
        d->longitude = hn_i32(p + 4) * 1e-7;
        d->valid |= HIPNUC_VALID_POSITION;
        break;
    case PGN_ALTITUDE:
        d->altitude_msl = hn_i32(p) * 0.01;
        d->undulation = hn_i16(p + 4) * 0.01f;
        d->diff_age = hn_i16(p + 6) * 0.01f;
        d->valid |= HIPNUC_VALID_ALTITUDE | HIPNUC_VALID_UNDULATION | HIPNUC_VALID_DIFF_AGE;
        break;
    case PGN_GNSS_STATUS:
        d->position_quality = p[0];
        d->heading_quality = p[1];
        d->position_satellites = p[2];
        d->heading_satellites = p[3];
        d->ins_status = p[4];
        d->valid |= HIPNUC_VALID_POSITION_QUALITY | HIPNUC_VALID_HEADING_QUALITY | HIPNUC_VALID_POSITION_SATELLITES |
                    HIPNUC_VALID_HEADING_SATELLITES | HIPNUC_VALID_INS_STATUS;
        break;
    case PGN_VELOCITY:   /* bytes 6..7: ground speed, which follows from vel_enu */
        for (i = 0; i < 3; ++i) d->vel_enu[i] = hn_i16(p + 2 * i) * 0.01f;
        d->valid |= HIPNUC_VALID_VELOCITY_ENU;
        break;
    default:
        break;
    }
}

int hipnuc_can_decode(const hipnuc_can_frame_t *frame, hipnuc_data_t *out)
{
    uint32_t pgn;
    uint8_t need;
    if (!frame || !frame->extended) return 0;   /* HiPNUC data is J1939 only */
    pgn = pgn_of(frame->id);
    need = min_len(pgn);
    if (need == 0) return 0;                    /* another PGN, or a register frame */
    if (frame->remote || frame->len > sizeof(frame->data) || frame->len < need) return -1;
    if (pgn == PGN_CANFD83) {
        uint32_t bm = hn_u32(frame->data);
        if (bm == 0 || (bm & ~FD83_SUPPORTED) || frame->len < fd83_size(bm)) return -1;
    } else if (frame->len > 8) {
        return -1;
    }
    if (out) {
        memset(out, 0, sizeof(*out));
        if (pgn == PGN_CANFD83) decode_fd83(frame->data, out);
        else decode_j1939(pgn, frame->data, frame->len, out);
        out->node_id = (uint8_t)(frame->id & 0xFFU);
        out->valid |= HIPNUC_VALID_NODE_ID;
    }
    return 1;
}

/* ---- Register access ---------------------------------------------------- */

static void request(hipnuc_can_frame_t *f, uint8_t host, uint8_t node, uint16_t reg, uint8_t cmd, uint32_t value)
{
    memset(f, 0, sizeof(*f));
    f->id = (PRIORITY << 26) | ((uint32_t)PGN_CONFIG << 8) | ((uint32_t)node << 8) | host;
    f->extended = 1;
    f->len = 8;
    hn_put_u16(f->data, reg);
    f->data[2] = cmd;
    hn_put_u32(f->data + 4, value);
}

void hipnuc_can_read_request(hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t reg)
{
    if (frame) request(frame, host, node, reg, HIPNUC_CAN_READ, 0);
}

void hipnuc_can_write_request(hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t reg, uint32_t value)
{
    if (frame) request(frame, host, node, reg, HIPNUC_CAN_WRITE, value);
}

int hipnuc_can_reply(const hipnuc_can_frame_t *frame, uint8_t host, uint8_t node, uint16_t *reg, uint32_t *value)
{
    const uint8_t *d;
    if (!frame || !frame->extended || frame->remote || frame->len != 8) return 0;
    if (((frame->id >> 16) & 0xFFU) != 0xEFU) return 0;                       /* PF of the register PGN */
    if (((frame->id >> 8) & 0xFFU) != host || (frame->id & 0xFFU) != node) return 0;
    d = frame->data;
    if (d[2] != HIPNUC_CAN_READ && d[2] != HIPNUC_CAN_WRITE) return 0;
    if (reg) *reg = hn_u16(d);
    if (value) *value = hn_u32(d + 4);
    return d[2];
}
