/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC serial stream decoder; see hipnuc_decoder.h.
 *
 * The decoder keeps a window of at most one frame. win[lo] is always the
 * oldest byte not yet handed out. A byte that cannot start a frame leaves the
 * window as text. A candidate whose length or CRC is wrong gives up only its
 * first byte, and the rest of its span is scanned again: inside that span a
 * candidate must also have a tag and a length this SDK decodes ("strict"),
 * which bounds the work a damaged or hostile stream can cause.
 */

#include "hipnuc_decoder.h"
#include "hipnuc_internal.h"

#include <string.h>

#define SYNC1 0x5AU
#define SYNC2 0xA5U
#define HEADER 6U                   /* sync (2), payload length (2), CRC (2) */
#define MAX_PAYLOAD (HIPNUC_FRAME_MAX - HEADER)

#define TAG_HI91 0x91U
#define TAG_HI81 0x81U
#define TAG_HI83 0x83U
#define HI91_SIZE 76U
#define HI81_SIZE 104U

/* HI83 data_bitmap bits decoded here: 0-19, 21, 30, 31. Bit 16 (odometer
 * speed) is skipped. On the wire bits 30 and 31 follow bit 27, so with bits
 * 22-29 unsupported every field comes in ascending bit order. */
#define HI83_SUPPORTED   UINT32_C(0xC02FFFFF)
#define HI83_HAS_DOUBLE  ((UINT32_C(1) << 14) | (UINT32_C(1) << 30))
#define HI83_INS_FIELDS  UINT32_C(0xC007F000)   /* bits 12-18, 30, 31: INS products */
#define HI83_MAX_PAYLOAD 248U                   /* every supported bit set */

/* Peek results */
#define PEEK_SKIP (-1)   /* the first byte cannot start a frame */
#define PEEK_MORE 0      /* more bytes are needed to decide */
#define PEEK_OK   1      /* plausible header; *total is the whole frame size */

/* CRC-16/XMODEM: poly 0x1021, init 0, no reflection, no final XOR. */
static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

static uint16_t crc16(uint16_t crc, const uint8_t *p, size_t len)
{
    while (len--) crc = (uint16_t)((crc << 8) ^ crc_table[(uint8_t)((crc >> 8) ^ *p++)]);
    return crc;
}

/* The CRC covers the sync and length bytes and the payload. */
static int crc_ok(const uint8_t *f, size_t total)
{
    return crc16(crc16(0, f, 4), f + HEADER, total - HEADER) == hn_u16(f + 4);
}

/* ---- Payloads ----------------------------------------------------------- */

/* Payload size of an HI83 bitmap, or 0 when this SDK does not decode it. */
static size_t hi83_size(uint32_t bitmap)
{
    static const uint8_t field_size[32] = {
        12, 12, 12, 12, 16, 8, 8, 4, 4, 12, 12, 12, 12, 12, 24, 4,
        4, 4, 4, 4, 0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 24, 12
    };
    size_t size = 8;
    unsigned bit;
    if (bitmap & ~HI83_SUPPORTED) return 0;
    if (sizeof(double) != 8 && (bitmap & HI83_HAS_DOUBLE)) return 0;
    for (bit = 0; bit < 32; ++bit) {
        if (bitmap & (UINT32_C(1) << bit)) size += field_size[bit];
    }
    return size;
}

static void decode_hi91(const uint8_t *p, hipnuc_data_t *d)
{
    int i;
    d->source = HIPNUC_SOURCE_HI91;
    hn_set_status(d, hn_u16(p + 1));
    d->temperature = (float)(int8_t)p[3];
    d->pressure = hn_f32(p + 4);
    d->system_time_ms = hn_u32(p + 8);
    for (i = 0; i < 3; ++i) {
        d->acc[i] = hn_f32(p + 12 + 4 * i) * HIPNUC_GRAVITY;
        d->gyr[i] = hn_f32(p + 24 + 4 * i) * HIPNUC_DEG2RAD;
        d->mag[i] = hn_f32(p + 36 + 4 * i) * 1e-6f;
    }
    d->roll = hn_f32(p + 48) * HIPNUC_DEG2RAD;
    d->pitch = hn_f32(p + 52) * HIPNUC_DEG2RAD;
    d->yaw = hn_f32(p + 56) * HIPNUC_DEG2RAD;
    hn_f32n(d->quat, p + 60, 4);
    d->valid |= HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG | HIPNUC_VALID_ROLL_PITCH |
                HIPNUC_VALID_YAW | HIPNUC_VALID_QUAT | HIPNUC_VALID_TEMPERATURE | HIPNUC_VALID_PRESSURE |
                HIPNUC_VALID_SYSTEM_TIME;
}

/* HI81: fixed point. Offset 1 (status) is reserved in the INS manual and is
 * not interpreted; the date is zero until the device has a time, so a valid
 * calendar date is the test for UTC. Offsets 32 and 77 are reserved too. */
static void decode_hi81(const uint8_t *p, hipnuc_data_t *d)
{
    int i;
    d->source = HIPNUC_SOURCE_HI81;
    d->ins_status = p[3];
    d->gps_week = hn_u16(p + 4);
    d->gps_tow_ms = hn_u32(p + 6);
    for (i = 0; i < 3; ++i) {
        d->gyr[i] = hn_i16(p + 12 + 2 * i) * 0.001f;
        d->acc[i] = hn_i16(p + 18 + 2 * i) * 0.0048828f;
        d->mag[i] = hn_i16(p + 24 + 2 * i) * 0.030517e-6f;
        d->vel_enu[i] = hn_i16(p + 78 + 2 * i) * 0.01f;
        d->acc_enu[i] = hn_i16(p + 84 + 2 * i) * 0.0048828f;
    }
    d->pressure = (float)hn_i16(p + 30) + 100000.0f;
    d->temperature = (float)(int8_t)p[34];
    hn_set_utc(d, p[35], p[36], p[37], p[38], p[39], hn_u16(p + 40));
    d->roll = hn_i16(p + 42) * 0.01f * HIPNUC_DEG2RAD;
    d->pitch = hn_i16(p + 44) * 0.01f * HIPNUC_DEG2RAD;
    d->heading = hn_u16(p + 46) * 0.01f * HIPNUC_DEG2RAD;
    for (i = 0; i < 4; ++i) d->quat[i] = hn_i16(p + 48 + 2 * i) * 0.0001f;
    d->longitude = hn_i32(p + 56) * 1e-7;
    d->latitude = hn_i32(p + 60) * 1e-7;
    d->altitude_msl = hn_i32(p + 64) * 0.001;
    d->pdop = p[68] * 0.1f;
    d->hdop = p[69] * 0.1f;
    d->position_quality = p[70];
    d->position_satellites = p[71];
    d->heading_quality = p[72];
    d->heading_satellites = p[73];
    d->diff_age = (float)p[74];
    d->undulation = hn_i16(p + 75) * 0.01f;
    d->valid |= HIPNUC_VALID_INS_STATUS | HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG |
                HIPNUC_VALID_PRESSURE | HIPNUC_VALID_TEMPERATURE | HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_HEADING |
                HIPNUC_VALID_QUAT | HIPNUC_VALID_POSITION | HIPNUC_VALID_ALTITUDE | HIPNUC_VALID_PDOP |
                HIPNUC_VALID_HDOP | HIPNUC_VALID_POSITION_QUALITY | HIPNUC_VALID_HEADING_QUALITY |
                HIPNUC_VALID_POSITION_SATELLITES | HIPNUC_VALID_HEADING_SATELLITES | HIPNUC_VALID_DIFF_AGE |
                HIPNUC_VALID_UNDULATION | HIPNUC_VALID_VELOCITY_ENU | HIPNUC_VALID_ACC_ENU;
    if (d->gps_week || d->gps_tow_ms) d->valid |= HIPNUC_VALID_GPS_TIME;
}

/* HI83: an 8-byte header (tag, main_status, ins_status, bitmap), then the
 * selected fields. The bitmap and the length were checked by the caller. */
static void decode_hi83(const uint8_t *p, hipnuc_data_t *d)
{
    uint16_t status = hn_u16(p + 1);
    uint32_t bm = hn_u32(p + 4);
    const uint8_t *f = p + 8;
    int i;

    d->source = HIPNUC_SOURCE_HI83;
    hn_set_status(d, status);
    if (bm & HI83_INS_FIELDS) {   /* ins_status means something only on INS products */
        d->ins_status = p[3];
        d->valid |= HIPNUC_VALID_INS_STATUS;
    }
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
    if (bm & (1UL << 7)) { d->pressure = hn_f32(f); f += 4; d->valid |= HIPNUC_VALID_PRESSURE; }
    if (bm & (1UL << 8)) { d->temperature = hn_f32(f); f += 4; d->valid |= HIPNUC_VALID_TEMPERATURE; }
    if (bm & (1UL << 9)) {
        d->inclination[0] = hn_f32(f) * HIPNUC_DEG2RAD;
        d->inclination[1] = hn_f32(f + 4) * HIPNUC_DEG2RAD;
        d->inclination_yaw = hn_f32(f + 8) * HIPNUC_DEG2RAD;
        f += 12;
        d->valid |= HIPNUC_VALID_INCLINATION | HIPNUC_VALID_INCLINATION_YAW;
    }
    if (bm & (1UL << 10)) { hn_f32n(d->heave_m, f, 3); f += 12; d->valid |= HIPNUC_VALID_HEAVE; }
    if (bm & (1UL << 11)) { hn_f32n(d->heave_hz, f, 3); f += 12; d->valid |= HIPNUC_VALID_HEAVE_FREQUENCY; }
    if (bm & (1UL << 12)) { hn_f32n(d->vel_enu, f, 3); f += 12; d->valid |= HIPNUC_VALID_VELOCITY_ENU; }
    if (bm & (1UL << 13)) { hn_f32n(d->acc_enu, f, 3); f += 12; d->valid |= HIPNUC_VALID_ACC_ENU; }
    if (bm & (1UL << 14)) {
        d->longitude = hn_f64(f);
        d->latitude = hn_f64(f + 8);
        d->altitude_msl = hn_f64(f + 16);
        f += 24;
        d->valid |= HIPNUC_VALID_POSITION | HIPNUC_VALID_ALTITUDE;
    }
    if (bm & (1UL << 15)) {
        d->position_quality = f[0];
        d->position_satellites = f[1];
        d->heading_quality = f[2];
        d->heading_satellites = f[3];
        f += 4;
        d->valid |= HIPNUC_VALID_POSITION_QUALITY | HIPNUC_VALID_POSITION_SATELLITES |
                    HIPNUC_VALID_HEADING_QUALITY | HIPNUC_VALID_HEADING_SATELLITES;
    }
    if (bm & (1UL << 16)) f += 4;   /* odometer speed: not delivered by the firmware */
    if (bm & (1UL << 17)) { d->undulation = hn_f32(f); f += 4; d->valid |= HIPNUC_VALID_UNDULATION; }
    if (bm & (1UL << 18)) { d->diff_age = hn_f32(f); f += 4; d->valid |= HIPNUC_VALID_DIFF_AGE; }
    if (bm & (1UL << 19)) { d->node_id = f[0]; f += 4; d->valid |= HIPNUC_VALID_NODE_ID; }
    if (bm & (1UL << 21)) {
        for (i = 0; i < 3; ++i) d->att_std[i] = hn_f32(f + 4 * i) * HIPNUC_DEG2RAD;
        f += 12;
        d->valid |= HIPNUC_VALID_ATT_STD;
    }
    if (bm & (1UL << 30)) {
        d->gnss_longitude = hn_f64(f);
        d->gnss_latitude = hn_f64(f + 8);
        d->gnss_altitude_msl = hn_f64(f + 16);
        f += 24;
        d->valid |= HIPNUC_VALID_GNSS_POSITION | HIPNUC_VALID_GNSS_ALTITUDE;
    }
    if (bm & (1UL << 31)) { hn_f32n(d->gnss_vel_enu, f, 3); d->valid |= HIPNUC_VALID_GNSS_VELOCITY; }
}

/* Decode a payload this SDK supports into out (cleared first). Returns 1, or
 * 0 for a tag, a length or an HI83 bitmap it does not decode; out is then
 * left alone. */
static int decode_payload(const uint8_t *p, size_t len, hipnuc_data_t *out)
{
    void (*decode)(const uint8_t *, hipnuc_data_t *);
    if (len == 0) return 0;
    switch (p[0]) {
    case TAG_HI91:
        if (len != HI91_SIZE) return 0;
        decode = decode_hi91;
        break;
    case TAG_HI81:
        if (len != HI81_SIZE) return 0;
        decode = decode_hi81;
        break;
    case TAG_HI83:
        if (len < 8 || hi83_size(hn_u32(p + 4)) != len) return 0;
        decode = decode_hi83;
        break;
    default:
        return 0;
    }
    if (out) {
        memset(out, 0, sizeof(*out));
        decode(p, out);
    }
    return 1;
}

/* ---- Frame scanning ----------------------------------------------------- */

/* Can p[0 .. avail) start a frame? `strict` also asks for a tag and a length
 * this SDK decodes. The CRC is not checked here. */
static int peek(const uint8_t *p, size_t avail, size_t *total, int strict)
{
    size_t len;
    if (p[0] != SYNC1) return PEEK_SKIP;
    if (avail < 2) return PEEK_MORE;
    if (p[1] != SYNC2) return PEEK_SKIP;
    if (avail < 4) return PEEK_MORE;
    len = hn_u16(p + 2);
    if (len == 0 || len > MAX_PAYLOAD) return PEEK_SKIP;
    if (strict) {
        if (avail < HEADER + 1) return PEEK_MORE;
        switch (p[HEADER]) {
        case TAG_HI91:
            if (len != HI91_SIZE) return PEEK_SKIP;
            break;
        case TAG_HI81:
            if (len != HI81_SIZE) return PEEK_SKIP;
            break;
        case TAG_HI83:
            if (len < 8 || len > HI83_MAX_PAYLOAD) return PEEK_SKIP;
            if (avail < HEADER + 8) return PEEK_MORE;
            if (hi83_size(hn_u32(p + HEADER + 4)) != len) return PEEK_SKIP;
            break;
        default:
            return PEEK_SKIP;
        }
    }
    *total = len + HEADER;
    return PEEK_OK;
}

/* One byte outside any frame. Returns 1 when it completes a line. */
static int text_byte(hipnuc_decoder_t *d, uint8_t b)
{
    if (b == '\r' || b == '\n') {
        int done = d->line_len > 0 && !d->line_over;
        d->line[d->line_len] = '\0';
        d->line_len = 0;
        d->line_over = 0;
        return done;
    }
    if (b >= 0x20 && b <= 0x7E) {
        if (d->line_over) return 0;
        if (d->line_len >= HIPNUC_LINE_MAX) {
            d->line_over = 1;
            d->line_len = 0;
            return 0;
        }
        d->line[d->line_len++] = (char)b;
        d->line[d->line_len] = '\0';
        return 0;
    }
    d->line_len = 0;   /* binary: whatever came before is not a line */
    d->line_over = 0;
    return 0;
}

static void drop_line(hipnuc_decoder_t *d)
{
    d->line_len = 0;
    d->line_over = 0;
}

/* k bytes left the window. When the last byte of a rejected candidate that
 * looked like a real frame leaves, the unfinished line is its binary tail:
 * drop it. Returns 1 in that case. */
static int advance(hipnuc_decoder_t *d, uint32_t k)
{
    d->pos += k;
    if (d->span_open && (int32_t)(d->pos - d->span_end) >= 0) d->span_open = 0;
    if (d->taint_open && (int32_t)(d->pos - d->taint_end) >= 0) {
        d->taint_open = 0;
        drop_line(d);
        return 1;
    }
    return 0;
}

/* A byte leaves the window as text. Returns 1 when it completes a line. */
static int text_out(hipnuc_decoder_t *d, uint8_t b)
{
    int done = text_byte(d, b);
    return advance(d, 1) ? 0 : done;
}

/* The candidate at win[lo] was rejected; it reaches `total` bytes ahead. */
static void reject(hipnuc_decoder_t *d, size_t total, int looked_real)
{
    uint32_t end = d->pos + (uint32_t)total;
    if (!d->span_open || (int32_t)(end - d->span_end) > 0) d->span_end = end;
    d->span_open = 1;
    if (looked_real && (!d->taint_open || (int32_t)(end - d->taint_end) > 0)) {
        d->taint_end = end;
        d->taint_open = 1;
    }
}

/* Hand the first window byte out as text. Returns 1 when it completes a line. */
static int drop_head(hipnuc_decoder_t *d)
{
    uint8_t b = d->win[d->lo];
    d->lo++;
    d->n--;
    if (d->n == 0) d->lo = 0;
    return text_out(d, b);
}

static int state_ok(const hipnuc_decoder_t *d)
{
    return (size_t)d->lo + d->n <= HIPNUC_FRAME_MAX && d->line_len <= HIPNUC_LINE_MAX;
}

void hipnuc_decoder_init(hipnuc_decoder_t *dec)
{
    if (dec) memset(dec, 0, sizeof(*dec));
}

int hipnuc_decoder_feed(hipnuc_decoder_t *d, const uint8_t **buf, size_t *len, hipnuc_data_t *out)
{
    const uint8_t *in = NULL;
    size_t left = 0;
    int ev = 0;

    if (!d) return 0;
    if (buf && len && *buf) {
        in = *buf;
        left = *len;
    }
    if (!state_ok(d)) {   /* never initialised or overwritten: start over */
        d->lo = d->n = 0;
        d->span_open = d->taint_open = 0;
        drop_line(d);
    }
    if (left > 0) d->give_up = 0;   /* bytes arrive again: the flush no longer applies */

    for (;;) {
        const uint8_t *h;
        size_t total = 0;
        int r;

        if (d->n == 0) {   /* nothing pending: the next input byte */
            uint8_t b;
            d->lo = 0;
            if (left == 0) break;
            b = *in++;
            left--;
            d->bytes++;
            if (b == SYNC1) {
                d->win[0] = b;
                d->n = 1;
                continue;
            }
            if (text_out(d, b)) {
                ev = HIPNUC_EV_LINE;
                break;
            }
            continue;
        }

        h = d->win + d->lo;
        r = peek(h, d->n, &total, d->span_open);
        if (r == PEEK_SKIP) {
            if (!d->span_open && d->n >= 4 && h[0] == SYNC1 && h[1] == SYNC2) {   /* a sync with an impossible length */
                d->crc_errors++;
                reject(d, HEADER, 0);
            }
            if (drop_head(d)) {
                ev = HIPNUC_EV_LINE;
                break;
            }
            continue;
        }
        if (r == PEEK_OK && d->n >= total) {   /* a complete candidate */
            if (!crc_ok(h, total)) {
                size_t ignored;
                d->crc_errors++;
                reject(d, total, peek(h, d->n, &ignored, 1) == PEEK_OK);
                if (drop_head(d)) {
                    ev = HIPNUC_EV_LINE;
                    break;
                }
                continue;
            }
            r = decode_payload(h + HEADER, total - HEADER, out);
            drop_line(d);   /* a frame ends whatever text came before it */
            d->lo = (uint16_t)(d->lo + total);
            d->n = (uint16_t)(d->n - total);
            if (d->n == 0) d->lo = 0;
            (void)advance(d, (uint32_t)total);
            if (r) {
                d->frames++;
                d->frame_size = (uint16_t)total;
                ev = HIPNUC_EV_DATA;
                break;
            }
            d->unsupported++;
            continue;
        }

        /* incomplete: the candidate needs more bytes */
        if (left == 0) {
            if (!d->give_up) break;
            if (r == PEEK_OK) d->crc_errors++;   /* flush: a frame whose rest will not come */
            reject(d, d->n, 0);
            if (drop_head(d)) {
                ev = HIPNUC_EV_LINE;
                break;
            }
            continue;
        }
        if ((size_t)d->lo + d->n >= HIPNUC_FRAME_MAX) {
            memmove(d->win, h, d->n);
            d->lo = 0;
        }
        d->win[d->lo + d->n] = *in++;
        d->n++;
        left--;
        d->bytes++;
    }

    if (buf && len && *buf) {
        *buf = in;
        *len = left;
    }
    return ev;
}

const char *hipnuc_decoder_line(const hipnuc_decoder_t *dec)
{
    return dec ? dec->line : "";
}

void hipnuc_decoder_flush(hipnuc_decoder_t *dec)
{
    if (dec && dec->n > 0) dec->give_up = 1;
}

size_t hipnuc_decoder_backlog(const hipnuc_decoder_t *dec)
{
    return dec && state_ok(dec) ? dec->n : 0;
}

int hipnuc_frame_decode(const uint8_t *frame, size_t len, hipnuc_data_t *out)
{
    size_t total = 0;
    if (!frame || len < HEADER || peek(frame, len, &total, 0) != PEEK_OK || total != len) return -1;
    if (!crc_ok(frame, total)) return -2;
    return decode_payload(frame + HEADER, total - HEADER, out) ? 1 : -3;
}

const char *hipnuc_line_value(const char *line, const char *key, char *value, size_t size)
{
    size_t klen;
    const char *p;
    if (!line || !key || !value || size == 0 || (klen = strlen(key)) == 0) return NULL;
    for (p = strstr(line, key); p; p = strstr(p + 1, key)) {
        if ((p == line || p[-1] == ' ' || p[-1] == ',') && p[klen] == '=') {
            const char *v = p + klen + 1;
            size_t n = 0;
            while (v[n] && v[n] != ',') n++;
            if (n >= size) return NULL;
            memcpy(value, v, n);
            value[n] = '\0';
            return value;
        }
    }
    return NULL;
}
