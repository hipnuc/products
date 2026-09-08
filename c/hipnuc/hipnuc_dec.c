/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC serial binary protocol decoder. See hipnuc_dec.h.
 */

#include "hipnuc_dec.h"

#include <string.h>

/* Wire sizes of the fixed packets; the structures must pack to the same. */
#define HI91_WIRE_SIZE 76
#define HI81_WIRE_SIZE 104
typedef char hipnuc_assert_hi91[(sizeof(hi91_t) == HI91_WIRE_SIZE) ? 1 : -1];
typedef char hipnuc_assert_hi81[(sizeof(hi81_t) == HI81_WIRE_SIZE) ? 1 : -1];
typedef char hipnuc_assert_float[(sizeof(float) == 4) ? 1 : -1];

/* Little-endian field readers; safe for any alignment. */
static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t rd_u64(const uint8_t *p)
{
    return (uint64_t)rd_u32(p) | ((uint64_t)rd_u32(p + 4) << 32);
}

static float rd_f32(const uint8_t *p)
{
    float f;
    uint32_t u = rd_u32(p);
    memcpy(&f, &u, sizeof(f));
    return f;
}

static double rd_f64(const uint8_t *p)
{
    double d = 0;
    uint64_t u = rd_u64(p);
    /* Frames with binary64 fields are rejected earlier when double is not 8 bytes. */
    if (sizeof(d) == 8) memcpy(&d, &u, sizeof(d));
    return d;
}

/* Element-wise assignment: taking the address of a packed member is not
 * portable, so arrays are filled through this macro instead of a pointer. */
#define RD_F32_ARRAY(dst, src, count) do { \
    int rd_i_; \
    for (rd_i_ = 0; rd_i_ < (count); ++rd_i_) \
        (dst)[rd_i_] = rd_f32((src) + 4 * rd_i_); \
} while (0)

uint16_t hipnuc_crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    size_t j;
    for (j = 0; j < len; ++j) {
        int i;
        crc ^= (uint16_t)((uint16_t)data[j] << 8);
        for (i = 0; i < 8; ++i) {
            unsigned int next = (unsigned int)crc << 1;
            if (crc & 0x8000U) next ^= 0x1021U;
            crc = (uint16_t)next;
        }
    }
    return crc;
}

static void clear_packet_tags(hipnuc_raw_t *raw)
{
    raw->hi91.tag = 0;
    raw->hi81.tag = 0;
    raw->hi83.tag = 0;
}

/* Byte length of every decodable HI83 field, indexed by bitmap bit. Bits
 * without a known length reject the frame; offsets are never guessed. */
static int hi83_packet_size(uint32_t bitmap)
{
    static const uint8_t field_sizes[32] = {
        12, 12, 12, 12, 16, 8, 8, 4, 4, 12, 12, 12, 12, 12, 24, 4,
        4, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 12
    };
    int size = 8;
    unsigned bit;

    if (bitmap & ~HI83_BMAP_SUPPORTED) return -1;
    if (sizeof(double) != 8 && (bitmap & (HI83_BMAP_INS_LON_LAT_MSL | HI83_BMAP_GNSS_LON_LAT_MSL))) {
        return -1;
    }
    for (bit = 0; bit < 32; ++bit) {
        if (bitmap & (UINT32_C(1) << bit)) {
            if (!field_sizes[bit]) return -1;
            size += field_sizes[bit];
        }
    }
    return size;
}

static int16_t rd_i16(const uint8_t *p) { return (int16_t)rd_u16(p); }
static int32_t rd_i32(const uint8_t *p) { return (int32_t)rd_u32(p); }

/* Field-by-field decoding keeps the decoder independent of host byte order. */
static void parse_hi91(hi91_t *o, const uint8_t *p)
{
    o->tag = p[0];
    o->main_status = rd_u16(p + 1);
    o->temp = (int8_t)p[3];
    o->air_pressure = rd_f32(p + 4);
    o->system_time = rd_u32(p + 8);
    RD_F32_ARRAY(o->acc, p + 12, 3);
    RD_F32_ARRAY(o->gyr, p + 24, 3);
    RD_F32_ARRAY(o->mag, p + 36, 3);
    o->roll = rd_f32(p + 48);
    o->pitch = rd_f32(p + 52);
    o->yaw = rd_f32(p + 56);
    RD_F32_ARRAY(o->quat, p + 60, 4);
}

static void parse_hi81(hi81_t *o, const uint8_t *p)
{
    int i;
    o->tag = p[0];
    o->main_status = rd_u16(p + 1);
    o->ins_status = p[3];
    o->gpst_wn = rd_u16(p + 4);
    o->gpst_tow = rd_u32(p + 6);
    o->reserved0 = rd_u16(p + 10);
    for (i = 0; i < 3; ++i) {
        o->gyr_b[i] = rd_i16(p + 12 + 2 * i);
        o->acc_b[i] = rd_i16(p + 18 + 2 * i);
        o->mag_b[i] = rd_i16(p + 24 + 2 * i);
    }
    o->air_pressure = rd_i16(p + 30);
    o->od_speed = rd_i16(p + 32);
    o->temperature = (int8_t)p[34];
    o->utc_year = p[35];
    o->utc_month = p[36];
    o->utc_day = p[37];
    o->utc_hour = p[38];
    o->utc_min = p[39];
    o->utc_msec = rd_u16(p + 40);
    o->roll = rd_i16(p + 42);
    o->pitch = rd_i16(p + 44);
    o->yaw = rd_u16(p + 46);
    for (i = 0; i < 4; ++i) o->quat[i] = rd_i16(p + 48 + 2 * i);
    o->ins_lon = rd_i32(p + 56);
    o->ins_lat = rd_i32(p + 60);
    o->ins_msl = rd_i32(p + 64);
    o->pdop = p[68];
    o->hdop = p[69];
    o->solq_pos = p[70];
    o->nv_pos = p[71];
    o->solq_heading = p[72];
    o->nv_heading = p[73];
    o->diff_age = p[74];
    o->undulation = rd_i16(p + 75);
    o->ant_status = p[77];
    for (i = 0; i < 3; ++i) {
        o->vel_enu[i] = rd_i16(p + 78 + 2 * i);
        o->acc_enu[i] = rd_i16(p + 84 + 2 * i);
    }
    memcpy(o->reserved_tail, p + 90, sizeof(o->reserved_tail));
}

/* Decode one HI83 sub-packet at p (bitmap already validated). */
static int parse_hi83(hi83_t *out, const uint8_t *p, int avail)
{
    uint32_t bm = rd_u32(p + 4);
    int size = hi83_packet_size(bm);
    int idx = 8;

    if (size < 0 || avail != size) return -1;

    memset(out, 0, sizeof(*out));
    out->tag = HIPNUC_ID_HI83;
    out->main_status = rd_u16(p + 1);
    out->ins_status = p[3];
    out->data_bitmap = bm;

    /* Fields follow in ascending bit order for every supported bit. */
    if (bm & HI83_BMAP_ACC_B)       { RD_F32_ARRAY(out->acc_b, p + idx, 3);       idx += 12; }
    if (bm & HI83_BMAP_GYR_B)       { RD_F32_ARRAY(out->gyr_b, p + idx, 3);       idx += 12; }
    if (bm & HI83_BMAP_MAG_B)       { RD_F32_ARRAY(out->mag_b, p + idx, 3);       idx += 12; }
    if (bm & HI83_BMAP_RPY)         { RD_F32_ARRAY(out->rpy, p + idx, 3);         idx += 12; }
    if (bm & HI83_BMAP_QUAT)        { RD_F32_ARRAY(out->quat, p + idx, 4);        idx += 16; }
    if (bm & HI83_BMAP_SYSTEM_TIME) {
        out->system_time_us = rd_u64(p + idx);
        idx += 8;
    }
    if (bm & HI83_BMAP_UTC) {
        out->utc.year = p[idx + 0];
        out->utc.month = p[idx + 1];
        out->utc.day = p[idx + 2];
        out->utc.hour = p[idx + 3];
        out->utc.min = p[idx + 4];
        out->utc.sec_ms = rd_u16(p + idx + 5);
        out->utc.rev = p[idx + 7];
        idx += 8;
    }
    if (bm & HI83_BMAP_AIR_PRESSURE) {
        out->air_pressure = rd_f32(p + idx);
          idx += 4;
    }
    if (bm & HI83_BMAP_TEMPERATURE)  { out->temperature = rd_f32(p + idx);    idx += 4; }
    if (bm & HI83_BMAP_INCLINATION)  { RD_F32_ARRAY(out->inclination, p + idx, 3); idx += 12; }
    if (bm & HI83_BMAP_HSS)          { RD_F32_ARRAY(out->hss, p + idx, 3);         idx += 12; }
    if (bm & HI83_BMAP_HSS_FRQ)      { RD_F32_ARRAY(out->hss_frq, p + idx, 3);     idx += 12; }
    if (bm & HI83_BMAP_VEL_ENU)      { RD_F32_ARRAY(out->vel_enu, p + idx, 3);     idx += 12; }
    if (bm & HI83_BMAP_ACC_ENU)      { RD_F32_ARRAY(out->acc_enu, p + idx, 3);     idx += 12; }
    if (bm & HI83_BMAP_INS_LON_LAT_MSL) {
        out->ins_lon_lat_msl[0] = rd_f64(p + idx);
        out->ins_lon_lat_msl[1] = rd_f64(p + idx + 8);
        out->ins_lon_lat_msl[2] = rd_f64(p + idx + 16);
        idx += 24;
    }
    if (bm & HI83_BMAP_GNSS_QUALITY_NV) {
        out->solq_pos = p[idx + 0];
        out->nv_pos = p[idx + 1];
        out->solq_heading = p[idx + 2];
        out->nv_heading = p[idx + 3];
        idx += 4;
    }
    if (bm & HI83_BMAP_OD_SPEED)   { out->od_speed = rd_f32(p + idx);   idx += 4; }
    if (bm & HI83_BMAP_UNDULATION) {
        out->undulation = rd_f32(p + idx);
        idx += 4;
    }
    if (bm & HI83_BMAP_DIFF_AGE)   { out->diff_age = rd_f32(p + idx);   idx += 4; }
    if (bm & HI83_BMAP_NODE_ID) {
        out->node.node_id = p[idx + 0];
        out->node.reserved[0] = p[idx + 1];
        out->node.reserved[1] = p[idx + 2];
        out->node.reserved[2] = p[idx + 3];
        idx += 4;
    }
    if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) {
        out->gnss_lon_lat_msl[0] = rd_f64(p + idx);
        out->gnss_lon_lat_msl[1] = rd_f64(p + idx + 8);
        out->gnss_lon_lat_msl[2] = rd_f64(p + idx + 16);
        idx += 24;
    }
    if (bm & HI83_BMAP_GNSS_VEL) {
        RD_F32_ARRAY(out->gnss_vel, p + idx, 3);
        idx += 12;
    }

    return idx;
}

/* A supported product frame contains exactly one sub-packet. Validate its
 * full length before exposing anything; never choose one of several tags. */
static int parse_payload(hipnuc_raw_t *raw)
{
    const uint8_t *p = &raw->buf[HIPNUC_HEADER_SIZE];

    clear_packet_tags(raw);
    if (raw->len == 0) goto invalid;
    switch (p[0]) {
    case HIPNUC_ID_HI91:
        if (raw->len != HI91_WIRE_SIZE) goto invalid;
        parse_hi91(&raw->hi91, p);
        break;
    case HIPNUC_ID_HI81:
        if (raw->len != HI81_WIRE_SIZE) goto invalid;
        parse_hi81(&raw->hi81, p);
        break;
    case HIPNUC_ID_HI83:
        if (raw->len < 8 || parse_hi83(&raw->hi83, p, raw->len) < 0) goto invalid;
        break;
    default:
        goto invalid;
    }
    raw->frame_count++;
    return 1;

invalid:
    clear_packet_tags(raw);
    raw->invalid_count++;
    return -1;
}

/* A damaged length/CRC can have consumed the start of the next frame. Keep
 * an unfinished candidate, or a final first sync byte, for subsequent input.
 * Complete candidates inside the damaged span are discarded: the byte API
 * has no queue for returning several already-consumed frames. */
static void recover_prefix(hipnuc_raw_t *raw)
{
    int start, size = raw->nbyte;
    raw->nbyte = 0;
    for (start = 1; start + 1 < size; ++start) {
        int available, length = 0;
        if (raw->buf[start] != HIPNUC_SYNC1 || raw->buf[start + 1] != HIPNUC_SYNC2) continue;
        available = size - start;
        if (available >= 4) {
            length = rd_u16(raw->buf + start + 2);
            if (length == 0 || length > HIPNUC_MAX_PAYLOAD_SIZE) continue;
            if (available >= length + HIPNUC_HEADER_SIZE) {
                start += length + HIPNUC_HEADER_SIZE - 1;
                continue;
            }
        }
        memmove(raw->buf, raw->buf + start, (size_t)available);
        raw->nbyte = available;
        raw->len = length;
        return;
    }
    raw->buf[1] = start < size && raw->buf[size - 1] == HIPNUC_SYNC1 ? HIPNUC_SYNC1 : 0;
}

static int decode_frame(hipnuc_raw_t *raw)
{
    uint16_t crc = hipnuc_crc16(0, raw->buf, 4);
    crc = hipnuc_crc16(crc, raw->buf + HIPNUC_HEADER_SIZE, (size_t)raw->len);
    if (crc != rd_u16(raw->buf + 4)) {
        clear_packet_tags(raw);
        raw->crc_error_count++;
        recover_prefix(raw);
        return -1;
    }
    raw->nbyte = 0;
    {
        int result = parse_payload(raw);
        /* A checksummed envelope is consumed whole, even if unsupported. */
        raw->buf[1] = 0;
        return result;
    }
}

int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    if (raw->nbyte == 0) {
        /* Two-byte sliding sync */
        raw->buf[0] = raw->buf[1];
        raw->buf[1] = data;
        if (raw->buf[0] == HIPNUC_SYNC1 && raw->buf[1] == HIPNUC_SYNC2) {
            raw->nbyte = 2;
        }
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == HIPNUC_HEADER_SIZE) {
        uint16_t payload_len = rd_u16(raw->buf + 2);
        if (payload_len == 0 || payload_len > HIPNUC_MAX_PAYLOAD_SIZE) {
            clear_packet_tags(raw);
            raw->invalid_count++;
            recover_prefix(raw);
            return -1;
        }
        raw->len = (int)payload_len;
    }

    if (raw->nbyte < HIPNUC_HEADER_SIZE || raw->nbyte < raw->len + HIPNUC_HEADER_SIZE) {
        return 0;
    }

    return decode_frame(raw);
}

int hipnuc_input_buffer(hipnuc_raw_t *raw, const uint8_t *data, size_t len, size_t *consumed)
{
    size_t i;
    for (i = 0; i < len; ++i) {
        int ret = hipnuc_input(raw, data[i]);
        if (ret != 0) {
            if (consumed) *consumed = i + 1;
            return ret;
        }
    }
    if (consumed) *consumed = len;
    return 0;
}

/* Wire packet to SI sample conversion. */
static void set_utc(hipnuc_sample_t *s, uint8_t year2, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint16_t sec_ms, int synchronized)
{
    if (!synchronized || (year2 == 0 && month == 0 && day == 0)) return;
    s->utc.year = (uint16_t)(2000 + year2);
    s->utc.month = month;
    s->utc.day = day;
    s->utc.hour = hour;
    s->utc.minute = minute;
    s->utc.second = (uint8_t)(sec_ms / 1000);
    s->utc.millisecond = (uint16_t)(sec_ms % 1000);
    if (hipnuc_utc_is_valid(&s->utc)) s->valid |= HIPNUC_VALID_UTC | HIPNUC_VALID_UTC_TIME_OF_DAY;
}

void hipnuc_sample_from_hi91(const hi91_t *p, hipnuc_sample_t *s)
{
    int i;
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_HI91;
    hipnuc_sample_set_status(s, p->main_status);
    for (i = 0; i < 3; ++i) {
        s->acc[i] = p->acc[i] * HIPNUC_GRAVITY;
        s->gyr[i] = p->gyr[i] * HIPNUC_DEG2RAD;
        s->mag[i] = p->mag[i] * 1e-6f;
    }
    s->roll = p->roll * HIPNUC_DEG2RAD;
    s->pitch = p->pitch * HIPNUC_DEG2RAD;
    s->yaw = p->yaw * HIPNUC_DEG2RAD;
    for (i = 0; i < 4; ++i) s->quat[i] = p->quat[i];
    s->temperature = (float)p->temp;
    s->pressure = p->air_pressure;
    s->device_time_us = (uint64_t)p->system_time * 1000U;
    s->valid |= HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG | HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_YAW |
                HIPNUC_VALID_QUAT | HIPNUC_VALID_TEMPERATURE | HIPNUC_VALID_PRESSURE |
                HIPNUC_VALID_DEVICE_TIME;
}

void hipnuc_sample_from_hi81(const hi81_t *p, hipnuc_sample_t *s)
{
    int i;
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_HI81;
    hipnuc_sample_set_status(s, p->main_status);
    s->ins_status = p->ins_status;
    for (i = 0; i < 3; ++i) {
        s->gyr[i] = p->gyr_b[i] * 0.001f;
        s->acc[i] = p->acc_b[i] * 0.0048828f;
        s->mag[i] = p->mag_b[i] * 0.030517e-6f;
        s->vel_enu[i] = p->vel_enu[i] * 0.01f;
        s->acc_enu[i] = p->acc_enu[i] * 0.0048828f;
    }
    s->pressure = (float)p->air_pressure + 100000.0f;
    s->temperature = (float)p->temperature;
    s->odometer_speed = p->od_speed * 0.01f;
    s->roll = p->roll * 0.01f * HIPNUC_DEG2RAD;
    s->pitch = p->pitch * 0.01f * HIPNUC_DEG2RAD;
    s->heading = p->yaw * 0.01f * HIPNUC_DEG2RAD;
    for (i = 0; i < 4; ++i) s->quat[i] = p->quat[i] * 0.0001f;
    s->longitude = p->ins_lon * 1e-7;
    s->latitude = p->ins_lat * 1e-7;
    s->altitude_msl = p->ins_msl * 0.001;
    s->pdop = p->pdop * 0.1f;
    s->hdop = p->hdop * 0.1f;
    s->position_quality = p->solq_pos;
    s->position_satellites = p->nv_pos;
    s->heading_quality = p->solq_heading;
    s->heading_satellites = p->nv_heading;
    s->diff_age = (float)p->diff_age;
    s->undulation = p->undulation * 0.01f;
    s->gps_week = p->gpst_wn;
    s->gps_tow_ms = p->gpst_tow;
    set_utc(s, p->utc_year, p->utc_month, p->utc_day, p->utc_hour, p->utc_min, p->utc_msec,
            !(p->main_status & HIPNUC_STATUS_UTC_UNSYNC));
    s->valid |= HIPNUC_VALID_INS_STATUS | HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG |
                HIPNUC_VALID_PRESSURE | HIPNUC_VALID_TEMPERATURE | HIPNUC_VALID_ODOMETER |
                HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_HEADING | HIPNUC_VALID_QUAT |
                HIPNUC_VALID_POSITION | HIPNUC_VALID_ALTITUDE | HIPNUC_VALID_PDOP | HIPNUC_VALID_HDOP |
                HIPNUC_VALID_POSITION_QUALITY | HIPNUC_VALID_HEADING_QUALITY |
                HIPNUC_VALID_POSITION_SATELLITES | HIPNUC_VALID_HEADING_SATELLITES |
                HIPNUC_VALID_DIFF_AGE | HIPNUC_VALID_UNDULATION | HIPNUC_VALID_VELOCITY_ENU |
                HIPNUC_VALID_ACC_ENU;
    if (p->gpst_wn || p->gpst_tow) s->valid |= HIPNUC_VALID_GPS_TIME;
}

void hipnuc_sample_from_hi83(const hi83_t *p, hipnuc_sample_t *s)
{
    uint32_t bm = p->data_bitmap;
    int i;
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_HI83;
    hipnuc_sample_set_status(s, p->main_status);
    s->ins_status = p->ins_status;
    s->valid |= HIPNUC_VALID_INS_STATUS;

    if (bm & HI83_BMAP_ACC_B) {
        for (i = 0; i < 3; ++i) s->acc[i] = p->acc_b[i];
        s->valid |= HIPNUC_VALID_ACC;
    }
    if (bm & HI83_BMAP_GYR_B) {
        for (i = 0; i < 3; ++i) s->gyr[i] = p->gyr_b[i];
        s->valid |= HIPNUC_VALID_GYR;
    }
    if (bm & HI83_BMAP_MAG_B) {
        for (i = 0; i < 3; ++i) s->mag[i] = p->mag_b[i] * 1e-6f;
        s->valid |= HIPNUC_VALID_MAG;
    }
    if (bm & HI83_BMAP_RPY) {
        s->roll = p->rpy[0] * HIPNUC_DEG2RAD;
        s->pitch = p->rpy[1] * HIPNUC_DEG2RAD;
        s->yaw = p->rpy[2] * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_YAW;
    }
    if (bm & HI83_BMAP_QUAT) {
        for (i = 0; i < 4; ++i) s->quat[i] = p->quat[i];
        s->valid |= HIPNUC_VALID_QUAT;
    }
    if (bm & HI83_BMAP_SYSTEM_TIME) {
        s->device_time_us = p->system_time_us;
        s->valid |= HIPNUC_VALID_DEVICE_TIME;
    }
    if (bm & HI83_BMAP_UTC) {
        set_utc(s, p->utc.year, p->utc.month, p->utc.day, p->utc.hour, p->utc.min, p->utc.sec_ms,
                !(p->main_status & HIPNUC_STATUS_UTC_UNSYNC));
    }
    if (bm & HI83_BMAP_AIR_PRESSURE) {
        s->pressure = p->air_pressure;
        s->valid |= HIPNUC_VALID_PRESSURE;
    }
    if (bm & HI83_BMAP_TEMPERATURE) {
        s->temperature = p->temperature;
        s->valid |= HIPNUC_VALID_TEMPERATURE;
    }
    if (bm & HI83_BMAP_INCLINATION) {
        s->inclination[0] = p->inclination[0] * HIPNUC_DEG2RAD;
        s->inclination[1] = p->inclination[1] * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_INCLINATION;
    }
    if (bm & HI83_BMAP_HSS) {
        for (i = 0; i < 3; ++i) s->heave_m[i] = p->hss[i];
        s->valid |= HIPNUC_VALID_HEAVE;
    }
    if (bm & HI83_BMAP_HSS_FRQ) {
        for (i = 0; i < 3; ++i) s->heave_hz[i] = p->hss_frq[i];
        s->valid |= HIPNUC_VALID_HEAVE_FREQUENCY;
    }
    if (bm & HI83_BMAP_VEL_ENU) {
        for (i = 0; i < 3; ++i) s->vel_enu[i] = p->vel_enu[i];
        s->valid |= HIPNUC_VALID_VELOCITY_ENU;
    }
    if (bm & HI83_BMAP_ACC_ENU) {
        for (i = 0; i < 3; ++i) s->acc_enu[i] = p->acc_enu[i];
        s->valid |= HIPNUC_VALID_ACC_ENU;
    }
    if (bm & HI83_BMAP_INS_LON_LAT_MSL) {
        s->longitude = p->ins_lon_lat_msl[0];
        s->latitude = p->ins_lon_lat_msl[1];
        s->altitude_msl = p->ins_lon_lat_msl[2];
        s->valid |= HIPNUC_VALID_POSITION | HIPNUC_VALID_ALTITUDE;
    }
    if (bm & HI83_BMAP_GNSS_QUALITY_NV) {
        s->position_quality = p->solq_pos;
        s->position_satellites = p->nv_pos;
        s->heading_quality = p->solq_heading;
        s->heading_satellites = p->nv_heading;
        s->valid |= HIPNUC_VALID_POSITION_QUALITY | HIPNUC_VALID_HEADING_QUALITY |
                    HIPNUC_VALID_POSITION_SATELLITES | HIPNUC_VALID_HEADING_SATELLITES;
    }
    if (bm & HI83_BMAP_OD_SPEED) {
        s->odometer_speed = p->od_speed;
        s->valid |= HIPNUC_VALID_ODOMETER;
    }
    if (bm & HI83_BMAP_UNDULATION) {
        s->undulation = p->undulation;
        s->valid |= HIPNUC_VALID_UNDULATION;
    }
    if (bm & HI83_BMAP_DIFF_AGE) {
        s->diff_age = p->diff_age;
        s->valid |= HIPNUC_VALID_DIFF_AGE;
    }
    if (bm & HI83_BMAP_NODE_ID) {
        s->node_id = p->node.node_id;
        s->valid |= HIPNUC_VALID_NODE_ID;
    }
    if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) {
        s->gnss_longitude = p->gnss_lon_lat_msl[0];
        s->gnss_latitude = p->gnss_lon_lat_msl[1];
        s->gnss_altitude_msl = p->gnss_lon_lat_msl[2];
        s->valid |= HIPNUC_VALID_GNSS_POSITION | HIPNUC_VALID_GNSS_ALTITUDE;
    }
    if (bm & HI83_BMAP_GNSS_VEL) {
        for (i = 0; i < 3; ++i) s->gnss_vel_enu[i] = p->gnss_vel[i];
        s->valid |= HIPNUC_VALID_GNSS_VELOCITY;
    }
}

int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *s)
{
    int count = (raw->hi83.tag == HIPNUC_ID_HI83) + (raw->hi81.tag == HIPNUC_ID_HI81) +
                (raw->hi91.tag == HIPNUC_ID_HI91);
    hipnuc_sample_clear(s);
    if (count != 1) return 0;
    if (raw->hi83.tag == HIPNUC_ID_HI83) { hipnuc_sample_from_hi83(&raw->hi83, s); return 1; }
    if (raw->hi81.tag == HIPNUC_ID_HI81) { hipnuc_sample_from_hi81(&raw->hi81, s); return 1; }
    if (raw->hi91.tag == HIPNUC_ID_HI91) { hipnuc_sample_from_hi91(&raw->hi91, s); return 1; }
    return 0;
}
