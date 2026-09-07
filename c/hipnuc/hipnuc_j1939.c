/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC CAN protocol decoder: J1939 data PGNs, the CANFD83 frame and the
 * register access frames. See hipnuc_j1939.h.
 *
 * All multi-byte wire fields are little-endian and are read with byte
 * arithmetic, so the decoder does not care about host endianness or the
 * alignment of the frame buffer.
 */

#include "hipnuc_j1939.h"

#include <string.h>

/* ------------------------------------------------------------------------- */
/* Little-endian field access                                                */
/* ------------------------------------------------------------------------- */

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t rd_i16(const uint8_t *p)
{
    return (int16_t)rd_u16(p);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

static uint64_t rd_u64(const uint8_t *p)
{
    return (uint64_t)rd_u32(p) | ((uint64_t)rd_u32(p + 4) << 32);
}

static float rd_f32(const uint8_t *p)
{
    uint32_t u = rd_u32(p);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

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

/* ------------------------------------------------------------------------- */
/* Identifier helpers                                                        */
/* ------------------------------------------------------------------------- */

uint32_t hipnuc_j1939_pgn(uint32_t id)
{
    return (id >> 8) & 0xFFFFU;
}

uint8_t hipnuc_j1939_source_address(uint32_t id)
{
    return (uint8_t)(id & 0xFFU);
}

uint32_t hipnuc_j1939_data_id(uint32_t pgn, uint8_t source)
{
    return (HIPNUC_J1939_PRIORITY << 26) | ((pgn & 0xFFFFU) << 8) | (uint32_t)source;
}

/* PDU1 frame: PF 0xEF, PS = destination address. Priority 3, no data page. */
static uint32_t config_id(uint8_t dest, uint8_t source)
{
    return (HIPNUC_J1939_PRIORITY << 26) | (HIPNUC_J1939_PGN_CONFIG << 8) | ((uint32_t)dest << 8) | (uint32_t)source;
}

/* ------------------------------------------------------------------------- */
/* Wire scale factors                                                        */
/* ------------------------------------------------------------------------- */

#define J1939_ACC_SCALE   ((16.0f / 32768.0f) * HIPNUC_GRAVITY)          /* LSB 1/2048 G -> m/s^2 */
#define J1939_GYR_SCALE   ((2000.0f / 32768.0f) * HIPNUC_DEG2RAD)        /* LSB 2000/32768 deg/s -> rad/s */
#define J1939_MAG_SCALE   ((1000.0f / 32768.0f) * 1e-6f)                 /* LSB 1000/32768 uT -> T */
#define J1939_MDEG2RAD    (0.001f * HIPNUC_DEG2RAD)                       /* x0.001 deg -> rad */

/* Payload sizes of the CANFD83 fields, indexed by bitmap bit */
#define CANFD83_SIZE_ACC          12
#define CANFD83_SIZE_GYR          12
#define CANFD83_SIZE_MAG          12
#define CANFD83_SIZE_RPY          12
#define CANFD83_SIZE_QUAT         16
#define CANFD83_SIZE_SYSTEM_TIME  8
#define CANFD83_SIZE_UTC          8
#define CANFD83_SIZE_TEMPERATURE  4

/* ------------------------------------------------------------------------- */
/* Data PGNs                                                                 */
/* ------------------------------------------------------------------------- */

/* Message type and minimum payload for a PGN; NONE when it is not HiPNUC data. */
static hipnuc_j1939_msg_t lookup_pgn(uint32_t pgn, uint8_t *min_len)
{
    switch (pgn) {
        case HIPNUC_J1939_PGN_ACC:         *min_len = 6; return HIPNUC_J1939_MSG_ACC;
        case HIPNUC_J1939_PGN_GYR:         *min_len = 6; return HIPNUC_J1939_MSG_GYR;
        case HIPNUC_J1939_PGN_MAG:         *min_len = 6; return HIPNUC_J1939_MSG_MAG;
        case HIPNUC_J1939_PGN_ROLL_PITCH:  *min_len = 8; return HIPNUC_J1939_MSG_ROLL_PITCH;
        case HIPNUC_J1939_PGN_YAW:         *min_len = 4; return HIPNUC_J1939_MSG_YAW;
        case HIPNUC_J1939_PGN_TEMP:        *min_len = 2; return HIPNUC_J1939_MSG_TEMP;
        case HIPNUC_J1939_PGN_QUAT:        *min_len = 8; return HIPNUC_J1939_MSG_QUAT;
        case HIPNUC_J1939_PGN_INCLINATION: *min_len = 8; return HIPNUC_J1939_MSG_INCLINATION;
        case HIPNUC_J1939_PGN_TIME:        *min_len = 8; return HIPNUC_J1939_MSG_TIME;
        case HIPNUC_J1939_PGN_POSITION:    *min_len = 8; return HIPNUC_J1939_MSG_POSITION;
        case HIPNUC_J1939_PGN_ALTITUDE:    *min_len = 8; return HIPNUC_J1939_MSG_ALTITUDE;
        case HIPNUC_J1939_PGN_GNSS_STATUS: *min_len = 5; return HIPNUC_J1939_MSG_GNSS_STATUS;
        case HIPNUC_J1939_PGN_VELOCITY:    *min_len = 6; return HIPNUC_J1939_MSG_VELOCITY;
        case HIPNUC_J1939_PGN_CANFD83:     *min_len = CANFD83_HEADER_SIZE; return HIPNUC_J1939_MSG_CANFD83;
        default:                           *min_len = 0; return HIPNUC_J1939_MSG_NONE;
    }
}

/*
 * UTC fields shared by the TIME PGN and CANFD83. The time of day is always
 * stored; HIPNUC_VALID_UTC is set only for a synchronized calendar time.
 * With no GNSS fix the firmware sends its uptime as h/m/s with a zero date,
 * which must not be mistaken for UTC.
 */
static void set_utc(hipnuc_sample_t *s, uint8_t year2, uint8_t month, uint8_t day,
                    uint8_t hour, uint8_t minute, uint8_t second, uint16_t ms, int synchronized)
{
    s->utc.year = (uint16_t)(2000 + year2);
    s->utc.month = month;
    s->utc.day = day;
    s->utc.hour = hour;
    s->utc.minute = minute;
    s->utc.second = second;
    s->utc.millisecond = ms;
    if (synchronized && (year2 != 0 || month != 0)) s->valid |= HIPNUC_VALID_UTC;
}

static int canfd83_logical_length(uint32_t bitmap)
{
    int n = CANFD83_HEADER_SIZE;
    if (bitmap & CANFD83_MAP_ACC_B)       n += CANFD83_SIZE_ACC;
    if (bitmap & CANFD83_MAP_GYR_B)       n += CANFD83_SIZE_GYR;
    if (bitmap & CANFD83_MAP_MAG_B)       n += CANFD83_SIZE_MAG;
    if (bitmap & CANFD83_MAP_RPY)         n += CANFD83_SIZE_RPY;
    if (bitmap & CANFD83_MAP_QUAT)        n += CANFD83_SIZE_QUAT;
    if (bitmap & CANFD83_MAP_SYSTEM_TIME) n += CANFD83_SIZE_SYSTEM_TIME;
    if (bitmap & CANFD83_MAP_UTC)         n += CANFD83_SIZE_UTC;
    if (bitmap & CANFD83_MAP_TEMPERATURE) n += CANFD83_SIZE_TEMPERATURE;
    return n;
}

/*
 * CANFD83: header (u32 bitmap, u16 main_status, u8 ins_status, u8 sequence)
 * followed by the selected fields in ascending bit order. CAN FD pads the
 * payload to a legal DLC, so the frame may be longer than the logical length.
 */
static int parse_canfd83(const hipnuc_can_frame_t *frame, hipnuc_sample_t *s, uint8_t *sequence)
{
    const uint8_t *d = frame->data;
    uint32_t bitmap = rd_u32(d);
    uint16_t main_status = rd_u16(d + 4);
    size_t pos = CANFD83_HEADER_SIZE;
    int i;

    if (bitmap == 0 || (bitmap & ~CANFD83_MAP_SUPPORTED) != 0) return -1;
    if ((int)frame->len < canfd83_logical_length(bitmap)) return -1;

    s->source = HIPNUC_SOURCE_CANFD83;
    hipnuc_sample_set_status(s, main_status);
    s->ins_status = d[6];
    s->valid |= HIPNUC_VALID_INS_STATUS;
    if (sequence) *sequence = d[7];

    if (bitmap & CANFD83_MAP_ACC_B) {
        for (i = 0; i < 3; ++i) s->acc[i] = rd_f32(d + pos + 4 * i);
        s->valid |= HIPNUC_VALID_ACC;
        pos += CANFD83_SIZE_ACC;
    }
    if (bitmap & CANFD83_MAP_GYR_B) {
        for (i = 0; i < 3; ++i) s->gyr[i] = rd_f32(d + pos + 4 * i);
        s->valid |= HIPNUC_VALID_GYR;
        pos += CANFD83_SIZE_GYR;
    }
    if (bitmap & CANFD83_MAP_MAG_B) {
        for (i = 0; i < 3; ++i) s->mag[i] = rd_f32(d + pos + 4 * i) * 1e-6f;
        s->valid |= HIPNUC_VALID_MAG;
        pos += CANFD83_SIZE_MAG;
    }
    if (bitmap & CANFD83_MAP_RPY) {
        s->roll = rd_f32(d + pos) * HIPNUC_DEG2RAD;
        s->pitch = rd_f32(d + pos + 4) * HIPNUC_DEG2RAD;
        s->yaw = rd_f32(d + pos + 8) * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_EULER;
        pos += CANFD83_SIZE_RPY;
    }
    if (bitmap & CANFD83_MAP_QUAT) {
        for (i = 0; i < 4; ++i) s->quat[i] = rd_f32(d + pos + 4 * i);
        s->valid |= HIPNUC_VALID_QUAT;
        pos += CANFD83_SIZE_QUAT;
    }
    if (bitmap & CANFD83_MAP_SYSTEM_TIME) {
        s->device_time_us = rd_u64(d + pos);
        s->valid |= HIPNUC_VALID_DEVICE_TIME;
        pos += CANFD83_SIZE_SYSTEM_TIME;
    }
    if (bitmap & CANFD83_MAP_UTC) {
        /* HI83 layout: year-2000, month, day, hour, min, u16 sec*1000+ms, reserved */
        uint16_t sec_ms = rd_u16(d + pos + 5);
        set_utc(s, d[pos], d[pos + 1], d[pos + 2], d[pos + 3], d[pos + 4],
                (uint8_t)(sec_ms / 1000), (uint16_t)(sec_ms % 1000),
                !(main_status & HIPNUC_STATUS_UTC_UNSYNC));
        pos += CANFD83_SIZE_UTC;
    }
    if (bitmap & CANFD83_MAP_TEMPERATURE) {
        s->temperature = rd_f32(d + pos);
        s->valid |= HIPNUC_VALID_TEMPERATURE;
        pos += CANFD83_SIZE_TEMPERATURE;
    }
    return HIPNUC_J1939_MSG_CANFD83;
}

int hipnuc_j1939_parse(const hipnuc_can_frame_t *frame, hipnuc_sample_t *sample, uint8_t *canfd83_sequence)
{
    const uint8_t *d;
    hipnuc_j1939_msg_t msg;
    uint8_t min_len = 0;
    int i;

    if (!frame || !sample) return -1;
    if (!frame->is_extended) return HIPNUC_J1939_MSG_NONE;   /* HiPNUC data is J1939 only */

    msg = lookup_pgn(hipnuc_j1939_pgn(frame->id), &min_len);
    if (msg == HIPNUC_J1939_MSG_NONE) return HIPNUC_J1939_MSG_NONE;   /* other PGNs, config frames */
    if (frame->is_remote || frame->is_error) return -1;
    if (frame->len < min_len) return -1;

    hipnuc_sample_clear(sample);
    sample->source = HIPNUC_SOURCE_J1939;
    sample->node_id = hipnuc_j1939_source_address(frame->id);
    sample->valid |= HIPNUC_VALID_NODE_ID;
    d = frame->data;

    switch (msg) {
        case HIPNUC_J1939_MSG_ACC:
            for (i = 0; i < 3; ++i) sample->acc[i] = rd_i16(d + 2 * i) * J1939_ACC_SCALE;
            sample->valid |= HIPNUC_VALID_ACC;
            break;

        case HIPNUC_J1939_MSG_GYR:
            for (i = 0; i < 3; ++i) sample->gyr[i] = rd_i16(d + 2 * i) * J1939_GYR_SCALE;
            sample->valid |= HIPNUC_VALID_GYR;
            break;

        case HIPNUC_J1939_MSG_MAG:
            for (i = 0; i < 3; ++i) sample->mag[i] = rd_i16(d + 2 * i) * J1939_MAG_SCALE;
            sample->valid |= HIPNUC_VALID_MAG;
            break;

        case HIPNUC_J1939_MSG_ROLL_PITCH:
            /*
             * Only roll and pitch travel in this PGN. HIPNUC_VALID_EULER is set
             * anyway so the two angles are usable on their own; yaw stays 0
             * until a YAW frame is merged (hipnuc_j1939_merge() keeps a yaw
             * that arrived earlier).
             */
            sample->roll = (float)rd_i32(d) * J1939_MDEG2RAD;
            sample->pitch = (float)rd_i32(d + 4) * J1939_MDEG2RAD;
            sample->valid |= HIPNUC_VALID_EULER;
            break;

        case HIPNUC_J1939_MSG_YAW:
            /*
             * First i32: heading, clockwise 0..360 deg. Second i32 (when
             * present): yaw in the device Euler convention (counter-clockwise).
             * HIPNUC_VALID_HEADING covers both; HIPNUC_VALID_EULER belongs to
             * the ROLL_PITCH frame.
             */
            sample->heading = (float)rd_i32(d) * J1939_MDEG2RAD;
            if (frame->len >= 8) sample->yaw = (float)rd_i32(d + 4) * J1939_MDEG2RAD;
            sample->valid |= HIPNUC_VALID_HEADING;
            break;

        case HIPNUC_J1939_MSG_TEMP:
            /* Bytes 4..7 are a reserved placeholder (999), not a pressure. */
            sample->temperature = rd_i16(d) * 0.01f;
            sample->valid |= HIPNUC_VALID_TEMPERATURE;
            break;

        case HIPNUC_J1939_MSG_QUAT:
            for (i = 0; i < 4; ++i) sample->quat[i] = rd_i16(d + 2 * i) * 0.0001f;
            sample->valid |= HIPNUC_VALID_QUAT;
            break;

        case HIPNUC_J1939_MSG_INCLINATION:
            sample->inclination[0] = (float)rd_i32(d) * J1939_MDEG2RAD;
            sample->inclination[1] = (float)rd_i32(d + 4) * J1939_MDEG2RAD;
            sample->valid |= HIPNUC_VALID_INCLINATION;
            break;

        case HIPNUC_J1939_MSG_TIME:
            set_utc(sample, d[0], d[1], d[2], d[3], d[4], d[5], rd_u16(d + 6), 1);
            break;

        case HIPNUC_J1939_MSG_POSITION:
            /* Latitude first. altitude_msl stays 0 until an ALTITUDE frame is merged. */
            sample->latitude = rd_i32(d) * 1e-7;
            sample->longitude = rd_i32(d + 4) * 1e-7;
            sample->valid |= HIPNUC_VALID_POSITION;
            break;

        case HIPNUC_J1939_MSG_ALTITUDE:
            /*
             * Height above mean sea level without a position, so
             * HIPNUC_VALID_POSITION is not set; hipnuc_j1939_merge() carries
             * altitude_msl over together with the undulation.
             */
            sample->altitude_msl = rd_i32(d) * 0.01;
            sample->undulation = rd_i16(d + 4) * 0.01f;
            sample->diff_age = rd_i16(d + 6) * 0.01f;
            sample->valid |= HIPNUC_VALID_UNDULATION | HIPNUC_VALID_DIFF_AGE;
            break;

        case HIPNUC_J1939_MSG_GNSS_STATUS:
            sample->position_quality = d[0];
            sample->heading_quality = d[1];
            sample->position_satellites = d[2];
            sample->heading_satellites = d[3];
            sample->ins_status = d[4];
            sample->valid |= HIPNUC_VALID_GNSS_QUALITY | HIPNUC_VALID_INS_STATUS;
            break;

        case HIPNUC_J1939_MSG_VELOCITY:
            /* Bytes 6..7 carry the 2D ground speed, which follows from vel_enu. */
            for (i = 0; i < 3; ++i) sample->vel_enu[i] = rd_i16(d + 2 * i) * 0.01f;
            sample->valid |= HIPNUC_VALID_VELOCITY_ENU;
            break;

        case HIPNUC_J1939_MSG_CANFD83:
            if (parse_canfd83(frame, sample, canfd83_sequence) < 0) {
                hipnuc_sample_clear(sample);
                return -1;
            }
            sample->node_id = hipnuc_j1939_source_address(frame->id);
            sample->valid |= HIPNUC_VALID_NODE_ID;
            break;

        default:
            hipnuc_sample_clear(sample);
            return HIPNUC_J1939_MSG_NONE;
    }
    return (int)msg;
}

/* ------------------------------------------------------------------------- */
/* Merge                                                                     */
/* ------------------------------------------------------------------------- */

void hipnuc_j1939_merge(hipnuc_sample_t *into, const hipnuc_sample_t *part)
{
    uint32_t v = part->valid;
    int i;

    if (into->source == HIPNUC_SOURCE_NONE) {
        into->source = part->source;
        into->node_id = part->node_id;
    }
    if (v & HIPNUC_VALID_NODE_ID) into->node_id = part->node_id;
    if (v & HIPNUC_VALID_STATUS) hipnuc_sample_set_status(into, part->main_status);
    if (v & HIPNUC_VALID_INS_STATUS) into->ins_status = part->ins_status;
    if (v & HIPNUC_VALID_ACC) for (i = 0; i < 3; ++i) into->acc[i] = part->acc[i];
    if (v & HIPNUC_VALID_GYR) for (i = 0; i < 3; ++i) into->gyr[i] = part->gyr[i];
    if (v & HIPNUC_VALID_MAG) for (i = 0; i < 3; ++i) into->mag[i] = part->mag[i];
    if (v & HIPNUC_VALID_EULER) {
        into->roll = part->roll;
        into->pitch = part->pitch;
        /* A ROLL_PITCH frame has no yaw: keep one that already came from a YAW frame. */
        if ((v & HIPNUC_VALID_HEADING) || !(into->valid & HIPNUC_VALID_HEADING)) into->yaw = part->yaw;
    }
    if (v & HIPNUC_VALID_HEADING) {
        into->heading = part->heading;
        into->yaw = part->yaw;
    }
    if (v & HIPNUC_VALID_QUAT) for (i = 0; i < 4; ++i) into->quat[i] = part->quat[i];
    if (v & HIPNUC_VALID_DEVICE_TIME) into->device_time_us = part->device_time_us;
    if (v & HIPNUC_VALID_UTC) into->utc = part->utc;
    if (v & HIPNUC_VALID_GPS_TIME) {
        into->gps_week = part->gps_week;
        into->gps_tow_ms = part->gps_tow_ms;
    }
    if (v & HIPNUC_VALID_PRESSURE) into->pressure = part->pressure;
    if (v & HIPNUC_VALID_TEMPERATURE) into->temperature = part->temperature;
    if (v & HIPNUC_VALID_INCLINATION) {
        into->inclination[0] = part->inclination[0];
        into->inclination[1] = part->inclination[1];
    }
    if (v & HIPNUC_VALID_HEAVE) {
        for (i = 0; i < 3; ++i) {
            into->heave_m[i] = part->heave_m[i];
            into->heave_hz[i] = part->heave_hz[i];
        }
    }
    if (v & HIPNUC_VALID_POSITION) {
        into->longitude = part->longitude;
        into->latitude = part->latitude;
        /* J1939 splits the position over two PGNs: POSITION has no altitude. */
        if (part->source != HIPNUC_SOURCE_J1939) into->altitude_msl = part->altitude_msl;
    }
    if (v & HIPNUC_VALID_UNDULATION) {
        into->undulation = part->undulation;
        /* ...and ALTITUDE carries altitude_msl next to the undulation. */
        if (part->source == HIPNUC_SOURCE_J1939 && !(v & HIPNUC_VALID_POSITION)) into->altitude_msl = part->altitude_msl;
    }
    if (v & HIPNUC_VALID_VELOCITY_ENU) for (i = 0; i < 3; ++i) into->vel_enu[i] = part->vel_enu[i];
    if (v & HIPNUC_VALID_ACC_ENU) for (i = 0; i < 3; ++i) into->acc_enu[i] = part->acc_enu[i];
    if (v & HIPNUC_VALID_GNSS_QUALITY) {
        into->position_quality = part->position_quality;
        into->position_satellites = part->position_satellites;
        into->heading_quality = part->heading_quality;
        into->heading_satellites = part->heading_satellites;
    }
    if (v & HIPNUC_VALID_DOP) {
        into->pdop = part->pdop;
        into->hdop = part->hdop;
    }
    if (v & HIPNUC_VALID_DIFF_AGE) into->diff_age = part->diff_age;
    if (v & HIPNUC_VALID_ODOMETER) into->odometer_speed = part->odometer_speed;
    if (v & HIPNUC_VALID_GNSS_POSITION) {
        into->gnss_longitude = part->gnss_longitude;
        into->gnss_latitude = part->gnss_latitude;
        into->gnss_altitude_msl = part->gnss_altitude_msl;
    }
    if (v & HIPNUC_VALID_GNSS_VELOCITY) for (i = 0; i < 3; ++i) into->gnss_vel_enu[i] = part->gnss_vel_enu[i];
    if (v & HIPNUC_VALID_SOG_COG) {
        into->sog = part->sog;
        into->cog = part->cog;
    }
    into->valid |= v;
}

/* ------------------------------------------------------------------------- */
/* Register access frames                                                    */
/* ------------------------------------------------------------------------- */

static void build_config(uint8_t dest, uint8_t source, uint16_t addr, hipnuc_j1939_cmd_t cmd,
                         uint32_t value, hipnuc_can_frame_t *out)
{
    memset(out, 0, sizeof(*out));
    out->id = config_id(dest, source);
    out->is_extended = 1;
    out->len = 8;
    wr_u16(&out->data[0], addr);
    out->data[2] = (uint8_t)cmd;
    out->data[3] = 0;
    wr_u32(&out->data[4], value);
}

void hipnuc_j1939_build_reg_write(uint8_t dest, uint8_t source, uint16_t addr, uint32_t value, hipnuc_can_frame_t *out)
{
    build_config(dest, source, addr, HIPNUC_J1939_CMD_WRITE, value, out);
}

void hipnuc_j1939_build_reg_read(uint8_t dest, uint8_t source, uint16_t addr, hipnuc_can_frame_t *out)
{
    /* The read request carries a value of 1 (register count, always one). */
    build_config(dest, source, addr, HIPNUC_J1939_CMD_READ, 1, out);
}

void hipnuc_j1939_build_trigger(uint8_t dest, uint8_t source, uint32_t pgn, hipnuc_can_frame_t *out)
{
    build_config(dest, source, 0x0096, HIPNUC_J1939_CMD_WRITE, pgn, out);
}

int hipnuc_j1939_is_config(const hipnuc_can_frame_t *frame)
{
    if (!frame || !frame->is_extended) return 0;
    /* priority 3, data page 0, PF 0xEF; PS is the destination address */
    return ((frame->id >> 16) & 0xFFFFU) == 0x0CEFU ? 1 : 0;
}

int hipnuc_j1939_parse_config(const hipnuc_can_frame_t *frame, uint8_t *source, uint16_t *addr,
                              hipnuc_j1939_cmd_t *cmd, uint8_t *status, uint32_t *value)
{
    if (!hipnuc_j1939_is_config(frame)) return -1;
    if (frame->is_remote || frame->is_error || frame->len != 8) return -1;
    if (source) *source = hipnuc_j1939_source_address(frame->id);
    if (addr) *addr = rd_u16(&frame->data[0]);
    if (cmd) *cmd = (hipnuc_j1939_cmd_t)frame->data[2];
    if (status) *status = frame->data[3];
    if (value) *value = rd_u32(&frame->data[4]);
    return 0;
}
