/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * JSON formatting of hipnuc_sample_t. See hipnuc_json.h.
 */

#include "hipnuc_json.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    char  *buf;
    size_t size;
    size_t used;     /* characters produced so far (also counts when sizing) */
    int    failed;
    int    first;    /* no member emitted yet */
} json_writer_t;

static void put(json_writer_t *w, const char *text)
{
    size_t n = strlen(text);
    if (w->failed) return;
    if (w->buf) {
        if (w->used + n + 1 > w->size) { w->failed = 1; return; }
        memcpy(w->buf + w->used, text, n + 1);
    }
    w->used += n;
}

static void key(json_writer_t *w, const char *name)
{
    put(w, w->first ? "\"" : ",\"");
    put(w, name);
    put(w, "\":");
    w->first = 0;
}

static void number(json_writer_t *w, double value, int digits)
{
    char text[40];
    char normalized[40];
    size_t in = 0, out = 0;
    int length;
    if (!(value == value) || value > DBL_MAX || value < -DBL_MAX) {
        w->failed = 1;
        return;
    }
    length = snprintf(text, sizeof(text), "%.*g", digits, value);
    if (length < 0 || (size_t)length >= sizeof(text)) { w->failed = 1; return; }
    /* Without the grouping flag, %g localizes only the radix character.
     * Normalize that possibly multibyte token in this result, without
     * changing the process locale or allocating a platform locale object. */
    while (text[in]) {
        char c = text[in++];
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == 'e' || c == 'E') {
            normalized[out++] = c;
        } else {
            normalized[out++] = '.';
            while (text[in] && !(text[in] >= '0' && text[in] <= '9') &&
                   text[in] != 'e' && text[in] != 'E') in++;
        }
    }
    normalized[out] = '\0';
    put(w, normalized);
}

/* Decimal formatting without %llu, which some C libraries lack. */
static void integer(json_writer_t *w, uint64_t value)
{
    char text[24];
    int pos = (int)sizeof(text) - 1;
    text[pos] = 0;
    do {
        text[--pos] = (char)('0' + (int)(value % 10U));
        value /= 10U;
    } while (value && pos > 0);
    put(w, text + pos);
}

static void scalar(json_writer_t *w, const char *name, double value, int digits)
{
    key(w, name);
    number(w, value, digits);
}

static void vector(json_writer_t *w, const char *name, const float *v, int count, int digits)
{
    int i;
    key(w, name);
    put(w, "[");
    for (i = 0; i < count; ++i) {
        if (i) put(w, ",");
        number(w, v[i], digits);
    }
    put(w, "]");
}

static const char *source_name(hipnuc_source_t source)
{
    switch (source) {
    case HIPNUC_SOURCE_HI91: return "HI91";
    case HIPNUC_SOURCE_HI81: return "HI81";
    case HIPNUC_SOURCE_HI83: return "HI83";
    case HIPNUC_SOURCE_NMEA_GGA: return "GGA";
    case HIPNUC_SOURCE_NMEA_RMC: return "RMC";
    case HIPNUC_SOURCE_J1939: return "J1939";
    case HIPNUC_SOURCE_CANFD83: return "CANFD83";
    default: return "NONE";
    }
}

static const char *ins_status_name(uint8_t status)
{
    switch (status) {
    case HIPNUC_INS_INVALID: return "invalid";
    case HIPNUC_INS_ALIGNING: return "aligning";
    case HIPNUC_INS_NAVIGATING: return "navigating";
    case HIPNUC_INS_DEAD_RECKONING: return "dead_reckoning";
    default: return "unknown";
    }
}

int hipnuc_json_sample(const hipnuc_sample_t *s, char *buf, size_t size)
{
    json_writer_t w;
    uint64_t v;

    if (!s || (buf == NULL && size != 0) || (buf != NULL && size == 0)) {
        if (buf && size) buf[0] = '\0';
        return -1;
    }
    w.buf = buf;
    w.size = size;
    w.used = 0;
    w.failed = 0;
    w.first = 1;
    v = s->valid;

    put(&w, "{");
    key(&w, "type");
    put(&w, "\"");
    put(&w, source_name(s->source));
    put(&w, "\"");

    if ((v & HIPNUC_VALID_NODE_ID) || s->source == HIPNUC_SOURCE_J1939 ||
        s->source == HIPNUC_SOURCE_CANFD83) {
        key(&w, "node_id"); integer(&w, s->node_id);
    }
    if (v & HIPNUC_VALID_STATUS) {
        key(&w, "main_status"); integer(&w, s->main_status);
        key(&w, "status_flags"); put(&w, "[");
        {
            static const struct { uint16_t bit; const char *name; } flags[] = {
                { HIPNUC_STATUS_WB_CONV, "WB_CONV" }, { HIPNUC_STATUS_MAG_DIST, "MAG_DIST" },
                { HIPNUC_STATUS_ACC_SAT, "ACC_SAT" }, { HIPNUC_STATUS_GYR_SAT, "GYR_SAT" },
                { HIPNUC_STATUS_ATT_CONV, "ATT_CONV" }, { HIPNUC_STATUS_STATIC, "STATIC" },
                { HIPNUC_STATUS_MAG_AIDING, "MAG_AIDING" }, { HIPNUC_STATUS_UTC_UNSYNC, "UTC_UNSYNC" },
                { HIPNUC_STATUS_SOUT_PULSE, "SOUT_PULSE" }
            };
            int i, n = 0;
            for (i = 0; i < (int)(sizeof(flags) / sizeof(flags[0])); ++i) {
                if (s->main_status & flags[i].bit) {
                    put(&w, n++ ? ",\"" : "\"");
                    put(&w, flags[i].name);
                    put(&w, "\"");
                }
            }
        }
        put(&w, "]");
    }
    if (v & HIPNUC_VALID_INS_STATUS) {
        key(&w, "ins_status"); integer(&w, s->ins_status);
        key(&w, "ins_status_name"); put(&w, "\""); put(&w, ins_status_name(s->ins_status)); put(&w, "\"");
    }
    if (v & HIPNUC_VALID_DEVICE_TIME) {
        key(&w, "device_time_us"); integer(&w, s->device_time_us);
        scalar(&w, "device_time_s", (double)s->device_time_us / 1e6, 15);
    }
    if (v & HIPNUC_VALID_UTC) {
        char text[40];
        snprintf(text, sizeof(text), "\"%04u-%02u-%02uT%02u:%02u:%02u.%03uZ\"",
                 (unsigned)s->utc.year, (unsigned)s->utc.month, (unsigned)s->utc.day,
                 (unsigned)s->utc.hour, (unsigned)s->utc.minute, (unsigned)s->utc.second,
                 (unsigned)s->utc.millisecond);
        key(&w, "utc");
        put(&w, text);
    }
    if ((v & HIPNUC_VALID_UTC_TIME_OF_DAY) && !(v & HIPNUC_VALID_UTC)) {
        char text[24];
        snprintf(text, sizeof(text), "\"%02u:%02u:%02u.%03u\"",
                 (unsigned)s->utc.hour, (unsigned)s->utc.minute,
                 (unsigned)s->utc.second, (unsigned)s->utc.millisecond);
        key(&w, "utc_time_of_day"); put(&w, text);
    }
    if (v & HIPNUC_VALID_GPS_TIME) {
        key(&w, "gps_week"); integer(&w, s->gps_week);
        key(&w, "gps_time_of_week_ms"); integer(&w, s->gps_tow_ms);
    }
    if (v & HIPNUC_VALID_ACC) vector(&w, "acceleration_m_s2", s->acc, 3, 7);
    if (v & HIPNUC_VALID_GYR) vector(&w, "angular_velocity_rad_s", s->gyr, 3, 7);
    if (v & HIPNUC_VALID_MAG) vector(&w, "magnetic_field_t", s->mag, 3, 7);
    if (v & (HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_YAW)) {
        key(&w, "euler_rad"); put(&w, "[");
        if (v & HIPNUC_VALID_ROLL_PITCH) {
            number(&w, s->roll, 7); put(&w, ","); number(&w, s->pitch, 7);
        } else {
            put(&w, "null,null");
        }
        put(&w, ",");
        if (v & HIPNUC_VALID_YAW) number(&w, s->yaw, 7); else put(&w, "null");
        put(&w, "]");
    }
    if (v & HIPNUC_VALID_HEADING) scalar(&w, "heading_rad", s->heading, 7);
    if (v & HIPNUC_VALID_QUAT) vector(&w, "quaternion_wxyz", s->quat, 4, 7);
    if (v & HIPNUC_VALID_INCLINATION) vector(&w, "inclination_rad", s->inclination, 2, 7);
    if (v & HIPNUC_VALID_INCLINATION_YAW) scalar(&w, "inclination_yaw_rad", s->inclination_yaw, 7);
    if (v & HIPNUC_VALID_PRESSURE) scalar(&w, "pressure_pa", s->pressure, 8);
    if (v & HIPNUC_VALID_TEMPERATURE) scalar(&w, "temperature_c", s->temperature, 6);
    if (v & HIPNUC_VALID_HEAVE) vector(&w, "heave_surge_sway_m", s->heave_m, 3, 6);
    if (v & HIPNUC_VALID_HEAVE_FREQUENCY) vector(&w, "heave_surge_sway_hz", s->heave_hz, 3, 6);
    if (v & HIPNUC_VALID_POSITION) {
        scalar(&w, "longitude_deg", s->longitude, 10);
        scalar(&w, "latitude_deg", s->latitude, 10);
    }
    if (v & HIPNUC_VALID_ALTITUDE) scalar(&w, "altitude_msl_m", s->altitude_msl, 9);
    if (v & HIPNUC_VALID_VELOCITY_ENU) vector(&w, "velocity_enu_m_s", s->vel_enu, 3, 6);
    if (v & HIPNUC_VALID_ACC_ENU) vector(&w, "acceleration_enu_m_s2", s->acc_enu, 3, 6);
    if (v & HIPNUC_VALID_SOG) scalar(&w, "speed_over_ground_m_s", s->sog, 6);
    if (v & HIPNUC_VALID_COG) scalar(&w, "course_over_ground_rad", s->cog, 7);
    if (v & HIPNUC_VALID_ODOMETER) scalar(&w, "odometer_speed_m_s", s->odometer_speed, 6);
    if (v & HIPNUC_VALID_GNSS_POSITION) {
        scalar(&w, "gnss_longitude_deg", s->gnss_longitude, 10);
        scalar(&w, "gnss_latitude_deg", s->gnss_latitude, 10);
    }
    if (v & HIPNUC_VALID_GNSS_ALTITUDE) scalar(&w, "gnss_altitude_msl_m", s->gnss_altitude_msl, 9);
    if (v & HIPNUC_VALID_GNSS_VELOCITY) vector(&w, "gnss_velocity_enu_m_s", s->gnss_vel_enu, 3, 6);
    if (v & HIPNUC_VALID_POSITION_QUALITY) { key(&w, "position_quality"); integer(&w, s->position_quality); }
    if (v & HIPNUC_VALID_POSITION_SATELLITES) { key(&w, "position_satellites"); integer(&w, s->position_satellites); }
    if (v & HIPNUC_VALID_HEADING_QUALITY) { key(&w, "heading_quality"); integer(&w, s->heading_quality); }
    if (v & HIPNUC_VALID_HEADING_SATELLITES) { key(&w, "heading_satellites"); integer(&w, s->heading_satellites); }
    if (v & HIPNUC_VALID_PDOP) scalar(&w, "pdop", s->pdop, 4);
    if (v & HIPNUC_VALID_HDOP) scalar(&w, "hdop", s->hdop, 4);
    if (v & HIPNUC_VALID_NMEA_STATUS) {
        char text[] = {'"', s->nmea_status, '"', '\0'};
        if (s->nmea_status != 'A' && s->nmea_status != 'V') w.failed = 1;
        key(&w, "nmea_status"); put(&w, text);
    }
    if (v & HIPNUC_VALID_NMEA_MODE) {
        char text[] = {'"', s->nmea_mode, '"', '\0'};
        if (s->nmea_mode < 'A' || s->nmea_mode > 'Z') w.failed = 1;
        key(&w, "nmea_mode"); put(&w, text);
    }
    if (v & HIPNUC_VALID_DIFF_AGE) scalar(&w, "differential_age_s", s->diff_age, 5);
    if (v & HIPNUC_VALID_UNDULATION) scalar(&w, "geoid_separation_m", s->undulation, 6);
    put(&w, "}");

    if (w.failed) {
        if (buf && size) buf[0] = '\0';
        return -1;
    }
    return (int)w.used;
}
