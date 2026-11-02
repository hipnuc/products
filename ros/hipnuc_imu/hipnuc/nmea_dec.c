/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NMEA 0183 decoder for GGA and RMC. See nmea_dec.h.
 */

#include "nmea_dec.h"

#include <float.h>
#include <string.h>

#define NMEA_MAX_FIELDS 24

/* Locale-independent decimal parser: [-]digits[.digits]. Returns 0 on an
 * empty or malformed field so the caller can keep its has_* flag clear. */
static int parse_number(const char *s, double *out)
{
    double value = 0.0;
    double scale = 1.0;
    int negative = 0;
    int digits = 0;

    if (*s == '-') { negative = 1; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') {
        if (value > (DBL_MAX - 9.0) / 10.0) return 0;
        value = value * 10.0 + (*s - '0');
        s++;
        digits++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            scale /= 10.0;
            value += (*s - '0') * scale;
            s++;
            digits++;
        }
    }
    if (*s != '\0' || digits == 0) return 0;
    *out = negative ? -value : value;
    return 1;
}

/* Unsigned decimal integer, checked before conversion (no floating cast). */
static int parse_uint(const char *s, uint32_t maximum, uint32_t *out)
{
    uint32_t value = 0;
    if (*s == '\0') return 0;
    while (*s) {
        uint32_t digit;
        if (*s < '0' || *s > '9') return 0;
        digit = (unsigned)(*s++ - '0');
        if (value > maximum / 10 || (value == maximum / 10 && digit > maximum % 10)) return 0;
        value = value * 10 + digit;
    }
    *out = value;
    return 1;
}

static int parse_float(const char *s, float *out)
{
    double value;
    if (!parse_number(s, &value) || value > FLT_MAX || value < -FLT_MAX) return 0;
    *out = (float)value;
    return 1;
}

/* ddmm.mmmm with hemisphere -> signed decimal degrees */
static int parse_coordinate(const char *value, const char *hemi, char positive, char negative, double *out)
{
    double dmm, minutes;
    int degrees;
    int maximum = positive == 'N' ? 90 : 180;
    if (!parse_number(value, &dmm) || dmm < 0.0 || dmm > maximum * 100.0) return 0;
    if ((hemi[0] != positive && hemi[0] != negative) || hemi[1] != '\0') return 0;
    degrees = (int)(dmm / 100.0);
    minutes = dmm - degrees * 100.0;
    if (minutes >= 60.0) return 0;
    *out = degrees + minutes / 60.0;
    if (hemi[0] == negative) *out = -*out;
    return 1;
}

/* hhmmss[.fraction], with sub-millisecond digits truncated. */
static int parse_time(const char *s, uint8_t *hour, uint8_t *minute, uint16_t *second_ms)
{
    unsigned h, m, second, ms = 0, scale = 100;
    int i;
    for (i = 0; i < 6; i++) if (s[i] < '0' || s[i] > '9') return 0;
    if (s[6] != '\0' && s[6] != '.') return 0;
    h = (unsigned)(s[0] - '0') * 10 + (unsigned)(s[1] - '0');
    m = (unsigned)(s[2] - '0') * 10 + (unsigned)(s[3] - '0');
    second = (unsigned)(s[4] - '0') * 10 + (unsigned)(s[5] - '0');
    if (h >= 24 || m >= 60 || second > 60 || (second == 60 && (h != 23 || m != 59))) return 0;
    if (s[6] == '.') {
        for (i = 7; s[i]; i++) {
            if (s[i] < '0' || s[i] > '9') return 0;
            ms += (unsigned)(s[i] - '0') * scale;
            scale /= 10;
        }
    }
    *hour = (uint8_t)h;
    *minute = (uint8_t)m;
    *second_ms = (uint16_t)(second * 1000 + ms);
    return 1;
}

/* ddmmyy */
static int parse_date(const char *s, uint16_t *year, uint8_t *month, uint8_t *day)
{
    uint32_t v;
    hipnuc_utc_t utc = {0};
    if (strlen(s) != 6 || !parse_uint(s, 999999, &v)) return 0;
    *day = (uint8_t)(v / 10000);
    *month = (uint8_t)((v / 100) % 100);
    *year = (uint16_t)(v % 100);
    *year = (uint16_t)(*year + (*year >= 80 ? 1900 : 2000));
    utc.year = *year; utc.month = *month; utc.day = *day;
    return hipnuc_utc_is_valid(&utc);
}

static void dec_gga(nmea_gga_t *g, char **f, int n)
{
    uint32_t i;
    memset(g, 0, sizeof(*g));
    if (n < 14) return;
    g->has_time = (uint8_t)parse_time(f[1], &g->hour, &g->minute, &g->second_ms);
    g->has_position = (uint8_t)(parse_coordinate(f[2], f[3], 'N', 'S', &g->lat) &&
                                parse_coordinate(f[4], f[5], 'E', 'W', &g->lon));
    if (parse_uint(f[6], UINT8_MAX, &i)) { g->quality = (uint8_t)i; g->has_quality = 1; }
    if (parse_uint(f[7], UINT8_MAX, &i)) { g->satellites = (uint8_t)i; g->has_satellites = 1; }
    g->has_hdop = (uint8_t)(parse_float(f[8], &g->hdop) && g->hdop >= 0);
    g->has_altitude = (uint8_t)(strcmp(f[10], "M") == 0 && parse_number(f[9], &g->altitude_msl));
    g->has_undulation = (uint8_t)(strcmp(f[12], "M") == 0 && parse_float(f[11], &g->undulation));
    g->has_diff_age = (uint8_t)(parse_float(f[13], &g->diff_age) && g->diff_age >= 0);
    if (n > 14 && parse_uint(f[14], UINT16_MAX, &i)) g->station_id = (uint16_t)i;
}

static void dec_rmc(nmea_rmc_t *r, char **f, int n)
{
    memset(r, 0, sizeof(*r));
    r->status = 'V';
    r->mode = 'N';
    if (n < 10) return;
    r->has_time = (uint8_t)parse_time(f[1], &r->hour, &r->minute, &r->second_ms);
    if ((f[2][0] == 'A' || f[2][0] == 'V') && f[2][1] == '\0') {
        r->status = f[2][0];
        r->has_status = 1;
    }
    r->has_position = (uint8_t)(parse_coordinate(f[3], f[4], 'N', 'S', &r->lat) &&
                                parse_coordinate(f[5], f[6], 'E', 'W', &r->lon));
    r->has_sog = (uint8_t)(parse_float(f[7], &r->sog) && r->sog >= 0);
    r->has_cog = (uint8_t)(parse_float(f[8], &r->cog) && r->cog >= 0 && r->cog <= 360);
    r->has_date = (uint8_t)parse_date(f[9], &r->year, &r->month, &r->day);
    if (n > 12 && f[12][0] >= 'A' && f[12][0] <= 'Z' && f[12][1] == '\0') {
        r->mode = f[12][0];
        r->has_mode = 1;
    }
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* raw->buf holds "$....*hh" followed by CR/LF and a NUL. */
static int parse_sentence(nmea_raw_t *raw)
{
    char *s = (char *)raw->buf;
    char *star;
    char *fields[NMEA_MAX_FIELDS];
    char *p;
    int n = 0;
    int h1, h2;
    uint8_t sum = 0;

    star = strchr(s, '*');
    if (!star || star - s < 6 || strlen(star) != 3) return -1;
    h1 = hex_value(star[1]);
    h2 = hex_value(star[2]);
    if (h1 < 0 || h2 < 0) return -1;
    for (p = s + 1; p < star; ++p) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)((h1 << 4) | h2)) {
        raw->checksum_error_count++;
        return -1;
    }
    *star = '\0';

    /* Split on commas; field 0 is the address, e.g. "GPGGA". */
    p = s + 1;
    while (n < NMEA_MAX_FIELDS) {
        char *comma = strchr(p, ',');
        fields[n++] = p;
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    if (strlen(fields[0]) != 5) return 0;

    raw->talker[0] = fields[0][0];
    raw->talker[1] = fields[0][1];
    raw->talker[2] = '\0';

    if (memcmp(fields[0] + 2, "GGA", 3) == 0) {
        if (n < 14) return -1;
        raw->msg_type = NMEA_MSG_GGA;
        dec_gga(&raw->gga, fields, n);
    } else if (memcmp(fields[0] + 2, "RMC", 3) == 0) {
        if (n < 10) return -1;
        raw->msg_type = NMEA_MSG_RMC;
        dec_rmc(&raw->rmc, fields, n);
    } else {
        return 0;
    }
    raw->sentence_count++;
    return 1;
}

int nmea_input(nmea_raw_t *raw, uint8_t data)
{
    if (raw->nbyte == 0) {
        if (data != '$') return 0;
        raw->buf[0] = '$';
        raw->nbyte = 1;
        return 0;
    }

    if (data == '$') {          /* a new sentence started before the old one ended */
        raw->buf[0] = '$';
        raw->nbyte = 1;
        return -1;
    }

    if (raw->nbyte >= NMEA_MAX_RAW_SIZE - 1) {
        raw->nbyte = 0;
        return -1;
    }

    if (data == '\r') return 0;
    if (data == '\n') {
        raw->buf[raw->nbyte] = '\0';
        raw->nbyte = 0;
        raw->msg_type = NMEA_MSG_NONE;
        return parse_sentence(raw);
    }
    if (data < 0x20 || data > 0x7E) {   /* binary noise: abandon the sentence */
        raw->nbyte = 0;
        return -1;
    }

    raw->buf[raw->nbyte++] = data;
    return 0;
}

/* NMEA fields to SI sample conversion. */
static void set_time_of_day(hipnuc_sample_t *s, uint8_t hour, uint8_t minute, uint16_t second_ms)
{
    s->utc.hour = hour;
    s->utc.minute = minute;
    s->utc.second = (uint8_t)(second_ms / 1000);
    s->utc.millisecond = (uint16_t)(second_ms % 1000);
    s->valid |= HIPNUC_VALID_UTC_TIME_OF_DAY;
}

void hipnuc_sample_from_gga(const nmea_gga_t *g, hipnuc_sample_t *s)
{
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_NMEA_GGA;
    if (g->has_quality) { s->position_quality = g->quality; s->valid |= HIPNUC_VALID_POSITION_QUALITY; }
    if (g->has_satellites) { s->position_satellites = g->satellites; s->valid |= HIPNUC_VALID_POSITION_SATELLITES; }
    if (g->has_hdop) { s->hdop = g->hdop; s->valid |= HIPNUC_VALID_HDOP; }
    if (g->has_position) {
        s->gnss_longitude = g->lon;
        s->gnss_latitude = g->lat;
        s->valid |= HIPNUC_VALID_GNSS_POSITION;
    }
    if (g->has_altitude) { s->gnss_altitude_msl = g->altitude_msl; s->valid |= HIPNUC_VALID_GNSS_ALTITUDE; }
    if (g->has_undulation) { s->undulation = g->undulation; s->valid |= HIPNUC_VALID_UNDULATION; }
    if (g->has_diff_age) { s->diff_age = g->diff_age; s->valid |= HIPNUC_VALID_DIFF_AGE; }
    if (g->has_time) {
        /* Time of day only: the date is unknown, so UTC stays invalid. */
        set_time_of_day(s, g->hour, g->minute, g->second_ms);
    }
}

void hipnuc_sample_from_rmc(const nmea_rmc_t *r, hipnuc_sample_t *s)
{
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_NMEA_RMC;
    if (r->has_status) { s->nmea_status = r->status; s->valid |= HIPNUC_VALID_NMEA_STATUS; }
    if (r->has_mode) { s->nmea_mode = r->mode; s->valid |= HIPNUC_VALID_NMEA_MODE; }
    if (r->has_position) {
        s->gnss_longitude = r->lon;
        s->gnss_latitude = r->lat;
        s->valid |= HIPNUC_VALID_GNSS_POSITION;
    }
    if (r->has_sog) {
        s->sog = r->sog * HIPNUC_KNOT2MPS;
        s->valid |= HIPNUC_VALID_SOG;
    }
    if (r->has_cog) {
        s->cog = r->cog * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_COG;
    }
    if (r->has_time) set_time_of_day(s, r->hour, r->minute, r->second_ms);
    if (r->has_date && r->has_time) {
        s->utc.year = r->year;
        s->utc.month = r->month;
        s->utc.day = r->day;
        s->valid |= HIPNUC_VALID_UTC;
    }
}

int hipnuc_sample_from_nmea(const nmea_raw_t *raw, hipnuc_sample_t *s)
{
    hipnuc_sample_clear(s);
    if (raw->msg_type == NMEA_MSG_GGA) { hipnuc_sample_from_gga(&raw->gga, s); return 1; }
    if (raw->msg_type == NMEA_MSG_RMC) { hipnuc_sample_from_rmc(&raw->rmc, s); return 1; }
    return 0;
}
