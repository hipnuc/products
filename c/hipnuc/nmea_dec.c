/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NMEA 0183 decoder for GGA and RMC. See nmea_dec.h.
 */

#include "nmea_dec.h"

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

static int parse_int(const char *s, int *out)
{
    double v;
    if (!parse_number(s, &v)) return 0;
    *out = (int)v;
    return 1;
}

/* ddmm.mmmm with hemisphere -> signed decimal degrees */
static int parse_coordinate(const char *value, const char *hemi, char positive, char negative, double *out)
{
    double dmm, minutes;
    int degrees;
    if (!parse_number(value, &dmm) || dmm < 0.0) return 0;
    if (hemi[0] != positive && hemi[0] != negative) return 0;
    degrees = (int)(dmm / 100.0);
    minutes = dmm - degrees * 100.0;
    if (minutes >= 60.0) return 0;
    *out = degrees + minutes / 60.0;
    if (hemi[0] == negative) *out = -*out;
    return 1;
}

/* hhmmss[.sss] */
static int parse_time(const char *s, uint8_t *hour, uint8_t *minute, float *second)
{
    double t;
    int whole;
    if (strlen(s) < 6 || !parse_number(s, &t)) return 0;
    whole = (int)t;
    *hour = (uint8_t)(whole / 10000);
    *minute = (uint8_t)((whole / 100) % 100);
    *second = (float)(t - (whole / 100) * 100);
    return *hour < 24 && *minute < 60;
}

/* ddmmyy */
static int parse_date(const char *s, uint16_t *year, uint8_t *month, uint8_t *day)
{
    int v;
    if (strlen(s) != 6 || !parse_int(s, &v)) return 0;
    *day = (uint8_t)(v / 10000);
    *month = (uint8_t)((v / 100) % 100);
    *year = (uint16_t)(v % 100);
    *year = (uint16_t)(*year + (*year >= 80 ? 1900 : 2000));
    return *month >= 1 && *month <= 12 && *day >= 1 && *day <= 31;
}

static void dec_gga(nmea_gga_t *g, char **f, int n)
{
    double v;
    int i;
    memset(g, 0, sizeof(*g));
    if (n < 14) return;
    g->has_time = (uint8_t)parse_time(f[1], &g->hour, &g->minute, &g->second);
    g->has_position = (uint8_t)(parse_coordinate(f[2], f[3], 'N', 'S', &g->lat) &&
                                parse_coordinate(f[4], f[5], 'E', 'W', &g->lon));
    if (parse_int(f[6], &i)) g->quality = (uint8_t)i;
    if (parse_int(f[7], &i)) g->satellites = (uint8_t)i;
    if (parse_number(f[8], &v)) g->hdop = (float)v;
    if (parse_number(f[9], &v)) { g->altitude_msl = v; g->has_altitude = 1; }
    if (parse_number(f[11], &v)) { g->undulation = (float)v; g->has_undulation = 1; }
    if (parse_number(f[13], &v)) { g->diff_age = (float)v; g->has_diff_age = 1; }
    if (n > 14 && parse_int(f[14], &i)) g->station_id = (uint16_t)i;
}

static void dec_rmc(nmea_rmc_t *r, char **f, int n)
{
    double v;
    memset(r, 0, sizeof(*r));
    r->status = 'V';
    r->mode = 'N';
    if (n < 10) return;
    r->has_time = (uint8_t)parse_time(f[1], &r->hour, &r->minute, &r->second);
    if (f[2][0] == 'A' || f[2][0] == 'V') r->status = f[2][0];
    r->has_position = (uint8_t)(parse_coordinate(f[3], f[4], 'N', 'S', &r->lat) &&
                                parse_coordinate(f[5], f[6], 'E', 'W', &r->lon));
    if (parse_number(f[7], &v)) { r->sog = (float)v; r->has_sog = 1; }
    if (parse_number(f[8], &v)) { r->cog = (float)v; r->has_cog = 1; }
    r->has_date = (uint8_t)parse_date(f[9], &r->year, &r->month, &r->day);
    if (n > 12 && f[12][0] != '\0') r->mode = f[12][0];
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
    if (!star || star - s < 6) return -1;
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
        raw->msg_type = NMEA_MSG_GGA;
        dec_gga(&raw->gga, fields, n);
    } else if (memcmp(fields[0] + 2, "RMC", 3) == 0) {
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
