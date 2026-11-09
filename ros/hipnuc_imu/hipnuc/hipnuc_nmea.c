/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NMEA GGA and RMC; see hipnuc_nmea.h.
 */

#include "hipnuc_nmea.h"
#include "hipnuc_internal.h"

#include <float.h>
#include <string.h>

#define SENTENCE_MAX 128   /* longest sentence kept, terminator included */
#define FIELDS_MAX 24

/* [-]digits[.digits], locale independent. Returns 0 for an empty or
 * malformed field so that the caller leaves the matching valid bit clear. */
static int parse_number(const char *s, double *out)
{
    double value = 0.0, scale = 1.0;
    int negative = 0, digits = 0;

    if (*s == '-') {
        negative = 1;
        s++;
    } else if (*s == '+') {
        s++;
    }
    while (*s >= '0' && *s <= '9') {
        if (value > (DBL_MAX - 9.0) / 10.0) return 0;
        value = value * 10.0 + (*s++ - '0');
        digits++;
    }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            scale /= 10.0;
            value += (*s++ - '0') * scale;
            digits++;
        }
    }
    if (*s != '\0' || digits == 0) return 0;
    *out = negative ? -value : value;
    return 1;
}

static int parse_uint(const char *s, uint32_t maximum, uint32_t *out)
{
    uint32_t value = 0;
    if (*s == '\0') return 0;
    while (*s) {
        uint32_t digit;
        if (*s < '0' || *s > '9') return 0;
        digit = (uint32_t)(*s++ - '0');
        if (value > maximum / 10U || (value == maximum / 10U && digit > maximum % 10U)) return 0;
        value = value * 10U + digit;
    }
    *out = value;
    return 1;
}

static int parse_float(const char *s, float *out)
{
    double value;
    if (!parse_number(s, &value) || value > (double)FLT_MAX || value < -(double)FLT_MAX) return 0;
    *out = (float)value;
    return 1;
}

/* ddmm.mmmm plus hemisphere into signed decimal degrees */
static int parse_coordinate(const char *value, const char *hemi, char positive, char negative, double *out)
{
    double dmm, minutes;
    int degrees;
    double maximum = positive == 'N' ? 9000.0 : 18000.0;
    if (!parse_number(value, &dmm) || dmm < 0.0 || dmm > maximum) return 0;
    if ((hemi[0] != positive && hemi[0] != negative) || hemi[1] != '\0') return 0;
    degrees = (int)(dmm / 100.0);
    minutes = dmm - degrees * 100.0;
    if (minutes >= 60.0) return 0;
    *out = degrees + minutes / 60.0;
    if (hemi[0] == negative) *out = -*out;
    return 1;
}

/* hhmmss[.fraction] into the time-of-day fields; sub-millisecond digits are cut */
static int parse_time(const char *s, hipnuc_utc_t *u)
{
    unsigned h, m, sec, ms = 0, scale = 100;
    int i;
    for (i = 0; i < 6; i++) {
        if (s[i] < '0' || s[i] > '9') return 0;
    }
    if (s[6] != '\0' && s[6] != '.') return 0;
    h = (unsigned)(s[0] - '0') * 10U + (unsigned)(s[1] - '0');
    m = (unsigned)(s[2] - '0') * 10U + (unsigned)(s[3] - '0');
    sec = (unsigned)(s[4] - '0') * 10U + (unsigned)(s[5] - '0');
    if (h >= 24 || m >= 60 || sec > 60 || (sec == 60 && (h != 23 || m != 59))) return 0;
    if (s[6] == '.') {
        for (i = 7; s[i]; i++) {
            if (s[i] < '0' || s[i] > '9') return 0;
            ms += (unsigned)(s[i] - '0') * scale;
            scale /= 10U;
        }
    }
    u->hour = (uint8_t)h;
    u->minute = (uint8_t)m;
    u->second = (uint8_t)sec;
    u->millisecond = (uint16_t)ms;
    return 1;
}

/* ddmmyy into the date fields */
static int parse_date(const char *s, hipnuc_utc_t *u)
{
    uint32_t v;
    unsigned year;
    if (strlen(s) != 6 || !parse_uint(s, 999999U, &v)) return 0;
    year = (unsigned)(v % 100U);
    u->year = (uint16_t)(year + (year >= 80U ? 1900U : 2000U));
    u->month = (uint8_t)((v / 100U) % 100U);
    u->day = (uint8_t)(v / 10000U);
    return 1;
}

static void decode_gga(char **f, int n, hipnuc_data_t *d)
{
    uint32_t v;
    d->source = HIPNUC_SOURCE_NMEA_GGA;
    if (parse_time(f[1], &d->utc)) d->valid |= HIPNUC_VALID_UTC_TIME_OF_DAY;   /* no date in GGA */
    if (parse_coordinate(f[2], f[3], 'N', 'S', &d->gnss_latitude) &&
        parse_coordinate(f[4], f[5], 'E', 'W', &d->gnss_longitude)) {
        d->valid |= HIPNUC_VALID_GNSS_POSITION;
    } else {
        d->gnss_latitude = d->gnss_longitude = 0.0;
    }
    if (parse_uint(f[6], UINT8_MAX, &v)) {
        d->position_quality = (uint8_t)v;
        d->valid |= HIPNUC_VALID_POSITION_QUALITY;
    }
    if (parse_uint(f[7], UINT8_MAX, &v)) {
        d->position_satellites = (uint8_t)v;
        d->valid |= HIPNUC_VALID_POSITION_SATELLITES;
    }
    if (parse_float(f[8], &d->hdop) && d->hdop >= 0.0f) d->valid |= HIPNUC_VALID_HDOP;
    if (strcmp(f[10], "M") == 0 && parse_number(f[9], &d->gnss_altitude_msl)) d->valid |= HIPNUC_VALID_GNSS_ALTITUDE;
    if (strcmp(f[12], "M") == 0 && parse_float(f[11], &d->undulation)) d->valid |= HIPNUC_VALID_UNDULATION;
    if (parse_float(f[13], &d->diff_age) && d->diff_age >= 0.0f) d->valid |= HIPNUC_VALID_DIFF_AGE;
    (void)n;
}

static void decode_rmc(char **f, int n, hipnuc_data_t *d)
{
    float knots, course;
    int have_time;
    d->source = HIPNUC_SOURCE_NMEA_RMC;
    have_time = parse_time(f[1], &d->utc);
    if (have_time) d->valid |= HIPNUC_VALID_UTC_TIME_OF_DAY;
    if ((f[2][0] == 'A' || f[2][0] == 'V') && f[2][1] == '\0') {
        d->nmea_status = f[2][0];
        d->valid |= HIPNUC_VALID_NMEA_STATUS;
    }
    if (parse_coordinate(f[3], f[4], 'N', 'S', &d->gnss_latitude) &&
        parse_coordinate(f[5], f[6], 'E', 'W', &d->gnss_longitude)) {
        d->valid |= HIPNUC_VALID_GNSS_POSITION;
    } else {
        d->gnss_latitude = d->gnss_longitude = 0.0;
    }
    if (parse_float(f[7], &knots) && knots >= 0.0f) {
        d->sog = knots * HIPNUC_KNOT2MPS;
        d->valid |= HIPNUC_VALID_SOG;
    }
    if (parse_float(f[8], &course) && course >= 0.0f && course <= 360.0f) {
        d->cog = course * HIPNUC_DEG2RAD;
        d->valid |= HIPNUC_VALID_COG;
    }
    if (have_time && parse_date(f[9], &d->utc) && hn_utc_valid(&d->utc)) d->valid |= HIPNUC_VALID_UTC;
    else d->utc.year = d->utc.month = d->utc.day = 0;
    if (n > 12 && f[12][0] >= 'A' && f[12][0] <= 'Z' && f[12][1] == '\0') {
        d->nmea_mode = f[12][0];
        d->valid |= HIPNUC_VALID_NMEA_MODE;
    }
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int hipnuc_nmea_decode(const char *line, hipnuc_data_t *out)
{
    char s[SENTENCE_MAX];
    char *fields[FIELDS_MAX];
    const char *start;
    char *star, *p;
    size_t len;
    int n = 0, h1, h2, gga;
    uint8_t sum = 0;

    if (!line || (start = strrchr(line, '$')) == NULL) return 0;
    len = strlen(start);
    if (len < 7 || start[6] != ',') return 0;   /* "$ttGGA," or "$ttRMC," */
    gga = memcmp(start + 3, "GGA", 3) == 0;
    if (!gga && memcmp(start + 3, "RMC", 3) != 0) return 0;
    while (len > 0 && (start[len - 1] == '\r' || start[len - 1] == '\n')) len--;
    if (len >= sizeof(s)) return -1;
    memcpy(s, start, len);
    s[len] = '\0';

    /* "$" address "," fields "*" hh */
    star = strchr(s, '*');
    if (!star || star - s < 6 || strlen(star) != 3) return -1;
    h1 = hex_value(star[1]);
    h2 = hex_value(star[2]);
    if (h1 < 0 || h2 < 0) return -1;
    for (p = s + 1; p < star; ++p) sum ^= (uint8_t)*p;
    if (sum != (uint8_t)((h1 << 4) | h2)) return -1;
    *star = '\0';

    for (p = s + 1; n < FIELDS_MAX;) {   /* field 0 is the address, e.g. "GPGGA" */
        char *comma = strchr(p, ',');
        fields[n++] = p;
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }
    if (n < (gga ? 14 : 10)) return -1;
    if (out) {
        memset(out, 0, sizeof(*out));
        if (gga) decode_gga(fields, n, out);
        else decode_rmc(fields, n, out);
    }
    return 1;
}
