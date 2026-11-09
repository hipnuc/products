/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal helpers shared by the decoders of this SDK. Not part of the API:
 * include it only from the SDK's own .c files.
 */

#ifndef HIPNUC_INTERNAL_H
#define HIPNUC_INTERNAL_H

#include <string.h>

#include "hipnuc_data.h"

#if defined(__cplusplus) || (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L)
#define HIPNUC_INLINE static inline
#else
#define HIPNUC_INLINE static __inline   /* C90 modes: MSVC, Keil ARMCC without --c99 */
#endif

/* Little-endian readers and writers; safe for any alignment and host byte order. */
HIPNUC_INLINE uint16_t hn_u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
HIPNUC_INLINE int16_t hn_i16(const uint8_t *p) { return (int16_t)hn_u16(p); }

HIPNUC_INLINE uint32_t hn_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

HIPNUC_INLINE int32_t hn_i32(const uint8_t *p) { return (int32_t)hn_u32(p); }
HIPNUC_INLINE uint64_t hn_u64(const uint8_t *p) { return (uint64_t)hn_u32(p) | ((uint64_t)hn_u32(p + 4) << 32); }

HIPNUC_INLINE float hn_f32(const uint8_t *p)
{
    float f;
    uint32_t u = hn_u32(p);
    memcpy(&f, &u, sizeof(f));
    return f;
}

/* binary64 fields; the decoders reject frames that carry them when double is not 8 bytes */
HIPNUC_INLINE double hn_f64(const uint8_t *p)
{
    double d = 0;
    uint64_t u = hn_u64(p);
    if (sizeof(d) == sizeof(u)) memcpy(&d, &u, sizeof(d));
    return d;
}

HIPNUC_INLINE void hn_f32n(float *dst, const uint8_t *p, int count)
{
    int i;
    for (i = 0; i < count; ++i) dst[i] = hn_f32(p + 4 * i);
}

HIPNUC_INLINE void hn_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

HIPNUC_INLINE void hn_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)(v >> 24);
}

/* main_status and the flags derived from it */
HIPNUC_INLINE void hn_set_status(hipnuc_data_t *d, uint16_t main_status)
{
    d->main_status = main_status;
    d->gyro_bias_converged = (main_status & HIPNUC_STATUS_WB_CONV) ? 0 : 1;
    d->attitude_converged = (main_status & HIPNUC_STATUS_ATT_CONV) ? 0 : 1;
    d->magnetic_disturbance = (main_status & HIPNUC_STATUS_MAG_DIST) ? 1 : 0;
    d->device_static = (main_status & HIPNUC_STATUS_STATIC) ? 1 : 0;
    d->magnetometer_aiding = (main_status & HIPNUC_STATUS_MAG_AIDING) ? 1 : 0;
    d->valid |= HIPNUC_VALID_STATUS;
}

/* Calendar and range check only. */
HIPNUC_INLINE int hn_utc_valid(const hipnuc_utc_t *u)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    unsigned limit;
    if (u->year < 1 || u->year > 9999 || u->month < 1 || u->month > 12 || u->hour > 23 || u->minute > 59 ||
        u->second > 60 || u->millisecond > 999) return 0;
    if (u->second == 60 && (u->hour != 23 || u->minute != 59)) return 0;
    limit = days[u->month - 1];
    if (u->month == 2 && u->year % 4 == 0 && (u->year % 100 != 0 || u->year % 400 == 0)) limit++;
    return u->day >= 1 && u->day <= limit;
}

/* Calendar time from the wire (year - 2000, ..., second * 1000 + ms). A zero
 * date means "no time yet" and is not UTC; nothing is stored then. */
HIPNUC_INLINE void hn_set_utc(hipnuc_data_t *d, uint8_t year2, uint8_t month, uint8_t day, uint8_t hour,
                              uint8_t minute, uint16_t sec_ms)
{
    hipnuc_utc_t u;
    if (year2 == 0 && month == 0 && day == 0) return;
    u.year = (uint16_t)(2000U + year2);
    u.month = month;
    u.day = day;
    u.hour = hour;
    u.minute = minute;
    u.second = (uint8_t)(sec_ms / 1000U);
    u.millisecond = (uint16_t)(sec_ms % 1000U);
    if (!hn_utc_valid(&u)) return;
    d->utc = u;
    d->valid |= HIPNUC_VALID_UTC | HIPNUC_VALID_UTC_TIME_OF_DAY;
}

#endif /* HIPNUC_INTERNAL_H */
