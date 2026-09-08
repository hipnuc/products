/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire packet -> SI sample conversion. See hipnuc_sample.h.
 */

#include "hipnuc_sample.h"

#include <string.h>

void hipnuc_sample_clear(hipnuc_sample_t *s)
{
    memset(s, 0, sizeof(*s));
}

void hipnuc_sample_set_status(hipnuc_sample_t *s, uint16_t main_status)
{
    s->main_status = main_status;
    s->gyro_bias_converged = (main_status & HIPNUC_STATUS_WB_CONV) ? 0 : 1;
    s->attitude_converged = (main_status & HIPNUC_STATUS_ATT_CONV) ? 0 : 1;
    s->magnetic_disturbance = (main_status & HIPNUC_STATUS_MAG_DIST) ? 1 : 0;
    s->device_static = (main_status & HIPNUC_STATUS_STATIC) ? 1 : 0;
    s->magnetometer_aiding = (main_status & HIPNUC_STATUS_MAG_AIDING) ? 1 : 0;
    s->valid |= HIPNUC_VALID_STATUS;
}

int hipnuc_utc_is_valid(const hipnuc_utc_t *utc)
{
    static const uint8_t days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    unsigned limit;
    if (utc->year < 1 || utc->year > 9999 || utc->month < 1 || utc->month > 12 ||
        utc->hour > 23 || utc->minute > 59 || utc->second > 60 || utc->millisecond > 999) return 0;
    if (utc->second == 60 && (utc->hour != 23 || utc->minute != 59)) return 0;
    limit = days[utc->month - 1];
    if (utc->month == 2 && utc->year % 4 == 0 && (utc->year % 100 != 0 || utc->year % 400 == 0)) limit++;
    return utc->day >= 1 && utc->day <= limit;
}
