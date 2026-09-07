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
    s->valid |= HIPNUC_VALID_UTC;
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
    s->valid |= HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG | HIPNUC_VALID_EULER |
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
                HIPNUC_VALID_EULER | HIPNUC_VALID_HEADING | HIPNUC_VALID_QUAT |
                HIPNUC_VALID_POSITION | HIPNUC_VALID_DOP | HIPNUC_VALID_GNSS_QUALITY |
                HIPNUC_VALID_DIFF_AGE | HIPNUC_VALID_UNDULATION | HIPNUC_VALID_VELOCITY_ENU |
                HIPNUC_VALID_ACC_ENU;
    if (p->gpst_wn || p->gpst_tow) s->valid |= HIPNUC_VALID_GPS_TIME;
    /* HI81 carries the INS heading; expose it as yaw too so the Euler triple is complete. */
    s->yaw = s->heading;
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

    if (bm & HI83_BMAP_ACC_B) { for (i = 0; i < 3; ++i) s->acc[i] = p->acc_b[i]; s->valid |= HIPNUC_VALID_ACC; }
    if (bm & HI83_BMAP_GYR_B) { for (i = 0; i < 3; ++i) s->gyr[i] = p->gyr_b[i]; s->valid |= HIPNUC_VALID_GYR; }
    if (bm & HI83_BMAP_MAG_B) { for (i = 0; i < 3; ++i) s->mag[i] = p->mag_b[i] * 1e-6f; s->valid |= HIPNUC_VALID_MAG; }
    if (bm & HI83_BMAP_RPY) {
        s->roll = p->rpy[0] * HIPNUC_DEG2RAD;
        s->pitch = p->rpy[1] * HIPNUC_DEG2RAD;
        s->yaw = p->rpy[2] * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_EULER;
    }
    if (bm & HI83_BMAP_QUAT) { for (i = 0; i < 4; ++i) s->quat[i] = p->quat[i]; s->valid |= HIPNUC_VALID_QUAT; }
    if (bm & HI83_BMAP_SYSTEM_TIME) { s->device_time_us = p->system_time_us; s->valid |= HIPNUC_VALID_DEVICE_TIME; }
    if (bm & HI83_BMAP_UTC) {
        set_utc(s, p->utc.year, p->utc.month, p->utc.day, p->utc.hour, p->utc.min, p->utc.sec_ms,
                !(p->main_status & HIPNUC_STATUS_UTC_UNSYNC));
    }
    if (bm & HI83_BMAP_AIR_PRESSURE) { s->pressure = p->air_pressure; s->valid |= HIPNUC_VALID_PRESSURE; }
    if (bm & HI83_BMAP_TEMPERATURE) { s->temperature = p->temperature; s->valid |= HIPNUC_VALID_TEMPERATURE; }
    if (bm & HI83_BMAP_INCLINATION) {
        s->inclination[0] = p->inclination[0] * HIPNUC_DEG2RAD;
        s->inclination[1] = p->inclination[1] * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_INCLINATION;
    }
    if (bm & HI83_BMAP_HSS) {
        for (i = 0; i < 3; ++i) s->heave_m[i] = p->hss[i];
        if (bm & HI83_BMAP_HSS_FRQ) for (i = 0; i < 3; ++i) s->heave_hz[i] = p->hss_frq[i];
        s->valid |= HIPNUC_VALID_HEAVE;
    }
    if (bm & HI83_BMAP_VEL_ENU) { for (i = 0; i < 3; ++i) s->vel_enu[i] = p->vel_enu[i]; s->valid |= HIPNUC_VALID_VELOCITY_ENU; }
    if (bm & HI83_BMAP_ACC_ENU) { for (i = 0; i < 3; ++i) s->acc_enu[i] = p->acc_enu[i]; s->valid |= HIPNUC_VALID_ACC_ENU; }
    if (bm & HI83_BMAP_INS_LON_LAT_MSL) {
        s->longitude = p->ins_lon_lat_msl[0];
        s->latitude = p->ins_lon_lat_msl[1];
        s->altitude_msl = p->ins_lon_lat_msl[2];
        s->valid |= HIPNUC_VALID_POSITION;
    }
    if (bm & HI83_BMAP_GNSS_QUALITY_NV) {
        s->position_quality = p->solq_pos;
        s->position_satellites = p->nv_pos;
        s->heading_quality = p->solq_heading;
        s->heading_satellites = p->nv_heading;
        s->valid |= HIPNUC_VALID_GNSS_QUALITY;
    }
    if (bm & HI83_BMAP_OD_SPEED) { s->odometer_speed = p->od_speed; s->valid |= HIPNUC_VALID_ODOMETER; }
    if (bm & HI83_BMAP_UNDULATION) { s->undulation = p->undulation; s->valid |= HIPNUC_VALID_UNDULATION; }
    if (bm & HI83_BMAP_DIFF_AGE) { s->diff_age = p->diff_age; s->valid |= HIPNUC_VALID_DIFF_AGE; }
    if (bm & HI83_BMAP_NODE_ID) { s->node_id = p->node.node_id; s->valid |= HIPNUC_VALID_NODE_ID; }
    if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) {
        s->gnss_longitude = p->gnss_lon_lat_msl[0];
        s->gnss_latitude = p->gnss_lon_lat_msl[1];
        s->gnss_altitude_msl = p->gnss_lon_lat_msl[2];
        s->valid |= HIPNUC_VALID_GNSS_POSITION;
    }
    if (bm & HI83_BMAP_GNSS_VEL) { for (i = 0; i < 3; ++i) s->gnss_vel_enu[i] = p->gnss_vel[i]; s->valid |= HIPNUC_VALID_GNSS_VELOCITY; }
}

int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *s)
{
    if (raw->hi83.tag == HIPNUC_ID_HI83) { hipnuc_sample_from_hi83(&raw->hi83, s); return 1; }
    if (raw->hi81.tag == HIPNUC_ID_HI81) { hipnuc_sample_from_hi81(&raw->hi81, s); return 1; }
    if (raw->hi91.tag == HIPNUC_ID_HI91) { hipnuc_sample_from_hi91(&raw->hi91, s); return 1; }
    return 0;
}

void hipnuc_sample_from_gga(const nmea_gga_t *g, hipnuc_sample_t *s)
{
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_NMEA_GGA;
    s->position_quality = g->quality;
    s->position_satellites = g->satellites;
    s->hdop = g->hdop;
    s->valid |= HIPNUC_VALID_GNSS_QUALITY | HIPNUC_VALID_DOP;
    if (g->has_position && g->has_altitude) {
        s->longitude = g->lon;
        s->latitude = g->lat;
        s->altitude_msl = g->altitude_msl;
        s->valid |= HIPNUC_VALID_POSITION;
    }
    if (g->has_undulation) { s->undulation = g->undulation; s->valid |= HIPNUC_VALID_UNDULATION; }
    if (g->has_diff_age) { s->diff_age = g->diff_age; s->valid |= HIPNUC_VALID_DIFF_AGE; }
    if (g->has_time) {
        /* Time of day only: the date is unknown, so UTC stays invalid. */
        s->utc.hour = g->hour;
        s->utc.minute = g->minute;
        s->utc.second = (uint8_t)g->second;
        s->utc.millisecond = (uint16_t)((g->second - (int)g->second) * 1000.0f + 0.5f);
    }
}

void hipnuc_sample_from_rmc(const nmea_rmc_t *r, hipnuc_sample_t *s)
{
    hipnuc_sample_clear(s);
    s->source = HIPNUC_SOURCE_NMEA_RMC;
    if (r->has_position && r->status == 'A') {
        s->longitude = r->lon;
        s->latitude = r->lat;
        s->valid |= HIPNUC_VALID_POSITION;   /* altitude_msl stays 0: RMC has no height */
    }
    if (r->has_sog && r->has_cog) {
        s->sog = r->sog * HIPNUC_KNOT2MPS;
        s->cog = r->cog * HIPNUC_DEG2RAD;
        s->valid |= HIPNUC_VALID_SOG_COG;
    }
    if (r->has_date && r->has_time) {
        s->utc.year = r->year;
        s->utc.month = r->month;
        s->utc.day = r->day;
        s->utc.hour = r->hour;
        s->utc.minute = r->minute;
        s->utc.second = (uint8_t)r->second;
        s->utc.millisecond = (uint16_t)((r->second - (int)r->second) * 1000.0f + 0.5f);
        s->valid |= HIPNUC_VALID_UTC;
    }
}

int hipnuc_sample_from_nmea(const nmea_raw_t *raw, hipnuc_sample_t *s)
{
    if (raw->msg_type == NMEA_MSG_GGA) { hipnuc_sample_from_gga(&raw->gga, s); return 1; }
    if (raw->msg_type == NMEA_MSG_RMC) { hipnuc_sample_from_rmc(&raw->rmc, s); return 1; }
    return 0;
}
