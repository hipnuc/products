/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "nmea_dec.h"
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#define MAXFIELD 64 /* Maximum number of fields in a NMEA sentence */

/* Synchronize NMEA sentence */
static int sync_nmea(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == '$' && (buf[1] == 'G' || buf[1] == 'P'); /* G for GPS, P for proprietary */
}

/* Convert degrees and decimal minutes to decimal degrees */
static double dmm2deg(double dmm)
{
    int deg = (int)(dmm / 100.0);
    double min = dmm - deg * 100.0;
    return deg + min / 60.0;
}

/* Separate time into hours, minutes, and seconds */
static void septime(double t, double *t1, double *t2, double *t3)
{
    *t1 = floor(t / 10000.0);
    t -= *t1 * 10000.0;
    *t2 = floor(t / 100.0);
    *t3 = t - *t2 * 100.0;
}

/* Decode NMEA GGA (Global Positioning System Fix Data) sentence */
static int dec_gga(nmea_raw_t *raw, char **val, int n)
{
    double tod = 0.0, lat = 0.0, lon = 0.0, hdop = 0.0, msl = 0.0, undulation = 0.0;
    double age = 0.0;
    char ns = 'N', ew = 'E';
    int solq = 0, nrcv = 0;
    int i;

    for (i = 0; i < n; i++)
    {
        switch (i)
        {
        case 0: tod = atof(val[i]); break;
        case 1: lat = atof(val[i]); break;
        case 2: ns = val[i][0]; break;
        case 3: lon = atof(val[i]); break;
        case 4: ew = val[i][0]; break;
        case 5: solq = atoi(val[i]); break;
        case 6: nrcv = atoi(val[i]); break;
        case 7: hdop = atof(val[i]); break;
        case 8: msl = atof(val[i]); break;
        case 10: undulation = atof(val[i]); break;
        case 12: age = atof(val[i]); break;
        }
    }

    raw->gga.lat = (ns == 'N' ? 1.0 : -1.0) * dmm2deg(lat);
    raw->gga.lon = (ew == 'E' ? 1.0 : -1.0) * dmm2deg(lon);
    raw->gga.msl = msl;
    raw->gga.undulation = undulation;
    raw->gga.nv = nrcv;
    raw->gga.status = solq;
    raw->gga.hdop = hdop;
    raw->gga.diff_age = age;

    septime(tod, &raw->gga.hour, &raw->gga.min, &raw->gga.sec);
    return 1;
}


/* Decode NMEA RMC (Recommended Minimum Specific GNSS Data) sentence */
static int dec_rmc(nmea_raw_t *raw, char **val, int n)
{
    double tod = 0.0, lat = 0.0, lon = 0.0, vel = 0.0, dir = 0.0, date = 0.0;
    char ns = 'N', ew = 'E', mode = 'A';
    int i;

    for (i = 0; i < n; i++)
    {
        switch (i)
        {
        case 0: tod = atof(val[i]); break;
        case 2: lat = atof(val[i]); break;
        case 3: ns = val[i][0]; break;
        case 4: lon = atof(val[i]); break;
        case 5: ew = val[i][0]; break;
        case 6: vel = atof(val[i]); break;
        case 7: dir = atof(val[i]); break;
        case 8: date = atof(val[i]); break;
        case 11: mode = val[i][0]; break;
        }
    }

    // Convert and store data in raw structure
    raw->rmc.lat = (ns == 'N' ? 1.0 : -1.0) * dmm2deg(lat);
    raw->rmc.lon = (ew == 'E' ? 1.0 : -1.0) * dmm2deg(lon);
    raw->rmc.mode = mode;
    raw->rmc.sog = vel;
    raw->rmc.cog = dir;

    // Parse date and time
    septime(date, &raw->rmc.day, &raw->rmc.month, &raw->rmc.year);
    septime(tod, &raw->rmc.hour, &raw->rmc.min, &raw->rmc.sec);
    raw->rmc.year += raw->rmc.year < 80.0 ? 2000.0 : 1900.0;
    return 1;
}

/* Decode NMEA SXT (proprietary) sentence */
static int dec_sxt(nmea_raw_t *raw, char **val, int n)
{
    double date = 0.0, lon = 0.0, lat = 0.0, msl = 0.0,
           yaw = 0.0, pitch = 0.0, speed_heading = 0.0,
           ground_speed = 0.0, roll = 0.0,
           gyr_x = 0.0, gyr_y = 0.0, gyr_z = 0.0,
           ve = 0.0, vn = 0.0, vu = 0.0;
    uint8_t solq = 0, solq_heading = 0, nv = 0, nv_heading = 0;
    int i;

    for (i = 0; i < n && i < 21; i++)
    {
        switch (i)
        {
        case 0: date = atof(val[i]); break;
        case 1: lon = atof(val[i]); break;
        case 2: lat = atof(val[i]); break;
        case 3: msl = atof(val[i]); break;
        case 4: yaw = atof(val[i]); break;
        case 5: pitch = atof(val[i]); break;
        case 6: speed_heading = atof(val[i]); break;
        case 7: ground_speed = atof(val[i]); break;
        case 8: roll = atof(val[i]); break;
        case 9: solq = atoi(val[i]); break;
        case 10: solq_heading = atoi(val[i]); break;
        case 11: nv = atoi(val[i]); break;
        case 12: nv_heading = atoi(val[i]); break;
        case 13: gyr_x = atof(val[i]); break;
        case 14: gyr_y = atof(val[i]); break;
        case 15: gyr_z = atof(val[i]); break;
        case 16: ve = atof(val[i]); break;
        case 17: vn = atof(val[i]); break;
        case 18: vu = atof(val[i]); break;
        case 19: raw->sxt.ins_stat = atoi(val[i]); break;
        case 20: raw->sxt.ant_stat = atoi(val[i]); break;
        }
    }

    // Parse date and time
    raw->sxt.year = floor(date / 10000000000);
    raw->sxt.month = floor((date - raw->sxt.year * 10000000000) / 100000000);
    raw->sxt.day = floor((date - raw->sxt.year * 10000000000 - raw->sxt.month * 100000000) / 1000000);
    septime(date - floor(date / 1000000) * 1000000, &raw->sxt.hour, &raw->sxt.min, &raw->sxt.sec);

    // Store parsed data in raw structure
    raw->sxt.lon = lon;
    raw->sxt.lat = lat;
    raw->sxt.msl = msl;
    raw->sxt.yaw = yaw;
    raw->sxt.pitch = pitch;
    raw->sxt.speed_heading = speed_heading;
    raw->sxt.ground_speed = ground_speed;
    raw->sxt.solq = solq;
    raw->sxt.solq_heading = solq_heading;
    raw->sxt.nv_heading = nv_heading;
    raw->sxt.nv = nv;
    raw->sxt.roll = roll;
    raw->sxt.gyr_x = gyr_x;
    raw->sxt.gyr_y = gyr_y;
    raw->sxt.gyr_z = gyr_z;
    raw->sxt.ve = ve;
    raw->sxt.vn = vn;
    raw->sxt.vu = vu;

    return 1;
}


/* Parse NMEA sentence */
static int parse_nmea(nmea_raw_t *raw)
{
    char *p, *q, *val[MAXFIELD], sum, sum2;
    int n = 0;

    /* checksum */
    for (q = (char *)raw->buf + 1, sum = 0; *q && *q != '*'; q++)
        sum ^= *q;
    q++;
    sum2 = (int)strtol(q, NULL, 16);

    if (sum != sum2)
    {
        // NL_DEBUG_LOG(NL_DEBUG_RCV, ("nmea checksum failed %02X %02X\n",sum, sum2));
        return 0;
    }

    /* Parse fields */
    for (p = (char *)raw->buf; *p && n < MAXFIELD; p = q + 1)
    {
        if (((q = strchr(p, ',')) != NULL) || ((q = strchr(p, '*')) != NULL))
        {
            val[n++] = p;
            *q = '\0';
        }
        else
            break;
    }
    if (n < 1)
    {
        return 0;
    }

    /* Copy NMEA header */
    memcpy(raw->nmea_header, raw->buf + 1, 5);
    raw->nmea_header[5] = '\0';

    if (strstr(raw->nmea_header, "GGA"))
    {
        raw->msg_type = NMEA_DEC_MSG_GGA;
        return dec_gga(raw, val + 1, n - 1);
    }
    if (strstr(raw->nmea_header, "RMC"))
    {
        raw->msg_type = NMEA_DEC_MSG_RMC;
        return dec_rmc(raw, val + 1, n - 1);
    }
    if (strstr(raw->nmea_header, "SXT"))
    {
        raw->msg_type = NMEA_DEC_MSG_SXT;
        return dec_sxt(raw, val + 1, n - 1);
    }

    return 0;
}

/* Input function for NMEA decoder */
int input_nmea(nmea_raw_t *raw, uint8_t data)
{
    /* Synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_nmea(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    if (raw->nbyte >= MAXRAWLEN)
    {
        raw->nbyte = 0;
        return -1;
    }

    raw->buf[raw->nbyte++] = data;

    if (data == '\n')
    {
        raw->len = raw->nbyte;
        raw->nbyte = 0;
        return parse_nmea(raw);
    }

    return 0;
}


int nmea_dec_dump_msg(nmea_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    ret = snprintf(buf + written, buf_size - written, "{\n  \"nmea_header\": \"%s\",\n", raw->nmea_header);
    if (ret > 0) written += ret;

    if (raw->msg_type == NMEA_DEC_MSG_GGA)
    {
        ret = snprintf(buf + written, buf_size - written,
            "  \"msg_type\": \"GGA\",\n"
            "  \"utc_time\": \"%02.0f:%02.0f:%05.2f\",\n"
            "  \"latitude\": %.7f,\n"
            "  \"latitude_dir\": \"%c\",\n"
            "  \"longitude\": %.7f,\n"
            "  \"longitude_dir\": \"%c\",\n"
            "  \"altitude_msl\": %.2f,\n"
            "  \"satellites\": %d,\n"
            "  \"hdop\": %.1f,\n"
            "  \"geoid_separation\": %.2f,\n"
            "  \"differential_age\": %.1f,\n"
            "  \"solution_quality\": %d\n",
            raw->gga.hour, raw->gga.min, raw->gga.sec,
            fabs(raw->gga.lat),
            raw->gga.lat >= 0 ? 'N' : 'S',
            fabs(raw->gga.lon),
            raw->gga.lon >= 0 ? 'E' : 'W',
            raw->gga.msl,
            raw->gga.nv,
            raw->gga.hdop,
            raw->gga.undulation,
            raw->gga.diff_age,
            raw->gga.status);
    }
    else if (raw->msg_type == NMEA_DEC_MSG_RMC)
    {
        ret = snprintf(buf + written, buf_size - written,
            "  \"msg_type\": \"RMC\",\n"
            "  \"utc\": \"%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%05.2fZ\",\n"
            "  \"latitude\": %.7f,\n"
            "  \"latitude_dir\": \"%c\",\n"
            "  \"longitude\": %.7f,\n"
            "  \"longitude_dir\": \"%c\",\n"
            "  \"speed_over_ground\": %.2f,\n"
            "  \"course_over_ground\": %.2f,\n"
            "  \"mode\": \"%c\"\n",
            raw->rmc.year, raw->rmc.month, raw->rmc.day,
            raw->rmc.hour, raw->rmc.min, raw->rmc.sec,
            fabs(raw->rmc.lat),
            raw->rmc.lat >= 0 ? 'N' : 'S',
            fabs(raw->rmc.lon),
            raw->rmc.lon >= 0 ? 'E' : 'W',
            raw->rmc.sog,
            raw->rmc.cog,
            raw->rmc.mode);
    }
    else if (raw->msg_type == NMEA_DEC_MSG_SXT)
    {
        ret = snprintf(buf + written, buf_size - written,
            "  \"msg_type\": \"SXT\",\n"
            "  \"utc\": \"%04.0f-%02.0f-%02.0fT%02.0f:%02.0f:%05.2fZ\",\n"
            "  \"latitude\": %.7f,\n"
            "  \"longitude\": %.7f,\n"
            "  \"altitude_msl\": %.2f,\n"
            "  \"yaw\": %.2f,\n"
            "  \"pitch\": %.2f,\n"
            "  \"roll\": %.2f,\n"
            "  \"speed_heading\": %.2f,\n"
            "  \"ground_speed\": %.2f,\n"
            "  \"position_solution_quality\": %d,\n"
            "  \"heading_solution_quality\": %d,\n"
            "  \"satellites_position\": %d,\n"
            "  \"satellites_heading\": %d,\n"
            "  \"gyroscope\": {\n"
            "    \"x\": %.3f,\n"
            "    \"y\": %.3f,\n"
            "    \"z\": %.3f\n"
            "  },\n"
            "  \"velocity\": {\n"
            "    \"e\": %.3f,\n"
            "    \"n\": %.3f,\n"
            "    \"u\": %.3f\n"
            "  },\n"
            "  \"ins_status\": %d,\n"
            "  \"antenna_status\": %d\n",
            raw->sxt.year, raw->sxt.month, raw->sxt.day,
            raw->sxt.hour, raw->sxt.min, raw->sxt.sec,
            raw->sxt.lat,
            raw->sxt.lon,
            raw->sxt.msl,
            raw->sxt.yaw,
            raw->sxt.pitch,
            raw->sxt.roll,
            raw->sxt.speed_heading,
            raw->sxt.ground_speed,
            raw->sxt.solq,
            raw->sxt.solq_heading,
            raw->sxt.nv,
            raw->sxt.nv_heading,
            raw->sxt.gyr_x, raw->sxt.gyr_y, raw->sxt.gyr_z,
            raw->sxt.ve, raw->sxt.vn, raw->sxt.vu,
            raw->sxt.ins_stat,
            raw->sxt.ant_stat);
    }

    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "}\n");
    if (ret > 0) written += ret;

    return written;
}



