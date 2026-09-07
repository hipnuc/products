/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NMEA 0183 decoder for the sentences emitted by HiPNUC INS products:
 * GGA and RMC. Portable C99, no dynamic memory, no stdio, no global state.
 * Copy nmea_dec.c and nmea_dec.h into your project.
 */

#ifndef NMEA_DEC_H
#define NMEA_DEC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NMEA_MAX_RAW_SIZE 128   /* longest sentence + terminator */

typedef enum {
    NMEA_MSG_NONE = 0,
    NMEA_MSG_GGA,   /* Global Positioning System Fix Data */
    NMEA_MSG_RMC    /* Recommended Minimum Specific GNSS Data */
} nmea_msg_type_t;

/**
 * $GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 * Empty fields keep the value 0 and clear the corresponding has_* flag.
 */
typedef struct {
    uint8_t  hour;            /* UTC time of day */
    uint8_t  minute;
    float    second;
    uint8_t  has_time;
    double   lat;             /* deg, north positive */
    double   lon;             /* deg, east positive */
    uint8_t  has_position;    /* both coordinates present */
    uint8_t  quality;         /* 0 = no fix, 1 = GNSS, 2 = differential, 4 = RTK fixed, 5 = RTK float */
    uint8_t  satellites;      /* satellites used */
    float    hdop;
    double   altitude_msl;    /* m above mean sea level */
    uint8_t  has_altitude;
    float    undulation;      /* m, geoid separation (ellipsoid = msl + undulation) */
    uint8_t  has_undulation;
    float    diff_age;        /* s, age of differential corrections */
    uint8_t  has_diff_age;
    uint16_t station_id;      /* differential station */
} nmea_gga_t;

/**
 * $GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,A*XX
 */
typedef struct {
    uint16_t year;            /* UTC date, four digits */
    uint8_t  month;
    uint8_t  day;
    uint8_t  has_date;
    uint8_t  hour;
    uint8_t  minute;
    float    second;
    uint8_t  has_time;
    char     status;          /* 'A' = valid, 'V' = void */
    double   lat;             /* deg, north positive */
    double   lon;             /* deg, east positive */
    uint8_t  has_position;
    float    sog;             /* knots, speed over ground */
    uint8_t  has_sog;
    float    cog;             /* deg true, course over ground */
    uint8_t  has_cog;
    char     mode;            /* A autonomous, D differential, R RTK fixed, F RTK float, E estimated, N not valid */
} nmea_rmc_t;

/**
 * Decoder state. Zero-initialize before first use; one per serial port.
 */
typedef struct {
    int             nbyte;                   /* bytes currently in buf */
    uint8_t         buf[NMEA_MAX_RAW_SIZE];  /* current sentence, NUL terminated after decode */
    nmea_msg_type_t msg_type;                /* sentence decoded by the last successful call */
    char            talker[3];               /* e.g. "GP", "GN" */
    nmea_gga_t      gga;
    nmea_rmc_t      rmc;
    uint32_t        sentence_count;          /* GGA/RMC sentences decoded */
    uint32_t        checksum_error_count;
} nmea_raw_t;

/**
 * Feed one received byte.
 *
 * @return 1 when a GGA or RMC sentence was decoded (see raw->msg_type),
 *         0 when more bytes are needed or the sentence type is not decoded,
 *         -1 on checksum or framing error. Test `> 0`.
 */
int nmea_input(nmea_raw_t *raw, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* NMEA_DEC_H */
