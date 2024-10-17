/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef __NMEA_DEC_H__
#define __NMEA_DEC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Maximum length of raw NMEA message */
#ifndef MAXRAWLEN
#define MAXRAWLEN 512
#endif

/** NMEA message types */
typedef enum {
    NMEA_DEC_MSG_GGA = 'G',  /**< Global Positioning System Fix Data */
    NMEA_DEC_MSG_RMC = 'R',  /**< Recommended Minimum Specific GNSS Data */
    NMEA_DEC_MSG_SXT = 'S'   /**< Proprietary NMEA sentence */
} nmea_msg_type_t;

/**
 * @brief Structure to hold GGA (Global Positioning System Fix Data) message data
 * 
 * GGA message provides essential fix data which include 3D location and accuracy data.
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 */
typedef struct {
    double hour;          /**< Hour of position fix (UTC) */
    double min;           /**< Minute of position fix (UTC) */
    double sec;           /**< Second of position fix (UTC) */
    double lat;           /**< Latitude in decimal degrees (North is positive) */
    double lon;           /**< Longitude in decimal degrees (East is positive) */
    double msl;           /**< Altitude above mean sea level in meters */
    uint8_t nv;           /**< Number of satellites used in position fix */
    uint8_t status;       /**< GPS quality indicator (0=No fix, 1=GPS fix, 2=DGPS fix) */
    float hdop;           /**< Horizontal Dilution of Precision */
    float undulation;     /**< Geoidal separation in meters (difference between WGS-84 earth ellipsoid and mean sea level) */
    float diff_age;       /**< Age of differential corrections in seconds */
    uint32_t sta_id;      /**< Differential base station ID */
} nmea_gga_t;

/**
 * @brief Structure to hold RMC (Recommended Minimum Specific GNSS Data) message data
 * 
 * RMC message provides the minimum recommended data for GPS.
 * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
 */
typedef struct {
    double year;          /**< Year (UTC) */
    double month;         /**< Month (UTC) */
    double day;           /**< Day (UTC) */
    double hour;          /**< Hour (UTC) */
    double min;           /**< Minute (UTC) */
    double sec;           /**< Second (UTC) */
    uint8_t mode;         /**< Mode indicator (A=Autonomous, D=Differential, E=Estimated, N=Data not valid) */
    double lat;           /**< Latitude in decimal degrees (North is positive) */
    double lon;           /**< Longitude in decimal degrees (East is positive) */
    float sog;            /**< Speed over ground in knots */
    float cog;            /**< Course over ground in degrees (true) */
} nmea_rmc_t;

/**
 * @brief Structure to hold SXT (proprietary NMEA sentence) message data
 * 
 * SXT is a proprietary NMEA sentence that includes INS data beyond standard NMEA messages.
 */
typedef struct {
    double year, month, day;
    double hour, min, sec;
    double lon;           /**< Longitude in decimal degrees (East is positive) */
    double lat;           /**< Latitude in decimal degrees (North is positive) */
    double msl;           /**< Altitude above mean sea level in meters */
    double yaw;           /**< Yaw angle in degrees */
    double pitch;         /**< Pitch angle in degrees */
    double speed_heading; /**< Speed heading angle in degrees */
    double ground_speed;  /**< Ground speed in meters per second */
    uint8_t solq;         /**< Solution quality (aligned with GGA's indicator) */
    uint8_t solq_heading; /**< Heading solution quality */
    uint8_t nv_heading;   /**< Number of satellites used for heading determination */
    uint8_t nv;           /**< Number of satellites used in position fix */
    double roll;          /**< Roll angle in degrees */
    double gyr_x;         /**< Gyroscope X-axis angular rate in degrees per second */
    double gyr_y;         /**< Gyroscope Y-axis angular rate in degrees per second */
    double gyr_z;         /**< Gyroscope Z-axis angular rate in degrees per second */
    double ve;            /**< Velocity in East direction in meters per second */
    double vn;            /**< Velocity in North direction in meters per second */
    double vu;            /**< Velocity in Up direction in meters per second */
    uint8_t ins_stat;     /**< INS status (aligned with ins->ins_stat) */
    uint8_t ant_stat;     /**< Antenna status */
} nmea_sxt_t;

/**
 * @brief Structure to hold raw NMEA message data and parsed results
 */
typedef struct {
    int32_t nbyte;        /**< Number of bytes in message buffer */
    int32_t len;          /**< Length of the message */
    uint8_t buf[MAXRAWLEN]; /**< Raw message buffer */
    nmea_msg_type_t msg_type; /**< Type of NMEA message */
    char nmea_header[8];  /**< NMEA message header (e.g., "$GPGGA") */
    nmea_gga_t gga;       /**< Parsed GGA message data */
    nmea_rmc_t rmc;       /**< Parsed RMC message data */
    nmea_sxt_t sxt;       /**< Parsed SXT message data */
} nmea_raw_t;

/**
 * @brief Input function for NMEA decoder
 * 
 * This function processes incoming NMEA data byte by byte. It accumulates bytes
 * until a complete NMEA sentence is received, then parses the sentence.
 *
 * @param raw Pointer to nmea_raw_t structure to store parsed data
 * @param data Input byte
 * @return 1 if a complete NMEA message is parsed, 0 if more data is needed, -1 on error
 */
int input_nmea(nmea_raw_t *raw, uint8_t data);

/**
 * @brief Dump parsed NMEA message to a string
 * 
 * This function formats the parsed NMEA data into a human-readable string.
 *
 * @param raw Pointer to nmea_raw_t structure containing parsed data
 * @param buf Buffer to store the formatted string
 * @param buf_size Size of the buffer
 * @return Number of characters written to the buffer, or -1 on error
 */
int nmea_dec_dump_msg(nmea_raw_t *raw, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __NMEA_DEC_H__ */
