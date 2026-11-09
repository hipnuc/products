/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * hipnuc_data_merge(); see hipnuc_data.h.
 */

#include "hipnuc_data.h"

#include <stddef.h>
#include <string.h>

/* Bit numbers of the HIPNUC_VALID_* flags, in the same order. */
enum {
    BIT_STATUS,
    BIT_INS_STATUS,
    BIT_NODE_ID,
    BIT_SEQUENCE,
    BIT_ACC,
    BIT_GYR,
    BIT_MAG,
    BIT_ROLL_PITCH,
    BIT_YAW,
    BIT_HEADING,
    BIT_QUAT,
    BIT_INCLINATION,
    BIT_INCLINATION_YAW,
    BIT_ATT_STD,
    BIT_SYSTEM_TIME,
    BIT_DEVICE_TIME,
    BIT_UTC,
    BIT_UTC_TIME_OF_DAY,
    BIT_GPS_TIME,
    BIT_PRESSURE,
    BIT_TEMPERATURE,
    BIT_HEAVE,
    BIT_HEAVE_FREQUENCY,
    BIT_POSITION,
    BIT_ALTITUDE,
    BIT_VELOCITY_ENU,
    BIT_ACC_ENU,
    BIT_SOG,
    BIT_COG,
    BIT_GNSS_POSITION,
    BIT_GNSS_ALTITUDE,
    BIT_GNSS_VELOCITY,
    BIT_POSITION_QUALITY,
    BIT_POSITION_SATELLITES,
    BIT_HEADING_QUALITY,
    BIT_HEADING_SATELLITES,
    BIT_NMEA_STATUS,
    BIT_NMEA_MODE,
    BIT_PDOP,
    BIT_HDOP,
    BIT_DIFF_AGE,
    BIT_UNDULATION,
    BIT_COUNT
};
typedef char bit_count_matches[(HIPNUC_VALID_ALL == (UINT64_C(1) << BIT_COUNT) - 1U) ? 1 : -1];

/* The fields behind each validity bit (some bits cover several fields). The
 * size term fails to compile when BIT_x does not match HIPNUC_VALID_x. */
#define BIT_CHECK(bit) sizeof(char[(HIPNUC_VALID_##bit == (UINT64_C(1) << BIT_##bit)) ? 1 : -1])
#define FIELD(bit, member)                                                   \
    { BIT_##bit, (uint8_t)(sizeof(((hipnuc_data_t *)0)->member) * BIT_CHECK(bit)), \
      (uint16_t)offsetof(hipnuc_data_t, member) }

static const struct {
    uint8_t bit;      /* bit number in hipnuc_data_t.valid */
    uint8_t size;
    uint16_t offset;
} fields[] = {
    FIELD(STATUS, main_status),           FIELD(STATUS, gyro_bias_converged),
    FIELD(STATUS, attitude_converged),    FIELD(STATUS, magnetic_disturbance),
    FIELD(STATUS, device_static),         FIELD(STATUS, magnetometer_aiding),
    FIELD(INS_STATUS, ins_status),        FIELD(NODE_ID, node_id),
    FIELD(SEQUENCE, sequence),            FIELD(ACC, acc),
    FIELD(GYR, gyr),                      FIELD(MAG, mag),
    FIELD(ROLL_PITCH, roll),              FIELD(ROLL_PITCH, pitch),
    FIELD(YAW, yaw),                      FIELD(HEADING, heading),
    FIELD(QUAT, quat),                    FIELD(INCLINATION, inclination),
    FIELD(INCLINATION_YAW, inclination_yaw), FIELD(ATT_STD, att_std),
    FIELD(SYSTEM_TIME, system_time_ms),   FIELD(DEVICE_TIME, device_time_us),
    FIELD(UTC, utc.year),                 FIELD(UTC, utc.month),
    FIELD(UTC, utc.day),                  FIELD(UTC, utc.hour),
    FIELD(UTC, utc.minute),               FIELD(UTC, utc.second),
    FIELD(UTC, utc.millisecond),          FIELD(UTC_TIME_OF_DAY, utc.hour),
    FIELD(UTC_TIME_OF_DAY, utc.minute),   FIELD(UTC_TIME_OF_DAY, utc.second),
    FIELD(UTC_TIME_OF_DAY, utc.millisecond), FIELD(GPS_TIME, gps_week),
    FIELD(GPS_TIME, gps_tow_ms),          FIELD(PRESSURE, pressure),
    FIELD(TEMPERATURE, temperature),      FIELD(HEAVE, heave_m),
    FIELD(HEAVE_FREQUENCY, heave_hz),     FIELD(POSITION, longitude),
    FIELD(POSITION, latitude),            FIELD(ALTITUDE, altitude_msl),
    FIELD(VELOCITY_ENU, vel_enu),         FIELD(ACC_ENU, acc_enu),
    FIELD(SOG, sog),                      FIELD(COG, cog),
    FIELD(GNSS_POSITION, gnss_longitude), FIELD(GNSS_POSITION, gnss_latitude),
    FIELD(GNSS_ALTITUDE, gnss_altitude_msl), FIELD(GNSS_VELOCITY, gnss_vel_enu),
    FIELD(POSITION_QUALITY, position_quality), FIELD(POSITION_SATELLITES, position_satellites),
    FIELD(HEADING_QUALITY, heading_quality),   FIELD(HEADING_SATELLITES, heading_satellites),
    FIELD(NMEA_STATUS, nmea_status),      FIELD(NMEA_MODE, nmea_mode),
    FIELD(PDOP, pdop),                    FIELD(HDOP, hdop),
    FIELD(DIFF_AGE, diff_age),            FIELD(UNDULATION, undulation),
};

void hipnuc_data_merge(hipnuc_data_t *dst, const hipnuc_data_t *src)
{
    size_t i;
    if (!dst || !src || dst == src) return;
    for (i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        if (src->valid & (UINT64_C(1) << fields[i].bit)) {
            memcpy((char *)dst + fields[i].offset, (const char *)src + fields[i].offset, fields[i].size);
        }
    }
    dst->valid |= src->valid;
    dst->source = src->source;
}
