/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * hipnuc_sample_t: one decoded measurement in SI units with validity flags.
 *
 * Every consumer of a HiPNUC device (JSON output, ROS nodes, MCU
 * applications) works from this structure instead of the wire packets, so
 * unit conversion happens in exactly one place. Fields are only meaningful
 * when the matching HIPNUC_VALID_* bit is set in `valid`. These bits mean
 * field availability, not convergence or a valid navigation fix. Check the
 * separately reported status/quality. Every converter clears the sample.
 *
 * Conventions: acceleration is specific force (gravity is not removed);
 * Euler angles and the quaternion follow the device coordinate
 * configuration (ENU / 312 by default); `heading_rad` (HI81, J1939 yaw) is
 * clockwise from north; positions are WGS84 with altitude above mean sea
 * level; velocities are east, north, up.
 */

#ifndef HIPNUC_SAMPLE_H
#define HIPNUC_SAMPLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Product wire conventions shared by every converter. */
#define HIPNUC_GRAVITY          9.8f                    /* 1 G on the wire = 9.8 m/s^2 */
#define HIPNUC_DEG2RAD          0.017453292519943295f
#define HIPNUC_KNOT2MPS         0.514444444f

/* Product MAIN_STATUS bits; a set convergence bit is a warning. */
#define HIPNUC_STATUS_WB_CONV      (1U << 3)
#define HIPNUC_STATUS_MAG_DIST     (1U << 4)
#define HIPNUC_STATUS_ACC_SAT      (1U << 5)
#define HIPNUC_STATUS_GYR_SAT      (1U << 6)
#define HIPNUC_STATUS_ATT_CONV     (1U << 7)
#define HIPNUC_STATUS_STATIC       (1U << 9)
#define HIPNUC_STATUS_MAG_AIDING   (1U << 10)
#define HIPNUC_STATUS_UTC_UNSYNC   (1U << 11)
#define HIPNUC_STATUS_SOUT_PULSE   (1U << 12)

#define HIPNUC_INS_INVALID          0
#define HIPNUC_INS_ALIGNING         1
#define HIPNUC_INS_NAVIGATING       3
#define HIPNUC_INS_DEAD_RECKONING   6

/* Where a sample came from */
typedef enum {
    HIPNUC_SOURCE_NONE = 0,
    HIPNUC_SOURCE_HI91,
    HIPNUC_SOURCE_HI81,
    HIPNUC_SOURCE_HI83,
    HIPNUC_SOURCE_NMEA_GGA,
    HIPNUC_SOURCE_NMEA_RMC,
    HIPNUC_SOURCE_J1939,
    HIPNUC_SOURCE_CANFD83
} hipnuc_source_t;

/* Validity bits for hipnuc_sample_t.valid */
#define HIPNUC_VALID_STATUS             (UINT64_C(1) << 0)
#define HIPNUC_VALID_INS_STATUS         (UINT64_C(1) << 1)
#define HIPNUC_VALID_ACC                (UINT64_C(1) << 2)
#define HIPNUC_VALID_GYR                (UINT64_C(1) << 3)
#define HIPNUC_VALID_MAG                (UINT64_C(1) << 4)
#define HIPNUC_VALID_ROLL_PITCH         (UINT64_C(1) << 5)
#define HIPNUC_VALID_HEADING            (UINT64_C(1) << 6)
#define HIPNUC_VALID_QUAT               (UINT64_C(1) << 7)
#define HIPNUC_VALID_DEVICE_TIME        (UINT64_C(1) << 8)
#define HIPNUC_VALID_UTC                (UINT64_C(1) << 9)  /* synchronized date and time */
#define HIPNUC_VALID_GPS_TIME           (UINT64_C(1) << 10)
#define HIPNUC_VALID_PRESSURE           (UINT64_C(1) << 11)
#define HIPNUC_VALID_TEMPERATURE        (UINT64_C(1) << 12)
#define HIPNUC_VALID_INCLINATION        (UINT64_C(1) << 13)
#define HIPNUC_VALID_HEAVE              (UINT64_C(1) << 14) /* displacement only */
#define HIPNUC_VALID_POSITION           (UINT64_C(1) << 15) /* INS longitude and latitude */
#define HIPNUC_VALID_VELOCITY_ENU       (UINT64_C(1) << 16)
#define HIPNUC_VALID_ACC_ENU            (UINT64_C(1) << 17)
#define HIPNUC_VALID_POSITION_QUALITY  (UINT64_C(1) << 18)
#define HIPNUC_VALID_PDOP               (UINT64_C(1) << 19)
#define HIPNUC_VALID_DIFF_AGE           (UINT64_C(1) << 20)
#define HIPNUC_VALID_UNDULATION         (UINT64_C(1) << 21)
#define HIPNUC_VALID_ODOMETER           (UINT64_C(1) << 22)
#define HIPNUC_VALID_GNSS_POSITION      (UINT64_C(1) << 23) /* GNSS longitude and latitude */
#define HIPNUC_VALID_GNSS_VELOCITY      (UINT64_C(1) << 24)
#define HIPNUC_VALID_SOG                (UINT64_C(1) << 25)
#define HIPNUC_VALID_NODE_ID            (UINT64_C(1) << 26)
#define HIPNUC_VALID_YAW                (UINT64_C(1) << 27)
#define HIPNUC_VALID_ALTITUDE           (UINT64_C(1) << 28) /* INS MSL altitude */
#define HIPNUC_VALID_HEADING_QUALITY   (UINT64_C(1) << 29)
#define HIPNUC_VALID_POSITION_SATELLITES (UINT64_C(1) << 30)
#define HIPNUC_VALID_HEADING_SATELLITES (UINT64_C(1) << 31)
#define HIPNUC_VALID_HDOP               (UINT64_C(1) << 32)
#define HIPNUC_VALID_HEAVE_FREQUENCY    (UINT64_C(1) << 33)
#define HIPNUC_VALID_GNSS_ALTITUDE      (UINT64_C(1) << 34)
#define HIPNUC_VALID_COG                (UINT64_C(1) << 35)
#define HIPNUC_VALID_NMEA_STATUS        (UINT64_C(1) << 36)
#define HIPNUC_VALID_NMEA_MODE          (UINT64_C(1) << 37)
#define HIPNUC_VALID_UTC_TIME_OF_DAY    (UINT64_C(1) << 38) /* date may be absent */
#define HIPNUC_VALID_INCLINATION_YAW   (UINT64_C(1) << 39)

typedef struct {
    uint16_t year;        /* four digits */
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t millisecond;
} hipnuc_utc_t;

typedef struct {
    hipnuc_source_t source;
    uint64_t valid;                    /* HIPNUC_VALID_* field availability bits */
    uint8_t  node_id;                  /* CAN source address or HI83 node id */

    /* Status */
    uint16_t main_status;              /* raw HIPNUC_STATUS_* bits */
    uint8_t  gyro_bias_converged;      /* 1 when the WB_CONV warning is clear */
    uint8_t  attitude_converged;       /* 1 when the ATT_CONV warning is clear */
    uint8_t  magnetic_disturbance;     /* 1 while MAG_DIST is set */
    uint8_t  device_static;
    uint8_t  magnetometer_aiding;
    uint8_t  ins_status;               /* HIPNUC_INS_* */

    /* Inertial, body axes */
    float acc[3];                      /* m/s^2, specific force */
    float gyr[3];                      /* rad/s */
    float mag[3];                      /* T */

    /* Attitude */
    float roll;                        /* rad */
    float pitch;                       /* rad */
    float yaw;                         /* rad, device Euler convention */
    float heading;                     /* rad, 0..2pi clockwise from north (INS heading) */
    float quat[4];                     /* w, x, y, z; body to navigation */
    float inclination[2];              /* rad, two independent tilt angles */
    float inclination_yaw;             /* rad, yaw carried by the HI83 inclination field */

    /* Time */
    uint64_t device_time_us;           /* device counter; HI91 ms x 1000 */
    hipnuc_utc_t utc;
    uint16_t gps_week;
    uint32_t gps_tow_ms;

    /* Environment */
    float pressure;                    /* Pa */
    float temperature;                 /* degC */

    /* MRU */
    float heave_m[3];                  /* heave, surge, sway; m */
    float heave_hz[3];                 /* Hz */

    /* Navigation (INS solution) */
    double longitude;                  /* deg */
    double latitude;                   /* deg */
    double altitude_msl;               /* m */
    float  vel_enu[3];                 /* m/s */
    float  acc_enu[3];                 /* m/s^2 */
    float  sog;                        /* m/s, speed over ground */
    float  cog;                        /* rad, course over ground, clockwise from north */
    float  odometer_speed;             /* m/s */

    /* GNSS raw solution and quality */
    double gnss_longitude;             /* deg */
    double gnss_latitude;              /* deg */
    double gnss_altitude_msl;          /* m */
    float  gnss_vel_enu[3];            /* m/s */
    uint8_t position_quality;          /* GGA quality: 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float */
    uint8_t position_satellites;
    uint8_t heading_quality;           /* 4 = fixed, only then is a dual-antenna heading valid */
    uint8_t heading_satellites;
    char nmea_status;                  /* RMC A/V, independently of mode */
    char nmea_mode;                    /* RMC mode, including E (estimated) */
    float  pdop;
    float  hdop;
    float  diff_age;                   /* s */
    float  undulation;                 /* m, geoid separation; ellipsoid height = msl + undulation */
} hipnuc_sample_t;

/* Reset a sample: source none, no valid bits. */
void hipnuc_sample_clear(hipnuc_sample_t *s);

/* Fill main_status and the derived *_converged / disturbance flags. */
void hipnuc_sample_set_status(hipnuc_sample_t *s, uint16_t main_status);

/* Calendar/range validation only; the caller establishes synchronization. */
int hipnuc_utc_is_valid(const hipnuc_utc_t *utc);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_SAMPLE_H */
