/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * hipnuc_data_t: one decoded message from a HiPNUC device, in SI units, with
 * a validity bit per field. Every decoder of this SDK fills it: serial frames
 * (HI91, HI81, HI83), NMEA sentences, CAN messages (J1939, CANFD83) and I2C.
 *
 * A field is meaningful only when its HIPNUC_VALID_* bit is set in `valid`.
 * The bits mean that the message carried the field, not that a solution has
 * converged or that a navigation fix is good: check the status and quality
 * fields for that. One message can carry only some fields (a J1939 message
 * carries one quantity); hipnuc_data_merge() collects them into one record.
 *
 * Conventions: acc is specific force (gravity is not removed). Euler
 * angles and the quaternion follow the device's output coordinate setting
 * (COORD 0: right-front-up, 312 Euler order; COORD 4: ROS/FLU, 321). `yaw` is
 * the device Euler yaw (counter-clockwise); `heading` is clockwise from north.
 * Positions are WGS84 with altitude above mean sea level; velocities are
 * east, north, up.
 */

#ifndef HIPNUC_DATA_H
#define HIPNUC_DATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Unit conversions used by the device wire formats. */
#define HIPNUC_GRAVITY  9.8f                    /* 1 G on the wire = 9.8 m/s^2 */
#define HIPNUC_DEG2RAD  0.017453292519943295f
#define HIPNUC_RAD2DEG  57.29577951308232f
#define HIPNUC_KNOT2MPS 0.514444444f

/* main_status bits (HI91, HI83, CANFD83). A set convergence bit is a warning. */
#define HIPNUC_STATUS_COORD_MASK (7U << 0)   /* output coordinate setting: 0 or 4 */
#define HIPNUC_STATUS_WB_CONV    (1U << 3)   /* gyroscope bias not yet converged */
#define HIPNUC_STATUS_MAG_DIST   (1U << 4)   /* magnetic disturbance */
#define HIPNUC_STATUS_ACC_SAT    (1U << 5)   /* accelerometer saturated */
#define HIPNUC_STATUS_GYR_SAT    (1U << 6)   /* gyroscope saturated */
#define HIPNUC_STATUS_ATT_CONV   (1U << 7)   /* attitude not yet converged */
#define HIPNUC_STATUS_STATIC     (1U << 9)   /* device at rest */
#define HIPNUC_STATUS_MAG_AIDING (1U << 10)  /* magnetometer aids heading */
#define HIPNUC_STATUS_UTC_UNSYNC (1U << 11)  /* time not synchronized to UTC */
#define HIPNUC_STATUS_SOUT_PULSE (1U << 12)

/* ins_status values (HI81, J1939, and HI83 frames that carry INS fields) */
#define HIPNUC_INS_INVALID        0
#define HIPNUC_INS_ALIGNING       1
#define HIPNUC_INS_NAVIGATING     3
#define HIPNUC_INS_DEAD_RECKONING 6

/* Where the message came from */
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

/* Validity bits for hipnuc_data_t.valid */
#define HIPNUC_VALID_STATUS              (UINT64_C(1) << 0)   /* main_status and the flags derived from it */
#define HIPNUC_VALID_INS_STATUS          (UINT64_C(1) << 1)
#define HIPNUC_VALID_NODE_ID             (UINT64_C(1) << 2)
#define HIPNUC_VALID_SEQUENCE            (UINT64_C(1) << 3)
#define HIPNUC_VALID_ACC                 (UINT64_C(1) << 4)
#define HIPNUC_VALID_GYR                 (UINT64_C(1) << 5)
#define HIPNUC_VALID_MAG                 (UINT64_C(1) << 6)
#define HIPNUC_VALID_ROLL_PITCH          (UINT64_C(1) << 7)
#define HIPNUC_VALID_YAW                 (UINT64_C(1) << 8)
#define HIPNUC_VALID_HEADING             (UINT64_C(1) << 9)
#define HIPNUC_VALID_QUAT                (UINT64_C(1) << 10)
#define HIPNUC_VALID_INCLINATION         (UINT64_C(1) << 11)
#define HIPNUC_VALID_INCLINATION_YAW     (UINT64_C(1) << 12)
#define HIPNUC_VALID_ATT_STD             (UINT64_C(1) << 13)
#define HIPNUC_VALID_SYSTEM_TIME         (UINT64_C(1) << 14)
#define HIPNUC_VALID_DEVICE_TIME         (UINT64_C(1) << 15)
#define HIPNUC_VALID_UTC                 (UINT64_C(1) << 16)  /* date and time of day */
#define HIPNUC_VALID_UTC_TIME_OF_DAY     (UINT64_C(1) << 17)  /* time of day; the date may be absent */
#define HIPNUC_VALID_GPS_TIME            (UINT64_C(1) << 18)
#define HIPNUC_VALID_PRESSURE            (UINT64_C(1) << 19)
#define HIPNUC_VALID_TEMPERATURE         (UINT64_C(1) << 20)
#define HIPNUC_VALID_HEAVE               (UINT64_C(1) << 21)
#define HIPNUC_VALID_HEAVE_FREQUENCY     (UINT64_C(1) << 22)
#define HIPNUC_VALID_POSITION            (UINT64_C(1) << 23)  /* INS longitude and latitude */
#define HIPNUC_VALID_ALTITUDE            (UINT64_C(1) << 24)  /* INS altitude */
#define HIPNUC_VALID_VELOCITY_ENU        (UINT64_C(1) << 25)
#define HIPNUC_VALID_ACC_ENU             (UINT64_C(1) << 26)
#define HIPNUC_VALID_SOG                 (UINT64_C(1) << 27)
#define HIPNUC_VALID_COG                 (UINT64_C(1) << 28)
#define HIPNUC_VALID_GNSS_POSITION       (UINT64_C(1) << 29)  /* GNSS longitude and latitude */
#define HIPNUC_VALID_GNSS_ALTITUDE       (UINT64_C(1) << 30)
#define HIPNUC_VALID_GNSS_VELOCITY       (UINT64_C(1) << 31)
#define HIPNUC_VALID_POSITION_QUALITY    (UINT64_C(1) << 32)
#define HIPNUC_VALID_POSITION_SATELLITES (UINT64_C(1) << 33)
#define HIPNUC_VALID_HEADING_QUALITY     (UINT64_C(1) << 34)
#define HIPNUC_VALID_HEADING_SATELLITES  (UINT64_C(1) << 35)
#define HIPNUC_VALID_NMEA_STATUS         (UINT64_C(1) << 36)
#define HIPNUC_VALID_NMEA_MODE           (UINT64_C(1) << 37)
#define HIPNUC_VALID_PDOP                (UINT64_C(1) << 38)
#define HIPNUC_VALID_HDOP                (UINT64_C(1) << 39)
#define HIPNUC_VALID_DIFF_AGE            (UINT64_C(1) << 40)
#define HIPNUC_VALID_UNDULATION          (UINT64_C(1) << 41)
#define HIPNUC_VALID_ALL                 ((UINT64_C(1) << 42) - 1U)

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
    uint64_t valid;                   /* HIPNUC_VALID_* */
    uint8_t  node_id;                 /* CAN source address, or the HI83 node id */
    uint8_t  sequence;                /* CANFD83 frame counter, wraps at 255 */

    /* Status */
    uint16_t main_status;             /* HIPNUC_STATUS_* bits */
    uint8_t  gyro_bias_converged;     /* 1 when the WB_CONV warning is clear */
    uint8_t  attitude_converged;      /* 1 when the ATT_CONV warning is clear */
    uint8_t  magnetic_disturbance;    /* 1 while MAG_DIST is set */
    uint8_t  device_static;
    uint8_t  magnetometer_aiding;
    uint8_t  ins_status;              /* HIPNUC_INS_* */

    /* Inertial, body axes */
    float acc[3];                     /* m/s^2, specific force */
    float gyr[3];                     /* rad/s */
    float mag[3];                     /* T */

    /* Attitude */
    float roll;                       /* rad */
    float pitch;                      /* rad */
    float yaw;                        /* rad, device Euler convention, counter-clockwise */
    float heading;                    /* rad, 0..2pi clockwise from north */
    float quat[4];                    /* w, x, y, z; body to navigation */
    float inclination[2];             /* rad, X tilt (roll) and Y tilt (pitch) of the inclination output */
    float inclination_yaw;            /* rad, the yaw carried by the HI83 inclination field */
    float att_std[3];                 /* rad, >= 0, 1-sigma about the navigation axes (E, N, U for COORD 0;
                                         N, W, U for 6-axis COORD 4); [2] is the heading uncertainty */

    /* Time */
    uint32_t system_time_ms;          /* HI91: UTC ms of day once synchronized, else uptime ms; wraps at 24 h */
    uint64_t device_time_us;          /* HI83, CANFD83: device time since power-up, us */
    hipnuc_utc_t utc;
    uint16_t gps_week;
    uint32_t gps_tow_ms;

    /* Environment */
    float pressure;                   /* Pa */
    float temperature;                /* degC */

    /* Motion reference (MRU models) */
    float heave_m[3];                 /* heave, surge, sway; m */
    float heave_hz[3];                /* Hz */

    /* Navigation solution (INS) */
    double longitude;                 /* deg */
    double latitude;                  /* deg */
    double altitude_msl;              /* m */
    float  vel_enu[3];                /* m/s */
    float  acc_enu[3];                /* m/s^2, gravity removed (about 0 at rest) */
    float  sog;                       /* m/s, speed over ground */
    float  cog;                       /* rad, course over ground, clockwise from north */

    /* GNSS solution and quality */
    double  gnss_longitude;           /* deg */
    double  gnss_latitude;            /* deg */
    double  gnss_altitude_msl;        /* m */
    float   gnss_vel_enu[3];          /* m/s */
    uint8_t position_quality;         /* GGA quality: 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float, 6 dead reckoning */
    uint8_t position_satellites;
    uint8_t heading_quality;          /* 4 = fixed: only then is a dual-antenna heading valid */
    uint8_t heading_satellites;
    char    nmea_status;              /* RMC A/V */
    char    nmea_mode;                /* RMC mode, including E (estimated) */
    float   pdop;
    float   hdop;
    float   diff_age;                 /* s */
    float   undulation;               /* m, geoid separation; ellipsoid height = msl + undulation */
} hipnuc_data_t;

/* Copy every field whose bit is set in src->valid into dst and add those bits
 * to dst->valid; dst->source becomes src->source. Collects the messages of
 * one epoch (J1939 messages, a binary frame plus NMEA) into one record; start
 * each epoch from a cleared record: a time of day without a date (GGA) keeps
 * the date already in dst. */
void hipnuc_data_merge(hipnuc_data_t *dst, const hipnuc_data_t *src);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_DATA_H */
