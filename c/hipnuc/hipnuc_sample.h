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
 * when the matching HIPNUC_VALID_* bit is set in `valid`; a converter
 * clears the structure first, so stale values never survive.
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

#include "hipnuc_dec.h"
#include "nmea_dec.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Product wire conventions shared by every converter. */
#define HIPNUC_GRAVITY          9.8f                    /* 1 G on the wire = 9.8 m/s^2 */
#define HIPNUC_DEG2RAD          0.017453292519943295f
#define HIPNUC_KNOT2MPS         0.514444444f

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
#define HIPNUC_VALID_STATUS          (UINT32_C(1) << 0)   /* main_status and the *_converged flags */
#define HIPNUC_VALID_INS_STATUS      (UINT32_C(1) << 1)
#define HIPNUC_VALID_ACC             (UINT32_C(1) << 2)
#define HIPNUC_VALID_GYR             (UINT32_C(1) << 3)
#define HIPNUC_VALID_MAG             (UINT32_C(1) << 4)
#define HIPNUC_VALID_EULER           (UINT32_C(1) << 5)   /* roll, pitch, yaw */
#define HIPNUC_VALID_HEADING         (UINT32_C(1) << 6)   /* heading_rad */
#define HIPNUC_VALID_QUAT            (UINT32_C(1) << 7)
#define HIPNUC_VALID_DEVICE_TIME     (UINT32_C(1) << 8)
#define HIPNUC_VALID_UTC             (UINT32_C(1) << 9)   /* utc fields are a synchronized UTC time */
#define HIPNUC_VALID_GPS_TIME        (UINT32_C(1) << 10)  /* gps_week, gps_tow_ms */
#define HIPNUC_VALID_PRESSURE        (UINT32_C(1) << 11)
#define HIPNUC_VALID_TEMPERATURE     (UINT32_C(1) << 12)
#define HIPNUC_VALID_INCLINATION     (UINT32_C(1) << 13)
#define HIPNUC_VALID_HEAVE           (UINT32_C(1) << 14)  /* heave_m and heave_hz */
#define HIPNUC_VALID_POSITION        (UINT32_C(1) << 15)  /* longitude, latitude, altitude_msl */
#define HIPNUC_VALID_VELOCITY_ENU    (UINT32_C(1) << 16)
#define HIPNUC_VALID_ACC_ENU         (UINT32_C(1) << 17)
#define HIPNUC_VALID_GNSS_QUALITY    (UINT32_C(1) << 18)  /* position/heading quality and satellites */
#define HIPNUC_VALID_DOP             (UINT32_C(1) << 19)
#define HIPNUC_VALID_DIFF_AGE        (UINT32_C(1) << 20)
#define HIPNUC_VALID_UNDULATION      (UINT32_C(1) << 21)
#define HIPNUC_VALID_ODOMETER        (UINT32_C(1) << 22)
#define HIPNUC_VALID_GNSS_POSITION   (UINT32_C(1) << 23)  /* raw GNSS, separate from the INS solution */
#define HIPNUC_VALID_GNSS_VELOCITY   (UINT32_C(1) << 24)
#define HIPNUC_VALID_SOG_COG         (UINT32_C(1) << 25)  /* speed and course over ground */
#define HIPNUC_VALID_NODE_ID         (UINT32_C(1) << 26)

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
    uint32_t valid;                    /* HIPNUC_VALID_* bits */
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
    float  pdop;
    float  hdop;
    float  diff_age;                   /* s */
    float  undulation;                 /* m, geoid separation; ellipsoid height = msl + undulation */
} hipnuc_sample_t;

/* Reset a sample: source none, no valid bits. */
void hipnuc_sample_clear(hipnuc_sample_t *s);

/* Serial binary packets (hipnuc_raw_t.hi91 / hi81 / hi83). */
void hipnuc_sample_from_hi91(const hi91_t *p, hipnuc_sample_t *s);
void hipnuc_sample_from_hi81(const hi81_t *p, hipnuc_sample_t *s);
void hipnuc_sample_from_hi83(const hi83_t *p, hipnuc_sample_t *s);

/**
 * Convert whatever hipnuc_input() just decoded. Returns 1 when a packet was
 * converted, 0 when no tag was set. With several sub-packets in one frame
 * the HI83 packet wins, then HI81, then HI91.
 */
int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *s);

/* NMEA sentences (nmea_raw_t.gga / rmc). */
void hipnuc_sample_from_gga(const nmea_gga_t *g, hipnuc_sample_t *s);
void hipnuc_sample_from_rmc(const nmea_rmc_t *r, hipnuc_sample_t *s);
int hipnuc_sample_from_nmea(const nmea_raw_t *raw, hipnuc_sample_t *s);

/* Fill main_status and the derived *_converged / disturbance flags. */
void hipnuc_sample_set_status(hipnuc_sample_t *s, uint16_t main_status);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_SAMPLE_H */
