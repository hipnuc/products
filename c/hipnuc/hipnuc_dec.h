/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC serial binary protocol decoder (0x5A 0xA5 frames).
 *
 * Portable C99, no dynamic memory, no stdio, no global state. Copy
 * hipnuc_dec.c/.h and hipnuc_sample.c/.h into your project, keep one
 * hipnuc_raw_t per serial port and feed received bytes with hipnuc_input().
 *
 * Structure fields keep the units used on the wire (see comments). Use
 * hipnuc_sample.h to obtain SI units with validity flags.
 *
 * Supported firmware: current platform, 1.7.0 or later. The HI83 timestamp is
 * the 8-byte microsecond layout; early 1.7.1 builds with a 4-byte layout must
 * be upgraded.
 */

#ifndef HIPNUC_DEC_H
#define HIPNUC_DEC_H

#include <stddef.h>
#include <stdint.h>
#include "hipnuc_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Packed wire structures for GCC, Clang, ARM Compiler and MSVC. */
#if defined(_MSC_VER)
#define HIPNUC_PACKED_BEGIN __pragma(pack(push, 1))
#define HIPNUC_PACKED_END   __pragma(pack(pop))
#define HIPNUC_PACKED
#elif defined(__MINGW32__)
#define HIPNUC_PACKED_BEGIN _Pragma("pack(push, 1)")
#define HIPNUC_PACKED_END   _Pragma("pack(pop)")
#define HIPNUC_PACKED       __attribute__((packed))
#else
#define HIPNUC_PACKED_BEGIN
#define HIPNUC_PACKED_END
#define HIPNUC_PACKED       __attribute__((packed))
#endif

/* Frame constants */
#define HIPNUC_SYNC1            0x5AU
#define HIPNUC_SYNC2            0xA5U
#define HIPNUC_HEADER_SIZE      6           /* sync(2) + length(2) + crc(2) */
#define HIPNUC_MAX_PAYLOAD_SIZE 506
#define HIPNUC_MAX_RAW_SIZE     512         /* header + maximum payload */

/* Sub-packet tags */
#define HIPNUC_ID_HI91          0x91U
#define HIPNUC_ID_HI81          0x81U
#define HIPNUC_ID_HI83          0x83U

HIPNUC_PACKED_BEGIN

/**
 * Packet 0x91: IMU/AHRS data, float. 76 bytes.
 */
typedef struct HIPNUC_PACKED {
    uint8_t  tag;            /* 0x91 */
    uint16_t main_status;    /* HIPNUC_STATUS_* bits */
    int8_t   temp;           /* degC */
    float    air_pressure;   /* Pa */
    uint32_t system_time;    /* ms; UTC time of day when synchronized, uptime otherwise */
    float    acc[3];         /* G (firmware divides m/s^2 by 9.8) */
    float    gyr[3];         /* deg/s */
    float    mag[3];         /* uT */
    float    roll;           /* deg */
    float    pitch;          /* deg */
    float    yaw;            /* deg, device Euler convention (ENU/312 by default) */
    float    quat[4];        /* w, x, y, z; body to navigation */
} hi91_t;

/**
 * Packet 0x81: INS data, fixed point. 104 bytes.
 */
typedef struct HIPNUC_PACKED {
    uint8_t  tag;            /* 0x81 */
    uint16_t main_status;    /* HIPNUC_STATUS_* bits */
    uint8_t  ins_status;     /* HIPNUC_INS_* */
    uint16_t gpst_wn;        /* GPS week */
    uint32_t gpst_tow;       /* GPS time of week, ms */
    uint16_t reserved0;
    int16_t  gyr_b[3];       /* x 0.001 rad/s */
    int16_t  acc_b[3];       /* x 0.0048828 m/s^2 */
    int16_t  mag_b[3];       /* x 0.030517 uT */
    int16_t  air_pressure;   /* Pa - 100000 */
    int16_t  od_speed;       /* x 0.01 m/s (odometer; 0 when unused) */
    int8_t   temperature;    /* degC */
    uint8_t  utc_year;       /* year - 2000 */
    uint8_t  utc_month;
    uint8_t  utc_day;
    uint8_t  utc_hour;
    uint8_t  utc_min;
    uint16_t utc_msec;       /* second * 1000 + millisecond */
    int16_t  roll;           /* x 0.01 deg */
    int16_t  pitch;          /* x 0.01 deg */
    uint16_t yaw;            /* x 0.01 deg, heading 0..360 clockwise from north */
    int16_t  quat[4];        /* x 0.0001; w, x, y, z */
    int32_t  ins_lon;        /* x 1e-7 deg */
    int32_t  ins_lat;        /* x 1e-7 deg */
    int32_t  ins_msl;        /* x 0.001 m, above mean sea level */
    uint8_t  pdop;           /* x 0.1 */
    uint8_t  hdop;           /* x 0.1 */
    uint8_t  solq_pos;       /* GNSS position quality (GGA style) */
    uint8_t  nv_pos;         /* satellites used for position */
    uint8_t  solq_heading;   /* GNSS heading quality (4 = fixed) */
    uint8_t  nv_heading;     /* satellites used for heading */
    uint8_t  diff_age;       /* s */
    int16_t  undulation;     /* x 0.01 m, geoid separation */
    uint8_t  ant_status;
    int16_t  vel_enu[3];     /* x 0.01 m/s */
    int16_t  acc_enu[3];     /* x 0.0048828 m/s^2 */
    uint8_t  reserved_tail[14]; /* always zero on current firmware */
} hi81_t;

/* HI83 data_bitmap bits decoded by this SDK. Bits 25..29 are internal and
 * are not decoded; a frame containing them is rejected. On the wire bits
 * 30 and 31 follow bit 27 and precede bits 28 and 29. */
#define HI83_BMAP_ACC_B              (UINT32_C(1) << 0)   /* acc_b, m/s^2 */
#define HI83_BMAP_GYR_B              (UINT32_C(1) << 1)   /* gyr_b, rad/s */
#define HI83_BMAP_MAG_B              (UINT32_C(1) << 2)   /* mag_b, uT */
#define HI83_BMAP_RPY                (UINT32_C(1) << 3)   /* rpy, deg */
#define HI83_BMAP_QUAT               (UINT32_C(1) << 4)
#define HI83_BMAP_SYSTEM_TIME        (UINT32_C(1) << 5)   /* system_time_us */
#define HI83_BMAP_UTC                (UINT32_C(1) << 6)
#define HI83_BMAP_AIR_PRESSURE       (UINT32_C(1) << 7)   /* Pa */
#define HI83_BMAP_TEMPERATURE        (UINT32_C(1) << 8)   /* degC */
#define HI83_BMAP_INCLINATION        (UINT32_C(1) << 9)   /* deg */
#define HI83_BMAP_HSS                (UINT32_C(1) << 10)  /* heave/surge/sway, m */
#define HI83_BMAP_HSS_FRQ            (UINT32_C(1) << 11)  /* Hz */
#define HI83_BMAP_VEL_ENU            (UINT32_C(1) << 12)  /* m/s */
#define HI83_BMAP_ACC_ENU            (UINT32_C(1) << 13)  /* m/s^2 */
#define HI83_BMAP_INS_LON_LAT_MSL    (UINT32_C(1) << 14)  /* deg, deg, m */
#define HI83_BMAP_GNSS_QUALITY_NV    (UINT32_C(1) << 15)
#define HI83_BMAP_OD_SPEED           (UINT32_C(1) << 16)  /* m/s */
#define HI83_BMAP_UNDULATION         (UINT32_C(1) << 17)  /* m */
#define HI83_BMAP_DIFF_AGE           (UINT32_C(1) << 18)  /* s */
#define HI83_BMAP_NODE_ID            (UINT32_C(1) << 19)
#define HI83_BMAP_GNSS_LON_LAT_MSL   (UINT32_C(1) << 30)  /* deg, deg, m */
#define HI83_BMAP_GNSS_VEL           (UINT32_C(1) << 31)  /* m/s, ENU */
#define HI83_BMAP_SUPPORTED          UINT32_C(0xC00FFFFF)

/**
 * Packet 0x83: INS data, float, bitmap selected fields. Variable length.
 * Only the fields whose bit is set in data_bitmap are valid; the others are
 * zero.
 */
typedef struct HIPNUC_PACKED {
    uint8_t  tag;            /* 0x83 */
    uint16_t main_status;    /* HIPNUC_STATUS_* bits */
    uint8_t  ins_status;     /* HIPNUC_INS_* */
    uint32_t data_bitmap;    /* HI83_BMAP_* */

    float    acc_b[3];       /* m/s^2 */
    float    gyr_b[3];       /* rad/s */
    float    mag_b[3];       /* uT */
    float    rpy[3];         /* roll, pitch, yaw, deg */
    float    quat[4];        /* w, x, y, z */
    uint64_t system_time_us; /* us, free-running device counter */
    struct HIPNUC_PACKED {
        uint8_t  year;       /* year - 2000 */
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint16_t sec_ms;     /* second * 1000 + millisecond */
        uint8_t  rev;
    } utc;
    float    air_pressure;   /* Pa */
    float    temperature;    /* degC */
    float    inclination[3]; /* incli_x, incli_y, yaw; deg */
    float    hss[3];         /* heave, surge, sway; m */
    float    hss_frq[3];     /* Hz */
    float    vel_enu[3];     /* m/s */
    float    acc_enu[3];     /* m/s^2 */
    double   ins_lon_lat_msl[3]; /* deg, deg, m above mean sea level */
    uint8_t  solq_pos;
    uint8_t  nv_pos;
    uint8_t  solq_heading;
    uint8_t  nv_heading;
    float    od_speed;       /* m/s */
    float    undulation;     /* m */
    float    diff_age;       /* s */
    struct HIPNUC_PACKED {
        uint8_t node_id;
        uint8_t reserved[3];
    } node;
    double   gnss_lon_lat_msl[3]; /* deg, deg, m */
    float    gnss_vel[3];    /* m/s, ENU */
} hi83_t;

HIPNUC_PACKED_END

/**
 * Decoder state. Zero-initialize before first use; one per serial port.
 * After hipnuc_input() returns 1, exactly one sub-packet with a nonzero tag holds
 * the newly decoded data. All tags are cleared before decoding a frame, so a
 * stale packet is never presented as new.
 */
typedef struct {
    int      nbyte;                     /* bytes currently in buf */
    int      len;                       /* announced payload length */
    uint8_t  buf[HIPNUC_MAX_RAW_SIZE];  /* frame buffer */
    hi91_t   hi91;                      /* valid when hi91.tag == HIPNUC_ID_HI91 */
    hi81_t   hi81;                      /* valid when hi81.tag == HIPNUC_ID_HI81 */
    hi83_t   hi83;                      /* valid when hi83.tag == HIPNUC_ID_HI83 */
    uint32_t frame_count;               /* frames decoded successfully */
    uint32_t crc_error_count;
    uint32_t invalid_count;             /* bad length, unknown tag, truncated */
} hipnuc_raw_t;

/**
 * Feed one received byte.
 *
 * @return 1 when a frame was decoded (inspect raw->hi91/hi81/hi83 tags),
 *         0 when more bytes are needed, -1 when the frame was invalid
 *         (CRC, length, multiple sub-packets, unknown tag or unsupported
 *         HI83 bits). Test `> 0`.
 * On CRC/length failure, an unfinished next-frame prefix is retained.
 * Complete candidates already swallowed by a damaged frame are discarded;
 * this byte API does not queue samples. CRC-valid unsupported envelopes are
 * always discarded whole. Continue feeding bytes after a negative result.
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data);

/**
 * Feed a block of received bytes. Stops at the first complete or invalid
 * frame; continue with data + consumed even after a negative result.
 *
 * @param consumed receives the number of bytes taken from data
 * @return same as hipnuc_input(); 0 when the block ended without a frame
 */
int hipnuc_input_buffer(hipnuc_raw_t *raw, const uint8_t *data, size_t len, size_t *consumed);

/**
 * CRC-16/XMODEM (poly 0x1021, init 0) as used by the frame header.
 */
uint16_t hipnuc_crc16(uint16_t crc, const uint8_t *data, size_t len);

/* Convert the one packet returned by hipnuc_input(); clears the destination.
 * Wire values remain available in the packet structures above. */
void hipnuc_sample_from_hi91(const hi91_t *p, hipnuc_sample_t *s);
void hipnuc_sample_from_hi81(const hi81_t *p, hipnuc_sample_t *s);
void hipnuc_sample_from_hi83(const hi83_t *p, hipnuc_sample_t *s);
/* Returns 1 for exactly one tagged packet, 0 otherwise. */
int hipnuc_sample_from_raw(const hipnuc_raw_t *raw, hipnuc_sample_t *s);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_DEC_H */
