/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Regression tests for hipnuc_j1939.c: hand-built frames with hand-computed
 * expected values. Plain C99, no framework; the process exit code is the
 * number of failed checks (capped by the shell).
 */

#include <stdio.h>
#include <string.h>

#include "hipnuc_j1939.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { \
        ++g_failures; \
        printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

static int near(double a, double b, double tol)
{
    double d = a - b;
    if (d < 0) d = -d;
    return d <= tol;
}

/* Little-endian writers into a frame payload */
static void put_u16(uint8_t *d, int off, uint16_t v)
{
    d[off] = (uint8_t)(v & 0xFF);
    d[off + 1] = (uint8_t)((v >> 8) & 0xFF);
}

static void put_i16(uint8_t *d, int off, int16_t v)
{
    put_u16(d, off, (uint16_t)v);
}

static void put_u32(uint8_t *d, int off, uint32_t v)
{
    d[off] = (uint8_t)(v & 0xFF);
    d[off + 1] = (uint8_t)((v >> 8) & 0xFF);
    d[off + 2] = (uint8_t)((v >> 16) & 0xFF);
    d[off + 3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_i32(uint8_t *d, int off, int32_t v)
{
    put_u32(d, off, (uint32_t)v);
}

static void put_u64(uint8_t *d, int off, uint64_t v)
{
    put_u32(d, off, (uint32_t)(v & 0xFFFFFFFFU));
    put_u32(d, off + 4, (uint32_t)(v >> 32));
}

static void put_f32(uint8_t *d, int off, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    put_u32(d, off, u);
}

/* Extended data frame from `source` with an all-zero payload of `len` bytes */
static void make_frame(hipnuc_can_frame_t *f, uint32_t pgn, uint8_t source, uint8_t len)
{
    memset(f, 0, sizeof(*f));
    f->id = hipnuc_j1939_data_id(pgn, source);
    f->is_extended = 1;
    f->len = len;
}

#define RAD(deg) ((deg) * 0.017453292519943295)
#define FTOL 1e-5
#define ATOL 1e-6   /* angles in rad */

/* ------------------------------------------------------------------------- */

static void test_id_helpers(void)
{
    /* priority 3 << 26 = 0x0C000000; PGN 0xFF34 << 8 = 0x00FF3400; SA 0x08 */
    CHECK(hipnuc_j1939_data_id(HIPNUC_J1939_PGN_ACC, 0x08) == 0x0CFF3408U);
    CHECK(hipnuc_j1939_pgn(0x0CFF3408U) == 0xFF34U);
    CHECK(hipnuc_j1939_source_address(0x0CFF3408U) == 0x08);
    CHECK(hipnuc_j1939_source_address(0x0CFF3488U) == 0x88);
    /* the priority bits are not part of the PGN */
    CHECK(hipnuc_j1939_pgn(0x18FF3408U) == 0xFF34U);
}

static void test_acc(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    put_i16(f.data, 0, 2048);    /* 2048 / 2048 G = 1 G   -> 9.8 m/s^2 */
    put_i16(f.data, 2, -2048);   /* -1 G                  -> -9.8 */
    put_i16(f.data, 4, 1024);    /* 0.5 G                 -> 4.9 */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_ACC);
    CHECK(s.source == HIPNUC_SOURCE_J1939);
    CHECK(s.node_id == 0x08);
    CHECK(s.valid & HIPNUC_VALID_NODE_ID);
    CHECK((s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG)) == HIPNUC_VALID_ACC);
    CHECK(near(s.acc[0], 9.8, FTOL));
    CHECK(near(s.acc[1], -9.8, FTOL));
    CHECK(near(s.acc[2], 4.9, FTOL));

    /* firmware sends 8 bytes; 6 is the minimum, 5 is too short */
    f.len = 6;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_ACC);
    f.len = 5;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_gyr(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_GYR, 0x08, 8);
    put_i16(f.data, 0, 16384);   /* 16384 * 2000/32768 = 1000 deg/s  -> 17.4532925 rad/s */
    put_i16(f.data, 2, -16384);  /* -1000 deg/s                      -> -17.4532925 */
    put_i16(f.data, 4, 328);     /* 328 * 2000/32768 = 20.01953125 deg/s -> 0.34940673 rad/s */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_GYR);
    CHECK(s.valid & HIPNUC_VALID_GYR);
    CHECK(near(s.gyr[0], RAD(1000.0), 1e-4));
    CHECK(near(s.gyr[1], -RAD(1000.0), 1e-4));
    CHECK(near(s.gyr[2], 0.34940673, 1e-5));
}

static void test_mag(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_MAG, 0x08, 8);
    put_i16(f.data, 0, 16384);   /* 16384 * 1000/32768 = 500 uT  -> 5.0e-4 T */
    put_i16(f.data, 2, -16384);  /* -500 uT                      -> -5.0e-4 T */
    put_i16(f.data, 4, 8192);    /* 250 uT                       -> 2.5e-4 T */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_MAG);
    CHECK(s.valid & HIPNUC_VALID_MAG);
    CHECK(near(s.mag[0], 5.0e-4, 1e-9));
    CHECK(near(s.mag[1], -5.0e-4, 1e-9));
    CHECK(near(s.mag[2], 2.5e-4, 1e-9));
}

static void test_roll_pitch(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_ROLL_PITCH, 0x08, 8);
    put_i32(f.data, 0, 45000);   /* 45.000 deg  -> 0.78539816 rad */
    put_i32(f.data, 4, -30000);  /* -30.000 deg -> -0.52359878 rad */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_ROLL_PITCH);
    CHECK(s.valid & HIPNUC_VALID_EULER);
    CHECK(!(s.valid & HIPNUC_VALID_HEADING));
    CHECK(near(s.roll, 0.78539816, ATOL));
    CHECK(near(s.pitch, -0.52359878, ATOL));
    CHECK(s.yaw == 0.0f);
    f.len = 7;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_yaw(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_YAW, 0x08, 8);
    put_i32(f.data, 0, 270000);  /* heading 270.000 deg CW -> 4.71238898 rad */
    put_i32(f.data, 4, -90000);  /* yaw -90.000 deg        -> -1.57079633 rad */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_YAW);
    CHECK(s.valid & HIPNUC_VALID_HEADING);
    CHECK(!(s.valid & HIPNUC_VALID_EULER));
    CHECK(near(s.heading, 4.71238898, ATOL));
    CHECK(near(s.yaw, -1.57079633, ATOL));

    /* heading only: 4 bytes are enough, yaw stays 0 */
    f.len = 4;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_YAW);
    CHECK(near(s.heading, 4.71238898, ATOL));
    CHECK(s.yaw == 0.0f);
    f.len = 3;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_temp(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_TEMP, 0x08, 8);
    put_i16(f.data, 0, 2534);    /* 25.34 degC */
    put_i32(f.data, 4, 999);     /* reserved placeholder, must not become a pressure */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_TEMP);
    CHECK(s.valid & HIPNUC_VALID_TEMPERATURE);
    CHECK(!(s.valid & HIPNUC_VALID_PRESSURE));
    CHECK(near(s.temperature, 25.34, FTOL));
    CHECK(s.pressure == 0.0f);

    put_i16(f.data, 0, -1050);   /* -10.50 degC */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_TEMP);
    CHECK(near(s.temperature, -10.5, FTOL));
    f.len = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_quat(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_QUAT, 0x08, 8);
    put_i16(f.data, 0, 10000);   /* w  1.0000 */
    put_i16(f.data, 2, -5000);   /* x -0.5000 */
    put_i16(f.data, 4, 2500);    /* y  0.2500 */
    put_i16(f.data, 6, 0);       /* z  0 */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_QUAT);
    CHECK(s.valid & HIPNUC_VALID_QUAT);
    CHECK(near(s.quat[0], 1.0, FTOL));
    CHECK(near(s.quat[1], -0.5, FTOL));
    CHECK(near(s.quat[2], 0.25, FTOL));
    CHECK(near(s.quat[3], 0.0, FTOL));
    f.len = 6;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_inclination(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_INCLINATION, 0x08, 8);
    put_i32(f.data, 0, 12345);   /* 12.345 deg -> 0.21546090 rad */
    put_i32(f.data, 4, -5000);   /* -5.000 deg -> -0.08726646 rad */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_INCLINATION);
    CHECK(s.valid & HIPNUC_VALID_INCLINATION);
    CHECK(near(s.inclination[0], 0.21546090, ATOL));
    CHECK(near(s.inclination[1], -0.08726646, ATOL));
}

static void test_time(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_TIME, 0x08, 8);
    f.data[0] = 26;              /* 2026 */
    f.data[1] = 9;
    f.data[2] = 7;
    f.data[3] = 12;
    f.data[4] = 34;
    f.data[5] = 56;
    put_u16(f.data, 6, 789);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_TIME);
    CHECK(s.valid & HIPNUC_VALID_UTC);
    CHECK(s.utc.year == 2026);
    CHECK(s.utc.month == 9);
    CHECK(s.utc.day == 7);
    CHECK(s.utc.hour == 12);
    CHECK(s.utc.minute == 34);
    CHECK(s.utc.second == 56);
    CHECK(s.utc.millisecond == 789);

    /* no GNSS time: uptime with a zero date is not UTC */
    memset(f.data, 0, sizeof(f.data));
    f.data[3] = 1;
    f.data[4] = 2;
    f.data[5] = 3;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_TIME);
    CHECK(!(s.valid & HIPNUC_VALID_UTC));
    f.len = 7;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_position(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_POSITION, 0x08, 8);
    put_i32(f.data, 0, 316000000);    /* lat  31.6000000 deg (first) */
    put_i32(f.data, 4, -1213000000);  /* lon -121.3000000 deg */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_POSITION);
    CHECK(s.valid & HIPNUC_VALID_POSITION);
    CHECK(near(s.latitude, 31.6, 1e-9));
    CHECK(near(s.longitude, -121.3, 1e-9));
    CHECK(s.altitude_msl == 0.0);
}

static void test_altitude(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_ALTITUDE, 0x08, 8);
    put_i32(f.data, 0, 123456);  /* 123456 cm -> 1234.56 m */
    put_i16(f.data, 4, -350);    /* -350 cm   -> -3.5 m */
    put_i16(f.data, 6, 150);     /* 150 x0.01 -> 1.5 s */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_ALTITUDE);
    CHECK(s.valid & HIPNUC_VALID_UNDULATION);
    CHECK(s.valid & HIPNUC_VALID_DIFF_AGE);
    CHECK(!(s.valid & HIPNUC_VALID_POSITION));
    CHECK(near(s.altitude_msl, 1234.56, 1e-9));
    CHECK(near(s.undulation, -3.5, FTOL));
    CHECK(near(s.diff_age, 1.5, FTOL));
}

static void test_gnss_status(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_GNSS_STATUS, 0x08, 8);
    f.data[0] = 4;               /* solq: RTK fixed */
    f.data[1] = 5;               /* solq heading: RTK float */
    f.data[2] = 28;              /* nv */
    f.data[3] = 26;              /* nv heading */
    f.data[4] = HIPNUC_INS_NAVIGATING;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_GNSS_STATUS);
    CHECK(s.valid & HIPNUC_VALID_GNSS_QUALITY);
    CHECK(s.valid & HIPNUC_VALID_INS_STATUS);
    CHECK(s.position_quality == 4);
    CHECK(s.heading_quality == 5);
    CHECK(s.position_satellites == 28);
    CHECK(s.heading_satellites == 26);
    CHECK(s.ins_status == HIPNUC_INS_NAVIGATING);
    f.len = 5;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_GNSS_STATUS);
    f.len = 4;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_velocity(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_VELOCITY, 0x08, 8);
    put_i16(f.data, 0, 150);     /* e  1.50 m/s */
    put_i16(f.data, 2, -250);    /* n -2.50 m/s */
    put_i16(f.data, 4, 10);      /* u  0.10 m/s */
    put_i16(f.data, 6, 292);     /* ground speed 2.92 m/s, ignored */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_VELOCITY);
    CHECK(s.valid & HIPNUC_VALID_VELOCITY_ENU);
    CHECK(!(s.valid & HIPNUC_VALID_SOG_COG));
    CHECK(near(s.vel_enu[0], 1.5, FTOL));
    CHECK(near(s.vel_enu[1], -2.5, FTOL));
    CHECK(near(s.vel_enu[2], 0.1, FTOL));
    CHECK(s.sog == 0.0f);
}

static void test_rejections(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;

    /* standard frame: never HiPNUC data, whatever the identifier */
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    f.is_extended = 0;
    f.id = 0x734;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);

    /* remote / error flags on a known PGN */
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    f.is_remote = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    f.is_remote = 0;
    f.is_error = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);

    /* unknown PGN, even with the flags set */
    make_frame(&f, 0xFF35, 0x08, 8);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);
    f.is_remote = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);
    make_frame(&f, 0xFEF1, 0x08, 8);     /* a standard J1939 PGN (CCVS) */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);

    /* config frames are not data */
    hipnuc_j1939_build_reg_read(0x08, 0x55, 0x0002, &f);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);

    CHECK(hipnuc_j1939_parse(NULL, &s, NULL) == -1);
    CHECK(hipnuc_j1939_parse(&f, NULL, NULL) == -1);
}

static void test_source_addresses(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t a, b;
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    put_i16(f.data, 4, 2048);
    CHECK(hipnuc_j1939_parse(&f, &a, NULL) == HIPNUC_J1939_MSG_ACC);
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x88, 8);
    put_i16(f.data, 4, -2048);
    CHECK(hipnuc_j1939_parse(&f, &b, NULL) == HIPNUC_J1939_MSG_ACC);
    CHECK(a.node_id == 0x08);
    CHECK(b.node_id == 0x88);
    CHECK(a.node_id != b.node_id);
    CHECK(near(a.acc[2], 9.8, FTOL));
    CHECK(near(b.acc[2], -9.8, FTOL));
}

static void test_config_frames(void)
{
    hipnuc_can_frame_t f;
    uint8_t source = 0, status = 0xFF;
    uint16_t addr = 0;
    uint32_t value = 0;
    hipnuc_j1939_cmd_t cmd = HIPNUC_J1939_CMD_READ;

    /* write: 0x0CEF0000 | dest 0x08 << 8 | source 0x55 */
    hipnuc_j1939_build_reg_write(0x08, 0x55, 0x0042, 0x12345678U, &f);
    CHECK(f.id == 0x0CEF0855U);
    CHECK(f.is_extended == 1);
    CHECK(f.is_remote == 0);
    CHECK(f.is_error == 0);
    CHECK(f.len == 8);
    CHECK(f.data[0] == 0x42 && f.data[1] == 0x00);
    CHECK(f.data[2] == HIPNUC_J1939_CMD_WRITE);
    CHECK(f.data[3] == 0);
    CHECK(f.data[4] == 0x78 && f.data[5] == 0x56 && f.data[6] == 0x34 && f.data[7] == 0x12);
    CHECK(hipnuc_j1939_is_config(&f) == 1);
    CHECK(hipnuc_j1939_parse_config(&f, &source, &addr, &cmd, &status, &value) == 0);
    CHECK(source == 0x55);
    CHECK(addr == 0x0042);
    CHECK(cmd == HIPNUC_J1939_CMD_WRITE);
    CHECK(status == 0);
    CHECK(value == 0x12345678U);
    CHECK(hipnuc_j1939_parse_config(&f, NULL, NULL, NULL, NULL, NULL) == 0);

    /* read request carries value 1 */
    hipnuc_j1939_build_reg_read(0x08, 0x55, 0x0001, &f);
    CHECK(f.id == 0x0CEF0855U);
    CHECK(hipnuc_j1939_parse_config(&f, &source, &addr, &cmd, &status, &value) == 0);
    CHECK(addr == 0x0001);
    CHECK(cmd == HIPNUC_J1939_CMD_READ);
    CHECK(value == 1);

    /* device reply: device 0x08 -> host 0x55, status 1 */
    memset(&f, 0, sizeof(f));
    f.id = 0x0CEF5508U;
    f.is_extended = 1;
    f.len = 8;
    put_u16(f.data, 0, 0x0001);
    f.data[2] = HIPNUC_J1939_CMD_READ;
    f.data[3] = 1;
    put_u32(f.data, 4, 0x00010203U);
    CHECK(hipnuc_j1939_is_config(&f) == 1);
    CHECK(hipnuc_j1939_parse_config(&f, &source, &addr, &cmd, &status, &value) == 0);
    CHECK(source == 0x08);
    CHECK(status == 1);
    CHECK(value == 0x00010203U);

    /* trigger: register 0x0096 <- PGN */
    hipnuc_j1939_build_trigger(0x08, 0x55, HIPNUC_J1939_PGN_ACC, &f);
    CHECK(f.id == 0x0CEF0855U);
    CHECK(hipnuc_j1939_parse_config(&f, &source, &addr, &cmd, &status, &value) == 0);
    CHECK(addr == 0x0096);
    CHECK(cmd == HIPNUC_J1939_CMD_WRITE);
    CHECK(value == 0xFF34U);

    /* not config: data frame, standard frame, wrong length, remote */
    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    CHECK(hipnuc_j1939_is_config(&f) == 0);
    CHECK(hipnuc_j1939_parse_config(&f, &source, &addr, &cmd, &status, &value) == -1);
    hipnuc_j1939_build_reg_read(0x08, 0x55, 0x0001, &f);
    f.is_extended = 0;
    CHECK(hipnuc_j1939_is_config(&f) == 0);
    CHECK(hipnuc_j1939_parse_config(&f, NULL, NULL, NULL, NULL, NULL) == -1);
    hipnuc_j1939_build_reg_read(0x08, 0x55, 0x0001, &f);
    f.len = 7;
    CHECK(hipnuc_j1939_is_config(&f) == 1);
    CHECK(hipnuc_j1939_parse_config(&f, NULL, NULL, NULL, NULL, NULL) == -1);
    f.len = 8;
    f.is_remote = 1;
    CHECK(hipnuc_j1939_parse_config(&f, NULL, NULL, NULL, NULL, NULL) == -1);
    CHECK(hipnuc_j1939_is_config(NULL) == 0);
    CHECK(hipnuc_j1939_parse_config(NULL, NULL, NULL, NULL, NULL, NULL) == -1);
}

/*
 * CANFD83 default bitmap 0x12B = ACC | GYR | RPY | SYSTEM_TIME | TEMPERATURE.
 * Layout: 0 bitmap, 4 main_status, 6 ins_status, 7 sequence,
 *         8 acc (12), 20 gyr (12), 32 rpy (12), 44 system time (8), 52 temperature (4)
 *         -> 56 logical bytes, padded to a 64-byte frame.
 */
static void test_canfd83_default(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    uint8_t seq = 0;
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 64);
    put_u32(f.data, 0, CANFD83_MAP_DEFAULT);
    put_u16(f.data, 4, (uint16_t)(HIPNUC_STATUS_MAG_DIST | HIPNUC_STATUS_STATIC));
    f.data[6] = HIPNUC_INS_ALIGNING;
    f.data[7] = 0xA5;
    put_f32(f.data, 8, 0.25f);
    put_f32(f.data, 12, -0.5f);
    put_f32(f.data, 16, 9.8f);
    put_f32(f.data, 20, 0.01f);
    put_f32(f.data, 24, -0.02f);
    put_f32(f.data, 28, 0.03f);
    put_f32(f.data, 32, 10.0f);      /* roll 10 deg   -> 0.17453293 rad */
    put_f32(f.data, 36, -20.0f);     /* pitch -20 deg -> -0.34906585 rad */
    put_f32(f.data, 40, 350.0f);     /* yaw 350 deg   -> 6.10865238 rad */
    put_u64(f.data, 44, UINT64_C(0x0000000123456789));
    put_f32(f.data, 52, 36.5f);
    memset(f.data + 56, 0xEE, 8);    /* padding must be ignored */

    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(seq == 0xA5);
    CHECK(s.source == HIPNUC_SOURCE_CANFD83);
    CHECK(s.node_id == 0x08);
    CHECK(s.valid & HIPNUC_VALID_NODE_ID);
    CHECK(s.valid & HIPNUC_VALID_STATUS);
    CHECK(s.main_status == (HIPNUC_STATUS_MAG_DIST | HIPNUC_STATUS_STATIC));
    CHECK(s.magnetic_disturbance == 1);
    CHECK(s.device_static == 1);
    CHECK(s.gyro_bias_converged == 1);
    CHECK(s.attitude_converged == 1);
    CHECK(s.valid & HIPNUC_VALID_INS_STATUS);
    CHECK(s.ins_status == HIPNUC_INS_ALIGNING);
    CHECK(s.valid & HIPNUC_VALID_ACC);
    CHECK(near(s.acc[0], 0.25, FTOL));
    CHECK(near(s.acc[1], -0.5, FTOL));
    CHECK(near(s.acc[2], 9.8, FTOL));
    CHECK(s.valid & HIPNUC_VALID_GYR);
    CHECK(near(s.gyr[0], 0.01, FTOL));
    CHECK(near(s.gyr[1], -0.02, FTOL));
    CHECK(near(s.gyr[2], 0.03, FTOL));
    CHECK(!(s.valid & HIPNUC_VALID_MAG));
    CHECK(s.valid & HIPNUC_VALID_EULER);
    CHECK(near(s.roll, 0.17453293, ATOL));
    CHECK(near(s.pitch, -0.34906585, ATOL));
    CHECK(near(s.yaw, 6.10865238, ATOL));
    CHECK(!(s.valid & HIPNUC_VALID_QUAT));
    CHECK(s.valid & HIPNUC_VALID_DEVICE_TIME);
    CHECK(s.device_time_us == UINT64_C(0x0000000123456789));
    CHECK(!(s.valid & HIPNUC_VALID_UTC));
    CHECK(s.valid & HIPNUC_VALID_TEMPERATURE);
    CHECK(near(s.temperature, 36.5, FTOL));

    /* truncated: 48 < 56 logical bytes */
    f.len = 48;
    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == -1);
    /* exactly the logical length is fine */
    f.len = 56;
    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == HIPNUC_J1939_MSG_CANFD83);
    /* the sequence pointer is optional */
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
}

/* Minimal frame: only ACC, 8 + 12 = 20 bytes */
static void test_canfd83_minimal(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    uint8_t seq = 0;
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 20);
    put_u32(f.data, 0, CANFD83_MAP_ACC_B);
    f.data[7] = 7;
    put_f32(f.data, 8, 1.0f);
    put_f32(f.data, 12, 2.0f);
    put_f32(f.data, 16, -3.0f);
    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(seq == 7);
    CHECK((s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_EULER | HIPNUC_VALID_TEMPERATURE)) == HIPNUC_VALID_ACC);
    CHECK(near(s.acc[0], 1.0, FTOL));
    CHECK(near(s.acc[1], 2.0, FTOL));
    CHECK(near(s.acc[2], -3.0, FTOL));
    f.len = 19;
    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == -1);
    /* header alone is too short even for the length check */
    f.len = 7;
    CHECK(hipnuc_j1939_parse(&f, &s, &seq) == -1);
}

/*
 * Bitmap 0x1B = ACC | GYR | RPY | QUAT: 8 + 12 + 12 + 12 + 16 = 60 bytes,
 * fits a 64-byte frame. Bitmap 0x7F (bits 0..6): 8 + 4*12 + 16 + 8 + 8 = 88
 * bytes, cannot fit; the maximal 0x17F needs 92.
 */
static void test_canfd83_combinations(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 64);
    put_u32(f.data, 0, 0x1B);
    put_f32(f.data, 8, 0.1f);
    put_f32(f.data, 20, 0.2f);
    put_f32(f.data, 32, 90.0f);       /* roll 90 deg -> 1.57079633 rad */
    put_f32(f.data, 44, 1.0f);        /* quat w */
    put_f32(f.data, 48, 0.0f);
    put_f32(f.data, 52, -0.5f);
    put_f32(f.data, 56, 0.25f);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(s.valid & HIPNUC_VALID_ACC);
    CHECK(s.valid & HIPNUC_VALID_GYR);
    CHECK(s.valid & HIPNUC_VALID_EULER);
    CHECK(s.valid & HIPNUC_VALID_QUAT);
    CHECK(!(s.valid & HIPNUC_VALID_MAG));
    CHECK(!(s.valid & HIPNUC_VALID_DEVICE_TIME));
    CHECK(near(s.acc[0], 0.1, FTOL));
    CHECK(near(s.gyr[0], 0.2, FTOL));
    CHECK(near(s.roll, 1.57079633, ATOL));
    CHECK(near(s.quat[0], 1.0, FTOL));
    CHECK(near(s.quat[1], 0.0, FTOL));
    CHECK(near(s.quat[2], -0.5, FTOL));
    CHECK(near(s.quat[3], 0.25, FTOL));
    f.len = 60;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    f.len = 59;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);

    /* MAG scaling: bitmap 0x04, 8 + 12 = 20 bytes; 25 uT -> 2.5e-5 T */
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 24);
    put_u32(f.data, 0, CANFD83_MAP_MAG_B);
    put_f32(f.data, 8, 25.0f);
    put_f32(f.data, 12, -40.0f);
    put_f32(f.data, 16, 0.0f);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(s.valid & HIPNUC_VALID_MAG);
    CHECK(near(s.mag[0], 2.5e-5, 1e-10));
    CHECK(near(s.mag[1], -4.0e-5, 1e-10));

    /* too large for any frame */
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 64);
    put_u32(f.data, 0, 0x7F);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    put_u32(f.data, 0, CANFD83_MAP_SUPPORTED);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);

    /* unsupported bits (7: pressure, 9: inclination) and an empty bitmap */
    put_u32(f.data, 0, CANFD83_MAP_ACC_B | (UINT32_C(1) << 7));
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    put_u32(f.data, 0, CANFD83_MAP_ACC_B | (UINT32_C(1) << 9));
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    put_u32(f.data, 0, UINT32_C(1) << 31);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    put_u32(f.data, 0, 0);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);

    /* remote / error / standard */
    put_u32(f.data, 0, CANFD83_MAP_ACC_B);
    f.is_remote = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    f.is_remote = 0;
    f.is_error = 1;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
    f.is_error = 0;
    f.is_extended = 0;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_NONE);
}

/* UTC only: bitmap 0x40, 8 + 8 = 16 bytes; sec_ms = 45 * 1000 + 678 = 45678 */
static void test_canfd83_utc(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t s;
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 16);
    put_u32(f.data, 0, CANFD83_MAP_UTC);
    put_u16(f.data, 4, 0);
    f.data[8] = 26;
    f.data[9] = 9;
    f.data[10] = 7;
    f.data[11] = 23;
    f.data[12] = 59;
    put_u16(f.data, 13, 45678);
    f.data[15] = 0;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(s.valid & HIPNUC_VALID_UTC);
    CHECK(s.utc.year == 2026);
    CHECK(s.utc.month == 9);
    CHECK(s.utc.day == 7);
    CHECK(s.utc.hour == 23);
    CHECK(s.utc.minute == 59);
    CHECK(s.utc.second == 45);
    CHECK(s.utc.millisecond == 678);

    /* same payload, device not synchronized to UTC */
    put_u16(f.data, 4, HIPNUC_STATUS_UTC_UNSYNC);
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(!(s.valid & HIPNUC_VALID_UTC));
    CHECK(s.valid & HIPNUC_VALID_STATUS);
    CHECK(s.main_status == HIPNUC_STATUS_UTC_UNSYNC);

    /* synchronized flag but a zero date is still not a calendar time */
    put_u16(f.data, 4, 0);
    f.data[8] = 0;
    f.data[9] = 0;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == HIPNUC_J1939_MSG_CANFD83);
    CHECK(!(s.valid & HIPNUC_VALID_UTC));
    f.len = 15;
    CHECK(hipnuc_j1939_parse(&f, &s, NULL) == -1);
}

static void test_merge(void)
{
    hipnuc_can_frame_t f;
    hipnuc_sample_t merged, part;

    hipnuc_sample_clear(&merged);
    CHECK(merged.source == HIPNUC_SOURCE_NONE);

    make_frame(&f, HIPNUC_J1939_PGN_ACC, 0x08, 8);
    put_i16(f.data, 4, 2048);                        /* acc z 9.8 */
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(merged.source == HIPNUC_SOURCE_J1939);
    CHECK(merged.node_id == 0x08);

    make_frame(&f, HIPNUC_J1939_PGN_GYR, 0x08, 8);
    put_i16(f.data, 0, 16384);                       /* gyr x 1000 deg/s */
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);

    /* YAW first, then ROLL_PITCH: the yaw must survive the roll/pitch merge */
    make_frame(&f, HIPNUC_J1939_PGN_YAW, 0x08, 8);
    put_i32(f.data, 0, 90000);                       /* heading 90 deg */
    put_i32(f.data, 4, -90000);                      /* yaw -90 deg */
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);

    make_frame(&f, HIPNUC_J1939_PGN_ROLL_PITCH, 0x08, 8);
    put_i32(f.data, 0, 45000);                       /* roll 45 deg */
    put_i32(f.data, 4, -30000);                      /* pitch -30 deg */
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);

    CHECK(merged.valid == (HIPNUC_VALID_NODE_ID | HIPNUC_VALID_ACC | HIPNUC_VALID_GYR |
                           HIPNUC_VALID_HEADING | HIPNUC_VALID_EULER));
    CHECK(near(merged.acc[2], 9.8, FTOL));
    CHECK(near(merged.acc[0], 0.0, FTOL));
    CHECK(near(merged.gyr[0], RAD(1000.0), 1e-4));
    CHECK(near(merged.roll, RAD(45.0), ATOL));
    CHECK(near(merged.pitch, RAD(-30.0), ATOL));
    CHECK(near(merged.heading, RAD(90.0), ATOL));
    CHECK(near(merged.yaw, RAD(-90.0), ATOL));

    /* ROLL_PITCH before YAW also ends with the yaw from the YAW frame */
    hipnuc_sample_clear(&merged);
    make_frame(&f, HIPNUC_J1939_PGN_ROLL_PITCH, 0x08, 8);
    put_i32(f.data, 0, 45000);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(merged.yaw == 0.0f);
    make_frame(&f, HIPNUC_J1939_PGN_YAW, 0x08, 8);
    put_i32(f.data, 0, 90000);
    put_i32(f.data, 4, -90000);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(near(merged.roll, RAD(45.0), ATOL));
    CHECK(near(merged.yaw, RAD(-90.0), ATOL));

    /* POSITION + ALTITUDE in either order give a full position */
    hipnuc_sample_clear(&merged);
    make_frame(&f, HIPNUC_J1939_PGN_ALTITUDE, 0x08, 8);
    put_i32(f.data, 0, 123456);                      /* 1234.56 m */
    put_i16(f.data, 4, -350);
    put_i16(f.data, 6, 150);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    make_frame(&f, HIPNUC_J1939_PGN_POSITION, 0x08, 8);
    put_i32(f.data, 0, 316000000);
    put_i32(f.data, 4, -1213000000);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(merged.valid & HIPNUC_VALID_POSITION);
    CHECK(merged.valid & HIPNUC_VALID_UNDULATION);
    CHECK(merged.valid & HIPNUC_VALID_DIFF_AGE);
    CHECK(near(merged.latitude, 31.6, 1e-9));
    CHECK(near(merged.longitude, -121.3, 1e-9));
    CHECK(near(merged.altitude_msl, 1234.56, 1e-9));
    CHECK(near(merged.undulation, -3.5, FTOL));
    CHECK(near(merged.diff_age, 1.5, FTOL));

    /* status, ins status, temperature and UTC from other frames */
    make_frame(&f, HIPNUC_J1939_PGN_GNSS_STATUS, 0x08, 8);
    f.data[0] = 4;
    f.data[4] = HIPNUC_INS_NAVIGATING;
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    make_frame(&f, HIPNUC_J1939_PGN_TEMP, 0x08, 8);
    put_i16(f.data, 0, 2534);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    make_frame(&f, HIPNUC_J1939_PGN_TIME, 0x08, 8);
    f.data[0] = 26;
    f.data[1] = 9;
    f.data[2] = 7;
    put_u16(f.data, 6, 5);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(merged.valid & HIPNUC_VALID_GNSS_QUALITY);
    CHECK(merged.valid & HIPNUC_VALID_INS_STATUS);
    CHECK(merged.valid & HIPNUC_VALID_TEMPERATURE);
    CHECK(merged.valid & HIPNUC_VALID_UTC);
    CHECK(merged.position_quality == 4);
    CHECK(merged.ins_status == HIPNUC_INS_NAVIGATING);
    CHECK(near(merged.temperature, 25.34, FTOL));
    CHECK(merged.utc.year == 2026 && merged.utc.millisecond == 5);

    /* a CANFD83 part contributes its status flags */
    make_frame(&f, HIPNUC_J1939_PGN_CANFD83, 0x08, 20);
    put_u32(f.data, 0, CANFD83_MAP_ACC_B);
    put_u16(f.data, 4, HIPNUC_STATUS_WB_CONV);
    put_f32(f.data, 16, -9.8f);
    CHECK(hipnuc_j1939_parse(&f, &part, NULL) > 0);
    hipnuc_j1939_merge(&merged, &part);
    CHECK(merged.valid & HIPNUC_VALID_STATUS);
    CHECK(merged.main_status == HIPNUC_STATUS_WB_CONV);
    CHECK(merged.gyro_bias_converged == 0);
    CHECK(near(merged.acc[2], -9.8, FTOL));
    CHECK(merged.source == HIPNUC_SOURCE_J1939);   /* first source wins */
}

int main(void)
{
    test_id_helpers();
    test_acc();
    test_gyr();
    test_mag();
    test_roll_pitch();
    test_yaw();
    test_temp();
    test_quat();
    test_inclination();
    test_time();
    test_position();
    test_altitude();
    test_gnss_status();
    test_velocity();
    test_rejections();
    test_source_addresses();
    test_config_frames();
    test_canfd83_default();
    test_canfd83_minimal();
    test_canfd83_combinations();
    test_canfd83_utc();
    test_merge();

    printf("test_j1939: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
