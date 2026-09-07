/*
 * Host tests for hipnuc_dec, nmea_dec, hipnuc_sample and hipnuc_json.
 *
 * Frames are built byte by byte with an independent CRC and every expected
 * value is computed by hand from the published field layout, so the tests
 * do not depend on the decoder for their expectations.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hipnuc_dec.h"
#include "hipnuc_json.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"

static int failures;

#define CHECK(cond) do { \
    if (!(cond)) { printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); failures++; } \
} while (0)

static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* ---- independent frame construction ------------------------------------ */

static uint16_t crc_xmodem(const uint8_t *data, size_t len, uint16_t crc)
{
    size_t i;
    int b;
    for (i = 0; i < len; ++i) {
        crc ^= (uint16_t)(data[i] << 8);
        for (b = 0; b < 8; ++b) crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}

static size_t frame(uint8_t *out, const uint8_t *payload, size_t len)
{
    uint16_t crc;
    out[0] = 0x5A; out[1] = 0xA5;
    out[2] = (uint8_t)(len & 0xFF); out[3] = (uint8_t)(len >> 8);
    crc = crc_xmodem(out, 4, 0);
    crc = crc_xmodem(payload, len, crc);
    out[4] = (uint8_t)(crc & 0xFF); out[5] = (uint8_t)(crc >> 8);
    memcpy(out + 6, payload, len);
    return len + 6;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { put_u16(p, (uint16_t)v); put_u16(p + 2, (uint16_t)(v >> 16)); }
static void put_i16(uint8_t *p, int16_t v) { put_u16(p, (uint16_t)v); }
static void put_i32(uint8_t *p, int32_t v) { put_u32(p, (uint32_t)v); }
static void put_f32(uint8_t *p, float v) { uint32_t u; memcpy(&u, &v, 4); put_u32(p, u); }
static void put_f64(uint8_t *p, double v) { uint64_t u; memcpy(&u, &v, 8); put_u32(p, (uint32_t)u); put_u32(p + 4, (uint32_t)(u >> 32)); }

static int feed(hipnuc_raw_t *raw, const uint8_t *data, size_t len)
{
    size_t i;
    int last = 0, frames = 0;
    for (i = 0; i < len; ++i) {
        last = hipnuc_input(raw, data[i]);
        if (last > 0) frames++;
        if (last < 0) return -1;
    }
    return frames;
}

/* ---- HI91 ---------------------------------------------------------------- */

static size_t build_hi91(uint8_t *out)
{
    uint8_t p[76];
    memset(p, 0, sizeof(p));
    p[0] = 0x91;
    put_u16(p + 1, (1 << 3) | (1 << 11));   /* WB_CONV warning, UTC not synced */
    p[3] = (uint8_t)(int8_t)-7;             /* -7 degC */
    put_f32(p + 4, 101325.0f);
    put_u32(p + 8, 123456);                 /* ms */
    put_f32(p + 12, 1.0f); put_f32(p + 16, -0.5f); put_f32(p + 20, 0.25f);   /* G */
    put_f32(p + 24, 180.0f); put_f32(p + 28, -90.0f); put_f32(p + 32, 45.0f); /* deg/s */
    put_f32(p + 36, 10.0f); put_f32(p + 40, -20.0f); put_f32(p + 44, 30.0f);  /* uT */
    put_f32(p + 48, 10.0f); put_f32(p + 52, -20.0f); put_f32(p + 56, 30.0f);  /* deg */
    put_f32(p + 60, 1.0f); put_f32(p + 64, 0.0f); put_f32(p + 68, 0.0f); put_f32(p + 72, 0.0f);
    return frame(out, p, sizeof(p));
}

static void test_hi91(void)
{
    uint8_t f[128];
    hipnuc_raw_t raw;
    hipnuc_sample_t s;
    size_t n = build_hi91(f);
    memset(&raw, 0, sizeof(raw));

    CHECK(feed(&raw, f, n) == 1);
    CHECK(raw.hi91.tag == 0x91);
    CHECK(raw.hi81.tag == 0 && raw.hi83.tag == 0);
    CHECK(raw.hi91.main_status == 0x0808);
    CHECK(raw.hi91.temp == -7);
    CHECK(raw.hi91.system_time == 123456);
    CHECK(raw.hi91.acc[1] == -0.5f);
    CHECK(raw.frame_count == 1 && raw.crc_error_count == 0);

    CHECK(hipnuc_sample_from_raw(&raw, &s) == 1);
    CHECK(s.source == HIPNUC_SOURCE_HI91);
    CHECK(near(s.acc[0], 9.8, 1e-6));            /* 1 G x 9.8 */
    CHECK(near(s.acc[1], -4.9, 1e-6));
    CHECK(near(s.acc[2], 2.45, 1e-6));
    CHECK(near(s.gyr[0], 3.14159265, 1e-6));     /* 180 deg/s */
    CHECK(near(s.gyr[1], -1.57079633, 1e-6));
    CHECK(near(s.mag[0], 10e-6, 1e-12));
    CHECK(near(s.roll, 0.17453293, 1e-6));       /* 10 deg */
    CHECK(near(s.yaw, 0.52359878, 1e-6));        /* 30 deg */
    CHECK(s.quat[0] == 1.0f && s.quat[3] == 0.0f);
    CHECK(s.temperature == -7.0f);
    CHECK(near(s.pressure, 101325.0, 0.01));
    CHECK(s.device_time_us == 123456000ULL);
    CHECK(s.gyro_bias_converged == 0);           /* WB_CONV set = not converged */
    CHECK(s.attitude_converged == 1);
    CHECK(s.magnetic_disturbance == 0);
    CHECK((s.valid & HIPNUC_VALID_UTC) == 0);
    CHECK((s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG | HIPNUC_VALID_EULER |
                      HIPNUC_VALID_QUAT | HIPNUC_VALID_DEVICE_TIME)) ==
          (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_MAG | HIPNUC_VALID_EULER |
           HIPNUC_VALID_QUAT | HIPNUC_VALID_DEVICE_TIME));
    CHECK((s.valid & HIPNUC_VALID_POSITION) == 0);
}

/* ---- HI81 ---------------------------------------------------------------- */

static size_t build_hi81(uint8_t *out)
{
    uint8_t p[104];
    memset(p, 0, sizeof(p));
    p[0] = 0x81;
    put_u16(p + 1, 0);                      /* everything converged, UTC synced */
    p[3] = 3;                               /* navigating */
    put_u16(p + 4, 2400);                   /* GPS week */
    put_u32(p + 6, 123456789);              /* tow ms */
    put_i16(p + 12, 1000); put_i16(p + 14, -500); put_i16(p + 16, 250);    /* gyr x0.001 rad/s */
    put_i16(p + 18, 100); put_i16(p + 20, -200); put_i16(p + 22, 2048);    /* acc x0.0048828 */
    put_i16(p + 24, 100); put_i16(p + 26, -200); put_i16(p + 28, 300);     /* mag x0.030517 uT */
    put_i16(p + 30, 2000);                  /* pressure 102000 Pa */
    put_i16(p + 32, 1234);                  /* odometer 12.34 m/s */
    p[34] = (uint8_t)(int8_t)-5;
    p[35] = 26; p[36] = 9; p[37] = 6; p[38] = 12; p[39] = 34; put_u16(p + 40, 56789);
    put_i16(p + 42, 1000); put_i16(p + 44, -2000); put_u16(p + 46, 9000);   /* 10, -20, 90 deg */
    put_i16(p + 48, 10000); put_i16(p + 50, 0); put_i16(p + 52, 0); put_i16(p + 54, 0);
    put_i32(p + 56, 1121234567); put_i32(p + 60, 281234567); put_i32(p + 64, 123456);
    p[68] = 15; p[69] = 8; p[70] = 4; p[71] = 22; p[72] = 4; p[73] = 18; p[74] = 2;
    put_i16(p + 75, -1234);                 /* undulation -12.34 m */
    p[77] = 1;
    put_i16(p + 78, 123); put_i16(p + 80, -456); put_i16(p + 82, 789);
    put_i16(p + 84, 100); put_i16(p + 86, 0); put_i16(p + 88, 0);
    p[90] = 0xAA; p[103] = 0x91;            /* reserved tail must be ignored */
    return frame(out, p, sizeof(p));
}

static void test_hi81(void)
{
    uint8_t f[128];
    hipnuc_raw_t raw;
    hipnuc_sample_t s;
    size_t n = build_hi81(f);
    memset(&raw, 0, sizeof(raw));

    CHECK(feed(&raw, f, n) == 1);
    CHECK(raw.hi81.tag == 0x81);
    CHECK(raw.hi81.gpst_wn == 2400 && raw.hi81.gpst_tow == 123456789);
    CHECK(raw.hi81.yaw == 9000);
    CHECK(raw.hi81.ins_lon == 1121234567);
    CHECK(raw.hi81.reserved_tail[0] == 0xAA && raw.hi81.reserved_tail[13] == 0x91);

    CHECK(hipnuc_sample_from_raw(&raw, &s) == 1);
    CHECK(s.source == HIPNUC_SOURCE_HI81);
    CHECK(s.ins_status == HIPNUC_INS_NAVIGATING);
    CHECK(near(s.gyr[0], 1.0, 1e-6) && near(s.gyr[1], -0.5, 1e-6));
    CHECK(near(s.acc[0], 0.48828, 1e-5));
    CHECK(near(s.acc[2], 9.9999744, 1e-4));     /* 2048 x 0.0048828 */
    CHECK(near(s.mag[0], 3.0517e-6, 1e-10));
    CHECK(near(s.pressure, 102000.0, 0.01));
    CHECK(near(s.odometer_speed, 12.34, 1e-5));
    CHECK(s.temperature == -5.0f);
    CHECK(near(s.roll, 0.17453293, 1e-6));
    CHECK(near(s.pitch, -0.34906585, 1e-6));
    CHECK(near(s.heading, 1.57079633, 1e-6));    /* 90 deg clockwise */
    CHECK(near(s.quat[0], 1.0, 1e-6));
    CHECK(near(s.longitude, 112.1234567, 1e-9));
    CHECK(near(s.latitude, 28.1234567, 1e-9));
    CHECK(near(s.altitude_msl, 123.456, 1e-9));
    CHECK(near(s.pdop, 1.5, 1e-6) && near(s.hdop, 0.8, 1e-6));
    CHECK(s.position_quality == 4 && s.position_satellites == 22);
    CHECK(s.heading_quality == 4 && s.heading_satellites == 18);
    CHECK(s.diff_age == 2.0f);
    CHECK(near(s.undulation, -12.34, 1e-5));
    CHECK(near(s.vel_enu[0], 1.23, 1e-6) && near(s.vel_enu[1], -4.56, 1e-6));
    CHECK(near(s.acc_enu[0], 0.48828, 1e-5));
    CHECK((s.valid & HIPNUC_VALID_UTC) != 0);
    CHECK(s.utc.year == 2026 && s.utc.month == 9 && s.utc.day == 6);
    CHECK(s.utc.hour == 12 && s.utc.minute == 34 && s.utc.second == 56 && s.utc.millisecond == 789);
    CHECK((s.valid & HIPNUC_VALID_GPS_TIME) != 0);
    CHECK(s.gps_week == 2400);
}

/* ---- HI83 ---------------------------------------------------------------- */

static size_t build_hi83(uint8_t *out, uint32_t bitmap, uint16_t status)
{
    uint8_t p[300];
    size_t i = 8;
    memset(p, 0, sizeof(p));
    p[0] = 0x83;
    put_u16(p + 1, status);
    p[3] = 6;                                  /* dead reckoning */
    put_u32(p + 4, bitmap);
    if (bitmap & HI83_BMAP_ACC_B) { put_f32(p + i, 9.8f); put_f32(p + i + 4, -4.9f); put_f32(p + i + 8, 2.45f); i += 12; }
    if (bitmap & HI83_BMAP_GYR_B) { put_f32(p + i, 1.0f); put_f32(p + i + 4, -0.5f); put_f32(p + i + 8, 0.25f); i += 12; }
    if (bitmap & HI83_BMAP_MAG_B) { put_f32(p + i, 10.0f); put_f32(p + i + 4, -20.0f); put_f32(p + i + 8, 30.0f); i += 12; }
    if (bitmap & HI83_BMAP_RPY) { put_f32(p + i, 10.0f); put_f32(p + i + 4, -20.0f); put_f32(p + i + 8, 30.0f); i += 12; }
    if (bitmap & HI83_BMAP_QUAT) { put_f32(p + i, 1.0f); put_f32(p + i + 4, 0); put_f32(p + i + 8, 0); put_f32(p + i + 12, 0); i += 16; }
    if (bitmap & HI83_BMAP_SYSTEM_TIME) { put_u32(p + i, 0xC6D1A9B2U); put_u32(p + i + 4, 0x62U); i += 8; } /* 424242424242 us = 0x62C6D1A9B2 */
    if (bitmap & HI83_BMAP_UTC) { p[i] = 26; p[i + 1] = 9; p[i + 2] = 6; p[i + 3] = 12; p[i + 4] = 34; put_u16(p + i + 5, 56789); i += 8; }
    if (bitmap & HI83_BMAP_AIR_PRESSURE) { put_f32(p + i, 101325.0f); i += 4; }
    if (bitmap & HI83_BMAP_TEMPERATURE) { put_f32(p + i, 24.5f); i += 4; }
    if (bitmap & HI83_BMAP_INCLINATION) { put_f32(p + i, 5.0f); put_f32(p + i + 4, -6.0f); put_f32(p + i + 8, 7.0f); i += 12; }
    if (bitmap & HI83_BMAP_HSS) { put_f32(p + i, 1.0f); put_f32(p + i + 4, 2.0f); put_f32(p + i + 8, 3.0f); i += 12; }
    if (bitmap & HI83_BMAP_HSS_FRQ) { put_f32(p + i, 0.1f); put_f32(p + i + 4, 0.2f); put_f32(p + i + 8, 0.3f); i += 12; }
    if (bitmap & HI83_BMAP_VEL_ENU) { put_f32(p + i, 1.0f); put_f32(p + i + 4, 2.0f); put_f32(p + i + 8, 3.0f); i += 12; }
    if (bitmap & HI83_BMAP_ACC_ENU) { put_f32(p + i, 0.1f); put_f32(p + i + 4, -0.2f); put_f32(p + i + 8, 0.3f); i += 12; }
    if (bitmap & HI83_BMAP_INS_LON_LAT_MSL) { put_f64(p + i, 112.1234567); put_f64(p + i + 8, 28.1234567); put_f64(p + i + 16, 123.456); i += 24; }
    if (bitmap & HI83_BMAP_GNSS_QUALITY_NV) { p[i] = 4; p[i + 1] = 22; p[i + 2] = 4; p[i + 3] = 18; i += 4; }
    if (bitmap & HI83_BMAP_OD_SPEED) { put_f32(p + i, 12.34f); i += 4; }
    if (bitmap & HI83_BMAP_UNDULATION) { put_f32(p + i, -12.34f); i += 4; }
    if (bitmap & HI83_BMAP_DIFF_AGE) { put_f32(p + i, 2.0f); i += 4; }
    if (bitmap & HI83_BMAP_NODE_ID) { p[i] = 8; i += 4; }
    if (bitmap & HI83_BMAP_GNSS_LON_LAT_MSL) { put_f64(p + i, 112.1); put_f64(p + i + 8, 28.1); put_f64(p + i + 16, 123.0); i += 24; }
    if (bitmap & HI83_BMAP_GNSS_VEL) { put_f32(p + i, 4.0f); put_f32(p + i + 4, 5.0f); put_f32(p + i + 8, 6.0f); i += 12; }
    return frame(out, p, i);
}

static void test_hi83(void)
{
    uint8_t f[320];
    hipnuc_raw_t raw;
    hipnuc_sample_t s;
    size_t n;

    /* Every supported bit (bits 0..19, 30, 31) -> 236 byte payload */
    n = build_hi83(f, HI83_BMAP_SUPPORTED, 0);
    CHECK(n == 236 + 6);
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, f, n) == 1);
    CHECK(raw.hi83.tag == 0x83 && raw.hi83.data_bitmap == HI83_BMAP_SUPPORTED);
    CHECK(raw.hi83.system_time_us == 424242424242ULL);
    CHECK(raw.hi83.node.node_id == 8);
    CHECK(raw.hi83.gnss_vel[2] == 6.0f);
    CHECK(hipnuc_sample_from_raw(&raw, &s) == 1);
    CHECK(s.source == HIPNUC_SOURCE_HI83);
    CHECK(near(s.acc[0], 9.8, 1e-6));            /* already m/s^2 */
    CHECK(near(s.gyr[1], -0.5, 1e-6));           /* already rad/s */
    CHECK(near(s.mag[2], 30e-6, 1e-12));
    CHECK(near(s.roll, 0.17453293, 1e-6) && near(s.yaw, 0.52359878, 1e-6));
    CHECK(s.device_time_us == 424242424242ULL);
    CHECK((s.valid & HIPNUC_VALID_UTC) != 0 && s.utc.year == 2026 && s.utc.millisecond == 789);
    CHECK(near(s.inclination[0], 0.08726646, 1e-6));   /* 5 deg */
    CHECK(s.heave_m[1] == 2.0f && near(s.heave_hz[2], 0.3, 1e-6));
    CHECK(near(s.longitude, 112.1234567, 1e-12) && near(s.altitude_msl, 123.456, 1e-12));
    CHECK(s.position_quality == 4 && s.heading_satellites == 18);
    CHECK(near(s.odometer_speed, 12.34, 1e-5) && near(s.undulation, -12.34, 1e-5) && s.diff_age == 2.0f);
    CHECK(s.node_id == 8 && (s.valid & HIPNUC_VALID_NODE_ID));
    CHECK(near(s.gnss_longitude, 112.1, 1e-12) && s.gnss_vel_enu[0] == 4.0f);
    CHECK(s.ins_status == HIPNUC_INS_DEAD_RECKONING);

    /* Default map 0xFF: acc, gyr, mag, rpy, quat, time, utc, pressure -> 8+12*4+16+8+8+4 = 92 */
    n = build_hi83(f, 0xFF, 1 << 11);
    CHECK(n == 92 + 6);
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, f, n) == 1);
    CHECK(hipnuc_sample_from_raw(&raw, &s) == 1);
    CHECK((s.valid & HIPNUC_VALID_UTC) == 0);    /* UTC_UNSYNC set: no UTC */
    CHECK((s.valid & HIPNUC_VALID_TEMPERATURE) == 0);
    CHECK((s.valid & HIPNUC_VALID_POSITION) == 0);
    CHECK(near(s.pressure, 101325.0, 0.01));

    /* Empty bitmap: header only */
    n = build_hi83(f, 0, 0);
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, f, n) == 1);
    CHECK(hipnuc_sample_from_raw(&raw, &s) == 1);
    CHECK(s.valid == (HIPNUC_VALID_STATUS | HIPNUC_VALID_INS_STATUS));

    /* Internal bit 25 -> rejected, no tag survives */
    n = build_hi83(f, HI83_BMAP_ACC_B | (UINT32_C(1) << 25), 0);
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, f, n) == -1);
    CHECK(raw.hi83.tag == 0 && raw.invalid_count == 1);

    /* Legacy 4-byte timestamp (bitmap says 8): length mismatch -> rejected */
    {
        uint8_t p[8 + 12 + 4];
        memset(p, 0, sizeof(p));
        p[0] = 0x83; put_u32(p + 4, HI83_BMAP_ACC_B | HI83_BMAP_SYSTEM_TIME);
        n = frame(f, p, sizeof(p));
        memset(&raw, 0, sizeof(raw));
        CHECK(feed(&raw, f, n) == -1);
    }

    /* Truncated by one byte -> rejected */
    n = build_hi83(f, 0xFF, 0);
    f[2] = (uint8_t)(91 & 0xFF);
    {
        uint16_t crc = crc_xmodem(f, 4, 0);
        crc = crc_xmodem(f + 6, 91, crc);
        f[4] = (uint8_t)crc; f[5] = (uint8_t)(crc >> 8);
    }
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, f, 91 + 6) == -1);
}

/* ---- framing ------------------------------------------------------------ */

static void test_framing(void)
{
    uint8_t a[128], b[128], stream[512];
    hipnuc_raw_t raw;
    hipnuc_sample_t s;
    size_t na = build_hi91(a), nb = build_hi81(b), n = 0, consumed = 0, i;
    int ret;

    /* noise, frame, noise, frame */
    stream[n++] = 0x00; stream[n++] = 0x5A; stream[n++] = 0x5A;
    memcpy(stream + n, a, na); n += na;
    stream[n++] = 0xA5; stream[n++] = 0x5A;
    memcpy(stream + n, b, nb); n += nb;
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, stream, n) == 2);
    CHECK(raw.frame_count == 2);
    CHECK(raw.hi81.tag == 0x81 && raw.hi91.tag == 0);   /* tags cleared per frame */

    /* CRC error: nothing survives, counter increments */
    memcpy(stream, a, na);
    stream[20] ^= 0xFF;
    memset(&raw, 0, sizeof(raw));
    CHECK(feed(&raw, stream, na) == -1);
    CHECK(raw.crc_error_count == 1 && raw.hi91.tag == 0);
    CHECK(hipnuc_sample_from_raw(&raw, &s) == 0);

    /* The frame after a CRC error still decodes */
    CHECK(feed(&raw, a, na) == 1);
    CHECK(raw.hi91.tag == 0x91);

    /* Announced length too large -> rejected without buffering 65 KiB */
    memset(&raw, 0, sizeof(raw));
    stream[0] = 0x5A; stream[1] = 0xA5; stream[2] = 0xFF; stream[3] = 0xFF; stream[4] = 0; stream[5] = 0;
    CHECK(feed(&raw, stream, 6) == -1);
    CHECK(raw.nbyte == 0);

    /* Length zero -> rejected */
    memset(&raw, 0, sizeof(raw));
    stream[2] = 0; stream[3] = 0;
    CHECK(feed(&raw, stream, 6) == -1);

    /* Unknown tag -> rejected */
    {
        uint8_t p[4] = { 0x90, 1, 2, 3 };
        size_t nf = frame(stream, p, 4);
        memset(&raw, 0, sizeof(raw));
        CHECK(feed(&raw, stream, nf) == -1);
        CHECK(raw.invalid_count == 1);
    }

    /* Two sub-packets in one frame */
    {
        uint8_t p[76 + 104];
        size_t nf;
        memcpy(p, a + 6, 76);
        memcpy(p + 76, b + 6, 104);
        nf = frame(stream, p, sizeof(p));
        memset(&raw, 0, sizeof(raw));
        CHECK(feed(&raw, stream, nf) == 1);
        CHECK(raw.hi91.tag == 0x91 && raw.hi81.tag == 0x81);
        CHECK(hipnuc_sample_from_raw(&raw, &s) == 1 && s.source == HIPNUC_SOURCE_HI81);
    }

    /* Truncated sub-packet inside a valid frame -> rejected */
    {
        uint8_t p[75];
        size_t nf;
        memcpy(p, a + 6, 75);
        nf = frame(stream, p, sizeof(p));
        memset(&raw, 0, sizeof(raw));
        CHECK(feed(&raw, stream, nf) == -1);
    }

    /* Buffer input stops after the first frame and reports consumed bytes */
    memcpy(stream, a, na); memcpy(stream + na, b, nb);
    memset(&raw, 0, sizeof(raw));
    ret = hipnuc_input_buffer(&raw, stream, na + nb, &consumed);
    CHECK(ret == 1 && consumed == na && raw.hi91.tag == 0x91);
    ret = hipnuc_input_buffer(&raw, stream + consumed, na + nb - consumed, &consumed);
    CHECK(ret == 1 && consumed == nb && raw.hi81.tag == 0x81);

    /* Every split point of a frame yields exactly one decode */
    for (i = 0; i < na; ++i) {
        memset(&raw, 0, sizeof(raw));
        ret = feed(&raw, a, i);
        CHECK(ret == 0);
        ret = feed(&raw, a + i, na - i);
        CHECK(ret == 1);
        CHECK(raw.hi91.tag == 0x91);
    }

    /* A frame whose payload ends with 0x5A must not poison the next sync */
    {
        uint8_t p[76];
        size_t nf;
        memcpy(p, a + 6, 76);
        put_f32(p + 72, 0.0f);
        p[75] = 0x5A;                      /* last byte of quat[3] */
        nf = frame(stream, p, sizeof(p));
        memcpy(stream + nf, a, na);
        memset(&raw, 0, sizeof(raw));
        CHECK(feed(&raw, stream, nf + na) == 2);
    }
}

/* ---- NMEA ---------------------------------------------------------------- */

static void nmea_line(char *out, const char *body)
{
    const char *p;
    uint8_t sum = 0;
    for (p = body; *p; ++p) sum ^= (uint8_t)*p;
    sprintf(out, "$%s*%02X\r\n", body, sum);
}

static int feed_nmea(nmea_raw_t *raw, const char *line)
{
    int last = 0;
    while (*line) {
        last = nmea_input(raw, (uint8_t)*line++);
        if (last != 0) return last;
    }
    return 0;
}

static void test_nmea(void)
{
    char line[160];
    nmea_raw_t raw;
    hipnuc_sample_t s;
    memset(&raw, 0, sizeof(raw));

    nmea_line(line, "GPGGA,123519.50,4807.038,N,01131.000,E,4,08,0.9,545.4,M,46.9,M,2.5,0123");
    CHECK(feed_nmea(&raw, line) == 1);
    CHECK(raw.msg_type == NMEA_MSG_GGA);
    CHECK(strcmp(raw.talker, "GP") == 0);
    CHECK(raw.gga.hour == 12 && raw.gga.minute == 35 && near(raw.gga.second, 19.5, 1e-4));
    CHECK(near(raw.gga.lat, 48.1173, 1e-6) && near(raw.gga.lon, 11.5166667, 1e-6));
    CHECK(raw.gga.quality == 4 && raw.gga.satellites == 8);
    CHECK(near(raw.gga.hdop, 0.9, 1e-6) && near(raw.gga.altitude_msl, 545.4, 1e-6));
    CHECK(near(raw.gga.undulation, 46.9, 1e-5) && near(raw.gga.diff_age, 2.5, 1e-6));
    CHECK(raw.gga.station_id == 123);
    CHECK(raw.gga.has_position && raw.gga.has_time && raw.gga.has_altitude);
    CHECK(hipnuc_sample_from_nmea(&raw, &s) == 1);
    CHECK(s.source == HIPNUC_SOURCE_NMEA_GGA);
    CHECK((s.valid & HIPNUC_VALID_POSITION) && near(s.altitude_msl, 545.4, 1e-6));
    CHECK((s.valid & HIPNUC_VALID_UTC) == 0);    /* no date in GGA */
    CHECK(s.position_quality == 4);

    /* Southern / western hemisphere */
    nmea_line(line, "GNGGA,000000.00,3351.000,S,15112.000,W,1,05,1.0,10.0,M,0.0,M,,");
    CHECK(feed_nmea(&raw, line) == 1);
    CHECK(near(raw.gga.lat, -33.85, 1e-6) && near(raw.gga.lon, -151.2, 1e-6));
    CHECK(strcmp(raw.talker, "GN") == 0);
    CHECK(raw.gga.has_diff_age == 0);

    /* No fix: empty coordinates must not become a position (or a southern one) */
    nmea_line(line, "GPGGA,123519.00,,,,,0,00,,,,,,,");
    CHECK(feed_nmea(&raw, line) == 1);
    CHECK(raw.gga.has_position == 0 && raw.gga.quality == 0);
    CHECK(raw.gga.lat == 0.0 && raw.gga.lon == 0.0);
    CHECK(hipnuc_sample_from_nmea(&raw, &s) == 1);
    CHECK((s.valid & HIPNUC_VALID_POSITION) == 0);

    /* RMC */
    nmea_line(line, "GPRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,230394,,,D");
    CHECK(feed_nmea(&raw, line) == 1);
    CHECK(raw.msg_type == NMEA_MSG_RMC);
    CHECK(raw.rmc.status == 'A' && raw.rmc.mode == 'D');
    CHECK(raw.rmc.year == 1994 && raw.rmc.month == 3 && raw.rmc.day == 23);
    CHECK(near(raw.rmc.sog, 22.4, 1e-5) && near(raw.rmc.cog, 84.4, 1e-5));
    CHECK(hipnuc_sample_from_nmea(&raw, &s) == 1);
    CHECK((s.valid & HIPNUC_VALID_UTC) && s.utc.year == 1994 && s.utc.hour == 12);
    CHECK((s.valid & HIPNUC_VALID_SOG_COG) && near(s.sog, 11.5235556, 1e-5));   /* 22.4 kn */
    CHECK(near(s.cog, 1.47305, 1e-5));

    /* RMC without mode field and void status */
    nmea_line(line, "GPRMC,123519.00,V,,,,,,,230394,,");
    CHECK(feed_nmea(&raw, line) == 1);
    CHECK(raw.rmc.status == 'V' && raw.rmc.mode == 'N' && raw.rmc.has_position == 0);

    /* Checksum error */
    strcpy(line, "$GPGGA,123519.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00\r\n");
    CHECK(feed_nmea(&raw, line) == -1);
    CHECK(raw.checksum_error_count == 1);

    /* Unsupported sentence with a valid checksum -> 0, decoder stays usable */
    nmea_line(line, "GPVTG,90.0,T,80.0,M,10.0,N,18.5,K,A");
    CHECK(feed_nmea(&raw, line) == 0);
    nmea_line(line, "GPGGA,123519.50,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    CHECK(feed_nmea(&raw, line) == 1);

    /* Overlong garbage is abandoned without overflow */
    {
        int i, r = 0;
        r = nmea_input(&raw, '$');
        for (i = 0; i < 300 && r == 0; ++i) r = nmea_input(&raw, 'A');
        CHECK(r == -1);
        CHECK(feed_nmea(&raw, line) == 1);
    }

    /* A '$' inside a sentence restarts it */
    CHECK(feed_nmea(&raw, "$GPGGA,12") == 0);
    CHECK(nmea_input(&raw, '$') == -1);
    CHECK(feed_nmea(&raw, line + 1) == 1);
}

/* ---- JSON ---------------------------------------------------------------- */

static void test_json(void)
{
    uint8_t f[128];
    hipnuc_raw_t raw;
    hipnuc_sample_t s;
    char json[1024];
    int n, want;
    size_t nf = build_hi91(f);

    memset(&raw, 0, sizeof(raw));
    feed(&raw, f, nf);
    hipnuc_sample_from_raw(&raw, &s);

    n = hipnuc_json_sample(&s, json, sizeof(json));
    CHECK(n > 0 && (size_t)n == strlen(json));
    CHECK(json[0] == '{' && json[n - 1] == '}');
    CHECK(strstr(json, "\"type\":\"HI91\"") != NULL);
    CHECK(strstr(json, "\"acceleration_m_s2\":[9.8,-4.9,2.45]") != NULL);
    CHECK(strstr(json, "\"status_flags\":[\"WB_CONV\",\"UTC_UNSYNC\"]") != NULL);
    CHECK(strstr(json, "\"device_time_us\":123456000") != NULL);
    CHECK(strstr(json, "\"temperature_c\":-7") != NULL);
    CHECK(strstr(json, "\"utc\"") == NULL);
    CHECK(strstr(json, "\"longitude_deg\"") == NULL);

    /* Sizing query matches the real output */
    want = hipnuc_json_sample(&s, NULL, 0);
    CHECK(want == n);

    /* Undersized buffers fail cleanly and never leave partial JSON */
    CHECK(hipnuc_json_sample(&s, json, (size_t)n) == -1);
    CHECK(json[0] == '\0');
    CHECK(hipnuc_json_sample(&s, json, (size_t)n + 1) == n);

    /* Non-finite values are refused */
    s.acc[0] = (float)(1e300 * 1e300);
    CHECK(hipnuc_json_sample(&s, json, sizeof(json)) == -1);
    CHECK(json[0] == '\0');

    /* Empty sample */
    hipnuc_sample_clear(&s);
    n = hipnuc_json_sample(&s, json, sizeof(json));
    CHECK(n > 0 && strcmp(json, "{\"type\":\"NONE\"}") == 0);
}

int main(void)
{
    test_hi91();
    test_hi81();
    test_hi83();
    test_framing();
    test_nmea();
    test_json();
    if (failures) {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("test_decoder: all checks passed\n");
    return 0;
}
