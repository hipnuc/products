#include "ch_serial.h"

#if defined(CH_DEBUG)
#include <stdio.h>
#define CH_TRACE printf
#else
#define CH_TRACE(...)
#endif

#define CHSYNC1 (0x5A)     /* CHAOHE message sync code 1 */
#define CHSYNC2 (0xA5)     /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE (0x06) /* CHAOHE protocol header size */

/* common type conversion */
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}
static uint32_t U4(uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}
static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

int ch_imu_data2str(hipnuc_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "temperature", raw->imu.temp);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "acc(G):", raw->imu.acc[0], raw->imu.acc[1], raw->imu.acc[2]);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "gyr(deg/s):", raw->imu.gyr[0], raw->imu.gyr[1], raw->imu.gyr[2]);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "mag(uT):", raw->imu.mag[0], raw->imu.mag[1], raw->imu.mag[2]);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "eul(deg):", raw->imu.eul[0], raw->imu.eul[1], raw->imu.eul[2]);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f %.3f\r\n", "quat:", raw->imu.quat[0], raw->imu.quat[1], raw->imu.quat[2], raw->imu.quat[3]);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%.3f\r\n", "presure(pa):", raw->imu.prs);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "timestamp(ms):", raw->imu.ts);
    if (ret > 0) written += ret;

    ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "pps_sync_time(ms):", raw->imu.pps_sync_ms);
    if (ret > 0) written += ret;
    
    return written;
}


/* parse the payload of a frame and feed into data section */
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0, i = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case kItemID:
            ofs += 2;
            break;
        case kItemAccRaw:
            raw->imu.acc[0] = (float)I2(p + ofs + 1) / 1000;
            raw->imu.acc[1] = (float)I2(p + ofs + 3) / 1000;
            raw->imu.acc[2] = (float)I2(p + ofs + 5) / 1000;
            ofs += 7;
            break;
        case kItemGyrRaw:
            raw->imu.gyr[0] = (float)I2(p + ofs + 1) / 10;
            raw->imu.gyr[1] = (float)I2(p + ofs + 3) / 10;
            raw->imu.gyr[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case kItemMagRaw:
            raw->imu.mag[0] = (float)I2(p + ofs + 1) / 10;
            raw->imu.mag[1] = (float)I2(p + ofs + 3) / 10;
            raw->imu.mag[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case kItemRotationEul:
            raw->imu.eul[0] = (float)I2(p + ofs + 1) / 100;
            raw->imu.eul[1] = (float)I2(p + ofs + 3) / 100;
            raw->imu.eul[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case kItemRotationQuat:
            raw->imu.quat[0] = R4(p + ofs + 1);
            raw->imu.quat[1] = R4(p + ofs + 5);
            raw->imu.quat[2] = R4(p + ofs + 9);
            raw->imu.quat[3] = R4(p + ofs + 13);
            ofs += 17;
            break;
        case kItemPressure:
            raw->imu.prs = R4(p + ofs + 1);
            ofs += 5;
            break;

        case KItemIMUSOL:
            raw->imu.pps_sync_ms = U2(p + ofs + 1);
            raw->imu.temp = U1(p + ofs + 3);
            raw->imu.prs = R4(p + ofs + 4);
            raw->imu.ts = U4(p + ofs + 8);
            raw->imu.acc[0] = R4(p + ofs + 12);
            raw->imu.acc[1] = R4(p + ofs + 16);
            raw->imu.acc[2] = R4(p + ofs + 20);
            raw->imu.gyr[0] = R4(p + ofs + 24);
            raw->imu.gyr[1] = R4(p + ofs + 28);
            raw->imu.gyr[2] = R4(p + ofs + 32);
            raw->imu.mag[0] = R4(p + ofs + 36);
            raw->imu.mag[1] = R4(p + ofs + 40);
            raw->imu.mag[2] = R4(p + ofs + 44);
            raw->imu.eul[0] = R4(p + ofs + 48);
            raw->imu.eul[1] = R4(p + ofs + 52);
            raw->imu.eul[2] = R4(p + ofs + 56);
            raw->imu.quat[0] = R4(p + ofs + 60);
            raw->imu.quat[1] = R4(p + ofs + 64);
            raw->imu.quat[2] = R4(p + ofs + 68);
            raw->imu.quat[3] = R4(p + ofs + 72);
            ofs += 76;
            break;
        default:
            ofs++;
            break;
        }
    }

    return 1;
}

static int decode_ch(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;

    /* checksum */
    crc16_update(&crc, raw->buf, 4);
    crc16_update(&crc, raw->buf + 6, raw->len);
    if (crc != U2(raw->buf + 4))
    {
        CH_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf + 4), crc, raw->len);
        return -1;
    }

    return parse_data(raw);
}

/* sync code */
static int sync_ch(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
 * @brief This function read serial data and decode
 *
 * @param raw the decode struct
 *
 * @param data stream data
 *
 * @return 1: sucessfully decode one frame. else: decode not complete or fail
 */
int ch_serial_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_ch(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (MAXRAWLEN - CH_HDR_SIZE))
        {
            CH_TRACE("ch length error: len=%d\n", raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;
    return decode_ch(raw);
}
