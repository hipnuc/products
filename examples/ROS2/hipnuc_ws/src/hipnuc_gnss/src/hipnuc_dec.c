#include "hipnuc.h"


/* legcy support of HI226/HI229 */
#define HIPNUC_ID_USRID         (0x90)
#define HIPNUC_ID_ACC_RAW       (0xA0)
#define HIPNUC_ID_ACC_CAL       (0xA1)
#define HIPNUC_ID_GYR_RAW       (0xB0)
#define HIPNUC_ID_GYR_CAL       (0xB1)
#define HIPNUC_ID_MAG_RAW       (0xC0)
#define HIPNUC_ID_EUL           (0xD0)
#define HIPNUC_ID_QUAT          (0xD1)
#define HIPNUC_ID_PRS           (0xF0)

/* new HiPNUC standard packet */
#define HIPNUC_ID_IMUSOL        (0x91)
#define HIPNUC_ID_IMUBIN        (0x92)
#define HIPNUC_ID_INSSOL        (0x81)

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif


static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);

/* common type conversion */
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

/* parse the payload of a frame and feed into data section */
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    
    /* ignore all previous data */
    raw->hi91.tag = 0;
    raw->hi81.tag = 0;
    raw->hi92.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_ID_USRID:
            ofs += 2;
            break;
        case HIPNUC_ID_ACC_RAW:
        case HIPNUC_ID_ACC_CAL:
             raw->hi91.tag = HIPNUC_ID_IMUSOL;
             raw->hi91.acc[0] = (float)I2(p + ofs + 1) / 1000;
             raw->hi91.acc[1] = (float)I2(p + ofs + 3) / 1000;
             raw->hi91.acc[2] = (float)I2(p + ofs + 5) / 1000;
            ofs += 7;
            break;
        case HIPNUC_ID_GYR_RAW:
        case HIPNUC_ID_GYR_CAL:
            raw->hi91.tag = HIPNUC_ID_IMUSOL;
            raw->hi91.gyr[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.gyr[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.gyr[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_MAG_RAW:
            raw->hi91.tag = HIPNUC_ID_IMUSOL;
            raw->hi91.mag[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.mag[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.mag[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_EUL:
            raw->hi91.tag = HIPNUC_ID_IMUSOL;
            raw->hi91.pitch = (float)I2(p + ofs + 1) / 100;
            raw->hi91.roll = (float)I2(p + ofs + 3) / 100;
            raw->hi91.yaw = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_QUAT:
            raw->hi91.tag = HIPNUC_ID_IMUSOL;
            raw->hi91.quat[0] = R4(p + ofs + 1);
            raw->hi91.quat[1] = R4(p + ofs + 5);
            raw->hi91.quat[2] = R4(p + ofs + 9);
            raw->hi91.quat[3] = R4(p + ofs + 13);
            ofs += 17;
            break;
        case HIPNUC_ID_PRS:
            raw->hi91.tag = HIPNUC_ID_IMUSOL;
            raw->hi91.prs = R4(p + ofs + 1);
            ofs += 5;
            break;
        case HIPNUC_ID_IMUSOL:
            memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;
        case HIPNUC_ID_INSSOL:
            memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
            ofs += sizeof(hi81_t);
            break;
        case HIPNUC_ID_IMUBIN:
            memcpy(&raw->hi92, p + ofs, sizeof(hi92_t));
            ofs += sizeof(hi92_t);
            break;
        default:
            ofs++;
            break;
        }
    }
    return 1;
}

static int decode_hipnuc(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;

    /* checksum */
    hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
    hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
    {
        // NL_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf + 4), crc, raw->len);
        return -1;
    }

    return parse_data(raw);
}

/* sync code */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
* @brief     hipnuc decoder input, read one byte at one time.
 *
 * @param    raw is the decoder struct.
 * @param    data is the one byte read from stram.
 * @param    buf is the log string buffer, ireturn > 0: decoder received a frame successfully, else: receiver not receive a frame successfully.
 *
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_hipnuc(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            // NL_TRACE("ch length error: len=%d\n",raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;

    return decode_hipnuc(raw);
}


/**
 * @brief    convert packet to string, only dump parts of data
 *
 * @param    raw is struct of decoder
 * @param    buf is the log string buffer, make sure buf larger than 256
 * @param    buf_size is the szie of the log buffer
 *
 */
int hipnuc_dump_packet(hipnuc_raw_t *raw, char *buf, size_t buf_size)
{
    int written = 0;
    int ret;

    /* dump 0x91 packet */
    if(raw->hi91.tag == HIPNUC_ID_IMUSOL)
    {
        ret = snprintf(buf + written, buf_size - written, "%-16s0x%X\r\n", "tag:", raw->hi91.tag);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "sync_time(ms):", raw->hi91.pps_sync_ms);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "acc(m/s^(2)):", raw->hi91.acc[0]*GRAVITY, raw->hi91.acc[1]*GRAVITY, raw->hi91.acc[2]*GRAVITY);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "gyr(deg/s):", raw->hi91.gyr[0], raw->hi91.gyr[1], raw->hi91.gyr[2]);
        if (ret > 0) written += ret;

        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "mag(uT):", raw->hi91.mag[0], raw->hi91.mag[1], raw->hi91.mag[2]);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "Roll/Pitch/Yaw(deg):", raw->hi91.roll, raw->hi91.pitch, raw->hi91.yaw);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "timestamp(ms):", raw->hi91.ts);
        if (ret > 0) written += ret;
    }
    
    /* dump 0x92 packet */
    if(raw->hi92.tag == HIPNUC_ID_IMUBIN)
    {
        ret = snprintf(buf + written, buf_size - written, "%-16s0x%X\r\n", "tag:", raw->hi92.tag);
        if (ret > 0) written += ret;
    
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "temperature", raw->hi92.temperature);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "sync_time(ms):", raw->hi92.sync_time);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "acc(m/s^(2)):", raw->hi92.acc_b[0]*0.0048828, raw->hi92.acc_b[1]*0.0048828, raw->hi92.acc_b[2]*0.0048828);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "gyr(deg/s):", raw->hi92.gyr_b[0]*(0.001*R2D), raw->hi92.gyr_b[1]*(0.001*R2D), raw->hi92.gyr_b[2]*(0.001*R2D));
        if (ret > 0) written += ret;

        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "mag(uT):", raw->hi92.mag_b[0]*0.030517, raw->hi92.mag_b[1]*0.030517, raw->hi92.mag_b[2]*0.030517);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "Roll/Pitch/Yaw(deg):", raw->hi92.roll*0.001, raw->hi92.pitch*0.001, raw->hi92.yaw*0.001);
        if (ret > 0) written += ret;
    }

    /* dump 0x81 packet */
    if(raw->hi81.tag == HIPNUC_ID_INSSOL)
    {
        ret = snprintf(buf + written, buf_size - written, "%-16s0x%X\r\n", "tag:", raw->hi81.tag);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "solq_pos:", raw->hi81.solq_pos);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%d\r\n", "sat number:", raw->hi81.nv_pos);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.7f %.7f\r\n", "Lat/Lon(deg):", raw->hi81.ins_lat*1e-7, raw->hi81.ins_lon*1e-7);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f\r\n", "height(m):", raw->hi81.ins_msl*1e-3);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "acc(m/s^(2)):", raw->hi81.acc_b[0]*0.0048828, raw->hi81.acc_b[1]*0.0048828, raw->hi81.acc_b[2]*0.0048828);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "gyr(deg/s):", raw->hi81.gyr_b[0]*(0.001*R2D), raw->hi81.gyr_b[1]*(0.001*R2D), raw->hi81.gyr_b[2]*(0.001*R2D));
        if (ret > 0) written += ret;

        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "mag(uT):", raw->hi81.mag_b[0]*0.030517, raw->hi81.mag_b[1]*0.030517, raw->hi81.mag_b[2]*0.030517);
        if (ret > 0) written += ret;
        
        ret = snprintf(buf + written, buf_size - written, "%-16s%.3f %.3f %.3f\r\n", "Roll/Pitch/Yaw(deg):", raw->hi81.roll*0.01, raw->hi81.pitch*0.01, raw->hi81.yaw*0.01);
        if (ret > 0) written += ret;
    }
    
    return written;
}

/**
 * @brief    calcuate hipnuc_crc16
 *
 * @param    inital is intial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 *
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
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
    *inital = crc;
}
