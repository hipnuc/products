#include "oem_decode.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

#define OEM4SYNC1 (0xAA)
#define OEM4SYNC2 (0x44)
#define OEM4SYNC3 (0x12)
#define OEM4SYNC3_UNICORE (0xB5)
#define OEM4HLEN (28) /* oem7/6/4 message header length (bytes) */
#define UNICOREHLEN (24)

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

static uint32_t _crc32(int i)
{
    int j;
    uint32_t ulCRC;
    ulCRC = i;
    for (j = 8; j > 0; j--)
    {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

uint32_t oem7_crc32(uint8_t *buf, uint32_t len)
{
    uint32_t ulTemp1;
    uint32_t ulTemp2;
    uint32_t ulCRC = 0;
    while (len-- != 0)
    {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
        ulTemp2 = _crc32(((int)ulCRC ^ *buf++) & 0xFF);
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return (ulCRC);
}

/* sync code */
static int sync_oem4(uint8_t *buff, uint8_t data)
{
    buff[0] = buff[1];
    buff[1] = buff[2];
    buff[2] = data;
    return buff[0] == OEM4SYNC1 && buff[1] == OEM4SYNC2 && (buff[2] == OEM4SYNC3 || buff[2] == OEM4SYNC3_UNICORE);
}

static int decode_oem4(oem_raw_t *raw)
{
    int type = U2(raw->buf + 4);
    int ret = 1;
    raw->type = type;
    if (oem7_crc32(raw->buf, raw->len) != U4(raw->buf + raw->len))
    {
        printf("oem4 crc error: type=%3d len=%d\n", type, raw->len);
        return -1;
    }
    switch (type)
    {
    case OEM_ID_AGRIC:
        memcpy(&raw->agric, raw->buf, sizeof(oem_agric_t));
        break;
    case OEM_ID_BESTXYZ:
        memcpy(&raw->bestxyz, raw->buf, sizeof(oem_bestxyz_t));
        break;
    case OEM_ID_BESTPOS:
        memcpy(&raw->bestpos, raw->buf, sizeof(oem_bestpos_t));
        break;
    case OEM_ID_HEADING:
        memcpy(&raw->heading, raw->buf, sizeof(oem_heading_t));
        break;
    case OEM_ID_ANTENNA:
        // return decode_antenna(raw);
        break;
    case OEM_ID_INSPVAX:
        memcpy(&raw->inspvax, raw->buf, sizeof(oem_inspvax_t));
        break;
    case OEM_ID_RAWIMUX:
        memcpy(&raw->rawimux, raw->buf, sizeof(oem_rawimux_t));
        break;
    case OEM_ID_PVTSLN:
        memcpy(&raw->pvtsln, raw->buf, sizeof(oem_pvtsln_t));
        break;
    case OEM_ID_GNSSRCV:
        memcpy(&raw->gnssrcv, raw->buf, sizeof(oem_gnssrcv_t));
        break;
    default:
        ret = 0;
    }
    return ret;
}

int input_oem(oem_raw_t *raw, uint8_t ch)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (sync_oem4(raw->buf, ch))
            raw->nbyte = 3;
        return 0;
    }

    raw->buf[raw->nbyte++] = ch;

    if (raw->buf[2] == OEM4SYNC3 && raw->nbyte == 10 && (raw->len = U2(raw->buf + 8) + OEM4HLEN) > MAXRAWLEN - 4)
    {
        raw->nbyte = 0;
        return -1;
    }

    if (raw->buf[2] == OEM4SYNC3_UNICORE && raw->nbyte == 8 && (raw->len = U2(raw->buf + 6) + UNICOREHLEN) > MAXRAWLEN - 4)
    {
        raw->nbyte = 0;
        return -1;
    }

    if (raw->nbyte < 10 || raw->nbyte < raw->len + 4)
        return 0;
    raw->nbyte = 0;

    /* decode oem7/6/4 message */
    return decode_oem4(raw);
}
