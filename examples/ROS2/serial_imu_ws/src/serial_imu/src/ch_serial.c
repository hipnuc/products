#include "ch_serial.h"

#if defined(CH_DEBUG)
#include <stdio.h>
#define CH_TRACE	printf
#else
#define CH_TRACE(...)
#endif


#define CHSYNC1         (0x5A)        /* CHAOHE message sync code 1 */
#define CHSYNC2         (0xA5)        /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE     (0x06)        /* CHAOHE protocol header size */

/* common type conversion */
#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
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

/* dump hi229 or hi226 data packet*/
void ch_dump_imu_data(raw_t *raw)
{
    int i;
    if(raw->item_code[0] != KItemGWSOL)
    {
        CH_TRACE("%-16s%d\r\n",       "id:",  raw->imu[0].id);
        CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "acc(G):",        raw->imu[0].acc[0], raw->imu[0].acc[1],  raw->imu[0].acc[2]);
        CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "gyr(deg/s):",    raw->imu[0].gyr[0], raw->imu[0].gyr[1],  raw->imu[0].gyr[2]);
        CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "mag(uT):",       raw->imu[0].mag[0], raw->imu[0].mag[1],  raw->imu[0].mag[2]);
        CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "eul(deg):",      raw->imu[0].eul[0], raw->imu[0].eul[1],  raw->imu[0].eul[2]);
        CH_TRACE("%-16s%.3f %.3f %.3f %.3f\r\n",  "quat:",          raw->imu[0].quat[0], raw->imu[0].quat[1],  raw->imu[0].quat[2], raw->imu[0].quat[3]);
        CH_TRACE("%-16s%.3f\r\n",       "presure(pa):",  raw->imu[0].pressure);
        CH_TRACE("%-16s%d\r\n",       "timestamp(ms):",  raw->imu[0].timestamp);
        
        CH_TRACE("item: ");
        for(i=0; i<raw->nitem_code; i++)
        {
            CH_TRACE("0x%02X ", raw->item_code[i]);
        }
        CH_TRACE("\r\n");
    }
    else
    {
        CH_TRACE("gateway: %s%d, %s%d\r\n",       "gwid:",      raw->gwid, "node cnt:", raw->nimu);
        for(i=0; i<raw->nimu; i++)
        {
            CH_TRACE("%-16s%d\r\n",       "id:",  raw->imu[i].id);
            CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "acc(G):",        raw->imu[i].acc[0], raw->imu[i].acc[1],  raw->imu[i].acc[2]);
            CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "gyr(deg/s):",    raw->imu[i].gyr[0], raw->imu[i].gyr[1],  raw->imu[i].gyr[2]);
            CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "mag(uT):",       raw->imu[i].mag[0], raw->imu[i].mag[1],  raw->imu[i].mag[2]);
            CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "eul(deg):",      raw->imu[i].eul[0], raw->imu[i].eul[1],  raw->imu[i].eul[2]);
            CH_TRACE("%-16s%.3f %.3f %.3f %.3f\r\n",  "quat:",          raw->imu[i].quat[0], raw->imu[i].quat[1],  raw->imu[i].quat[2], raw->imu[i].quat[3]);
            CH_TRACE("%-16s%.3f\r\n",       "presure(pa):",  raw->imu[i].pressure);
            CH_TRACE("%-16s%d\r\n",       "timestamp(ms):",  raw->imu[i].timestamp);
        }
    }
}



/* parse the payload of a frame and feed into data section */
static int parse_data(raw_t *raw)
{
    int ofs = 0, i = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    memset(raw->item_code, 0, sizeof(raw->item_code));
    raw->nitem_code = 0;

	while(ofs < raw->len)
	{
		switch(p[ofs])
		{
            case kItemID:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemID;
                raw->imu[0].id = U1(p+ofs+1);
                ofs += 2;
                break;
            case kItemAccRaw:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemAccRaw;
                raw->imu[0].acc[0] = (float)I2(p+ofs+1) / 1000;
                raw->imu[0].acc[1] = (float)I2(p+ofs+3) / 1000;
                raw->imu[0].acc[2] = (float)I2(p+ofs+5) / 1000;
                ofs += 7;
                break;
            case kItemGyrRaw:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemGyrRaw;
                raw->imu[0].gyr[0] = (float)I2(p+ofs+1) / 10;
                raw->imu[0].gyr[1] = (float)I2(p+ofs+3) / 10;
                raw->imu[0].gyr[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemMagRaw:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemMagRaw;
                raw->imu[0].mag[0] = (float)I2(p+ofs+1) / 10;
                raw->imu[0].mag[1] = (float)I2(p+ofs+3) / 10;
                raw->imu[0].mag[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemRotationEul:
                raw->item_code[raw->nitem_code++] = kItemRotationEul;
                raw->imu[0].eul[0] = (float)I2(p+ofs+1) / 100;
                raw->imu[0].eul[1] = (float)I2(p+ofs+3) / 100;
                raw->imu[0].eul[2] = (float)I2(p+ofs+5) / 10;
                ofs += 7;
                break;
            case kItemRotationQuat:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemRotationQuat;
                raw->imu[0].quat[0] = R4(p+ofs+1);
                raw->imu[0].quat[1] = R4(p+ofs+5);
                raw->imu[0].quat[2] = R4(p+ofs+9);
                raw->imu[0].quat[3] = R4(p+ofs+13);
                ofs += 17;
                break;
            case kItemPressure:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = kItemPressure;
                raw->imu[0].pressure = R4(p+ofs+1);
                ofs += 5;
                break;

            case KItemIMUSOL:
                raw->nimu = 1;
                raw->item_code[raw->nitem_code++] = KItemIMUSOL;
                raw->imu[0].id = U1(p+ofs+1);
                raw->imu[0].pressure = R4(p+ofs+4);
                raw->imu[0].timestamp = U4(p+ofs+8);
                raw->imu[0].acc[0] = R4(p+ofs+12);
                raw->imu[0].acc[1] = R4(p+ofs+16);
                raw->imu[0].acc[2] = R4(p+ofs+20);
                raw->imu[0].gyr[0] = R4(p+ofs+24);
                raw->imu[0].gyr[1] = R4(p+ofs+28);
                raw->imu[0].gyr[2] = R4(p+ofs+32);
                raw->imu[0].mag[0] = R4(p+ofs+36);
                raw->imu[0].mag[1] = R4(p+ofs+40);
                raw->imu[0].mag[2] = R4(p+ofs+44);
                raw->imu[0].eul[0] = R4(p+ofs+48);
                raw->imu[0].eul[1] = R4(p+ofs+52);
                raw->imu[0].eul[2] = R4(p+ofs+56);
                raw->imu[0].quat[0] = R4(p+ofs+60);
                raw->imu[0].quat[1] = R4(p+ofs+64);
                raw->imu[0].quat[2] = R4(p+ofs+68);
                raw->imu[0].quat[3] = R4(p+ofs+72);
                ofs += 76;
                break;
				
            case KItemGWSOL:
                raw->item_code[raw->nitem_code++] = KItemGWSOL;
                raw->gwid = U1(p+ofs+1);
                raw->nimu = U1(p+ofs+2);
                ofs += 8;
                for (i=0; i<raw->nimu; i++)
                {
                    raw->imu[i].id = U1(p+ofs+1);
                    raw->imu[i].pressure = R4(p+ofs+4);
                    raw->imu[i].timestamp = U4(p+ofs+8);
                    raw->imu[i].acc[0] = R4(p+ofs+12);
                    raw->imu[i].acc[1] = R4(p+ofs+16);
                    raw->imu[i].acc[2] = R4(p+ofs+20);
                    raw->imu[i].gyr[0] = R4(p+ofs+24);
                    raw->imu[i].gyr[1] = R4(p+ofs+28);
                    raw->imu[i].gyr[2] = R4(p+ofs+32);
                    raw->imu[i].mag[0] = R4(p+ofs+36);
                    raw->imu[i].mag[1] = R4(p+ofs+40);
                    raw->imu[i].mag[2] = R4(p+ofs+44);
                    raw->imu[i].eul[0] = R4(p+ofs+48);
                    raw->imu[i].eul[1] = R4(p+ofs+52);
                    raw->imu[i].eul[2] = R4(p+ofs+56);
                    raw->imu[i].quat[0] = R4(p+ofs+60);
                    raw->imu[i].quat[1] = R4(p+ofs+64);
                    raw->imu[i].quat[2] = R4(p+ofs+68);
                    raw->imu[i].quat[3] = R4(p+ofs+72);
                    ofs += 76;
                }
                break;
            default:
				ofs++;
                break;
		}
    }
    
    return 1;
}

static int decode_ch(raw_t *raw)
{
    uint16_t crc = 0;   

    /* checksum */
    crc16_update(&crc, raw->buf, 4);
    crc16_update(&crc, raw->buf+6, raw->len);
    if (crc != U2(raw->buf+4))
    {
        CH_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf+4), crc, raw->len);
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

int ch_serial_input(raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_ch(raw->buf, data)) return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;
    
    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf+2)) > (MAXRAWLEN - CH_HDR_SIZE))
        {
            CH_TRACE("ch length error: len=%d\n",raw->len);
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
