#ifndef __HIPNUC_H__
#define __HIPNUC_H__

#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>


#ifdef QT_CORE_LIB
#pragma pack(push)
#pragma pack(1)
#endif

#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)          /* CHAOHE protocol header size */
#define HIPNUC_MAX_RAW_SIZE     (128+CH_HDR_SIZE)


/**
* packet 0x91: IMU data(floating point)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag; /* data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        pps_sync_ms;
    int8_t          temp;
    float           prs;
    uint32_t        ts; /* timestamp */
    float           acc[3];
    float           gyr[3];
    float           mag[3];
    float           roll;
    float           pitch;
    float           yaw;
    float           quat[4];
} hi91_t;

/**
* packet 0x92: IMU data(int type)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;
    uint16_t        status;
    int8_t          temperature;
    uint16_t        sync_time;
    int16_t         air_pressure;
    int16_t         reserved;
    int16_t         gyr_b[3];
    int16_t         acc_b[3];
    int16_t         mag_b[3];
    int32_t         roll;
    int32_t         pitch;
    int32_t         yaw;
    int16_t        quat[4];
} hi92_t;

/**
* packet 0x81: INS data, include lat, lon, eul, quat, raw IMU data 
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;
    uint16_t        status;
    uint8_t         ins_status;
    uint16_t        gpst_wn;
    uint32_t        gpst_tow;
    uint16_t        sync_time;
    int16_t         gyr_b[3];
    int16_t         acc_b[3];
    int16_t         mag_b[3];
    int16_t         air_pressure;
    int16_t         reserved1;
    int8_t          temperature;
    uint8_t         utc_year;
    uint8_t         utc_mouth;
    uint8_t         utc_day;
    uint8_t         utc_hour;
    uint8_t         utc_min;
    uint16_t        utc_msec;
    int16_t         roll;
    int16_t         pitch;
    uint16_t        yaw;
    int16_t         quat[4];
    int32_t         ins_lon;
    int32_t         ins_lat;
    int32_t         ins_msl;
    uint8_t         pdop;
    uint8_t         hdop;
    uint8_t         solq_pos;
    uint8_t         nv_pos;
    uint8_t         solq_heading;
    uint8_t         nv_heading;
    uint8_t         diff_age;
    int16_t         undulation;
    uint8_t         reserved;
    int16_t         vel_enu[3];
    int16_t         acc_enu[3];
    int32_t         gnss_lon;
    int32_t         gnss_lat;
    int32_t         gnss_msl;
    uint8_t         reserved2[2];
} hi81_t;



typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    int len;                            /* message length (bytes) */
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* message raw buffer */
    hi91_t hi91;
    hi92_t hi92;
    hi81_t hi81;
}hipnuc_raw_t;

#ifdef QT_CORE_LIB
#pragma pack(pop)
#endif

int hipnuc_input(hipnuc_raw_t *raw, uint8_t data);
int hipnuc_dump_packet(hipnuc_raw_t *raw, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif


#endif


