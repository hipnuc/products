#ifndef __OEM_DECODE_H__
#define __OEM_DECODE_H__

#include <stdio.h>
#include "stdint.h"
#include <string.h>

#define OEM_ID_INSPVAX         (1465)
#define OEM_ID_RAWIMUX         (1461)
#define OEM_ID_BESTPOS         (42)
#define OEM_ID_ANTENNA         (51)
#define OEM_ID_BESTXYZ         (241)
#define OEM_ID_HEADING         (971)
#define OEM_ID_PVTSLN          (1021)
#define OEM_ID_VERSION         (37)
#define OEM_ID_GNSSRCV         (1590) 
#define OEM_ID_MCUSTAT         (1592) /* HiPNUC: MCU ESKF status info */
#define OEM_ID_AGRIC           (11276)

#define MAXRAWLEN   1800               /* max length of receiver raw message */

typedef struct __attribute__((__packed__))
{
    uint32_t preamble; /* 0x1C1244AA: AA 44 12 1C(header len(0x1C=28)) */
    uint16_t msg_id;
    uint16_t msg_len;
    uint8_t  time_ref;
    uint8_t  time_stat;
    uint16_t gps_wn;
    uint32_t gps_tow_ms;
    uint32_t rev;
    uint8_t  version;
    uint8_t  leap_sec;
    uint16_t delay_ms;
} unicore_hdr_t;

typedef struct __attribute__((__packed__))
{
    uint32_t preamble; /* 0x1C1244AA: AA 44 12 1C(header len(0x1C=28)) */
    uint16_t msg_id;
    uint8_t msg_type; /* 00 =二进制 01 = ASCII 10 = 简化ASCII */
    uint8_t rev1;
    uint16_t msg_len;
    uint16_t sequence;
    uint8_t idle_time;
    uint8_t time_quality; /* TIME_QUALITY_UNKNOWN or TIME_QUALITY_FINE */
    uint16_t gps_wn;
    uint32_t gps_tow_ms;
    uint32_t evt_bit;
    uint8_t rev3[4];
} oem_hdr_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t hdr;
    uint32_t ins_stat;
    uint32_t pos_type;  /* see SPAN stardard protocol(OEM7_Commands_Logs_Manual.pdf) or solq2span_pos_type */

    double lla[3];      /* lat, lon, msl */
    float undulation;
    double vel_neu[3];
    double roll;
    double pitch;
    double yaw;

    float lla_std[3];
    float vel_neu_std[3];
    float roll_std;
    float pitch_std;
    float yaw_std;
    uint32_t ext_sol_stat;
    uint16_t time_since_update;
    uint32_t crc;
} oem_inspvax_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t hdr;
    uint8_t nv;              /* # of sat in antA */
    uint8_t nv_heading;      /* # of sat in antB */
    uint8_t solq;            /* aligned with NL, see SOLQ_NONE */
    uint8_t solq_heading;    /* aligned with NL, see SOLQ_NONE */
    float   diff_age;        /* diff age */

    double lla[3];          /* aligned with INSPVAX */
    float undulation;
    double vel_neu[3];
    double bl_yaw;            /* dual bl heading, deg, 0-360 */
    double bl_pitch;          /* dual bl pitch, deg, 0-360 */
    double bl_len;            /* dual bl len, m */
    float  bl_pitch_std;
    float  bl_yaw_std;

    float lla_std[3];
    float vel_neu_std[3];
    float wb[3];            /* gyr bais estimated by MCU's KF, RAD */
    float gb[3];            /* acc bais estimated by MCU's KF, M/S^(2) */
    float od_speed;         /* CAN od speed */
    uint16_t sv_info[64];
    uint16_t time_since_update;
    uint32_t crc;
} oem_gnssrcv_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t hdr;
    uint8_t imu_info;
    uint8_t imu_type;
    uint16_t gps_wn;
    double gps_tow;
    uint32_t imu_stat;
    int32_t acc_znyx[3]; /* m/s^(2) * 100000 * dt */
    int32_t gyr_znyx[3]; /* rad/s * 100000 * dt */

    uint32_t crc;
} oem_rawimux_t;

typedef struct __attribute__((__packed__)) /* UM982 AGIRC msg */
{
    unicore_hdr_t  hdr;
    uint8_t         gnss_str[4];  /* 固定为 GNSS */
    uint8_t         len; /* 232 */
    uint8_t         year; /*2016年，为 16 */
    uint8_t         month;
    uint8_t         day;
    uint8_t         hour;
    uint8_t         minute;
    uint8_t         second;
    uint8_t         rtk_stat; /* 0：无效解  1：单点定位解  2：伪距差分  4：固定解  5：浮动解 */
    uint8_t         heading_stat; /* 0：无效解 4：固定解 5：浮动解 */
    uint8_t         nv_gps;
    uint8_t         nv_bd;
    uint8_t         nv_glo;
    float           bl_neu[3];        /* 到基站RTK的 基线 */
    float           bl_neu_std[3];    /* 到基站RTK的 基线 std */
    float           heading;          /* 0-360 */
    float           pitch;
    float           roll;
    float           speed;      /* spped 标量 */
    float           vel_neu[3];
    float           vel_neu_std[3];
    double          lla[3];
    double          XYZ[3];
    float           lla_std[3]; /* meaning pos_neu_std */
    float           XYZ_std[3];
    double          base_lla[3];
    double          sec_lla[3];
    uint32_t        tow;
    float           diff_age;
    float           speed_heading;
    float           undulation;
    float           rev[2];
    uint8_t         nv_gal;
    uint8_t         rev2[3];
    uint32_t        crc;
} oem_agric_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t  hdr;
    uint32_t    sol_stat;
    uint32_t    pos_type;
    float       length;
    float       heading;
    float       pitch;
    float       reserved;
    float       hdgsddev;
    float       ptchstddev;
    char        stn_id[4];
    uint8_t     SVs;
    uint8_t     solnSVs;
    uint8_t     obs;
    uint8_t     multi;
    uint8_t     reserved2;
    uint8_t     ext_sol_stat;
    uint8_t     reserved3[2];
    uint32_t    crc;
} oem_heading_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t hdr;
    uint32_t sol_stat;
    uint32_t pos_type;
    double lla[3];
    float undulation;
    uint32_t datum_id;
    float lla_std[3];
    uint32_t std_id;
    float diff_age;
    float sol_age;
    uint8_t n_sv;
    uint8_t n_sol_sv;
    uint8_t rev[3];
    uint8_t ext_sol_stat;
    uint8_t gal_bd_sig_mask;
    uint8_t gps_glo_sig_mask;
    uint32_t crc;
} oem_bestpos_t;

typedef struct __attribute__((__packed__)) /* UM982 AGIRC msg */
{
    unicore_hdr_t  hdr;
    uint32_t bestpos_type;
    float    bestpos_ght;
    double   bestpos_lat;
    double   bestpos_lon;
    float    bestpos_hgtstd;
    float    bestpos_latstd;
    float    bestpos_lonstd;
    float    bestpos_diffage;
    uint32_t psrpos_type;
    float    psrpos_ght;
    double   psrpos_lat;
    double   psrpos_lon;
    float    undulation;
    uint8_t  bestpos_svs;
    uint8_t  bestpos_solnsvs;
    uint8_t  psrpos_svs;
    uint8_t  psrpos_solnsvs;
    double   psrvel_north;
    double   psrvel_east;
    double   psrvel_ground;
    uint32_t heading_type;
    float    heading_length;
    float    heading_degree;
    float    heading_pitch;
    uint8_t  heading_trackedsvs;
    uint8_t  heading_solnsvs;
    uint8_t  heading_ggl1;
    uint8_t  heading_ggl1l2;
    float    gdop;
    float    pdop;
    float    hdop;
    float    htdop;
    float    tdop;
    float    cutoff;
    uint16_t PRN_No;
    uint16_t PRN_list[41];
    uint32_t        crc;
} oem_pvtsln_t;

typedef struct __attribute__((__packed__))
{
    oem_hdr_t hdr;
    uint32_t p_sol_stat;
    uint32_t pos_type;
    double P_XYZ[3];
    float P_std_XYZ[3];
    uint32_t v_sol_stat;
    uint32_t v_vel_type;
    double V_XYZ[3];
    float V_std_XYZ[3];
    char stn_id[4];
    float v_latency;
    float diff_age;
    float sol_age;
    uint8_t nsv;
    uint8_t soln_nsv;
    uint8_t ggL1;
    uint8_t soln_multi_svs;
    uint8_t rev;
    uint8_t ext_sol_stat;
    uint8_t gal_bd_sig_mask;
    uint8_t gps_glo_sig_mask;
    uint32_t crc;
} oem_bestxyz_t;

typedef struct
{
    int               nbyte;
    int               len;
    uint8_t           buf[MAXRAWLEN];
    oem_agric_t       agric;
    oem_heading_t     heading;
    oem_inspvax_t     inspvax;
    oem_rawimux_t     rawimux;
    oem_gnssrcv_t     gnssrcv;
    oem_bestpos_t     bestpos;
    oem_pvtsln_t      pvtsln;     
    oem_bestxyz_t     bestxyz;     
    uint32_t          type;
}oem_raw_t;

int input_oem(oem_raw_t *raw, uint8_t ch);

#endif