#ifndef __CH_SERIAL_H_
#define __CH_SERIAL_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* dump logs */
// #define  CH_DEBUG

#define MAXRAWLEN (512) /* max raw frame long */

/* data items */
#define kItemID (0x90)
#define kItemAccRaw (0xA0)
#define kItemGyrRaw (0xB0)
#define kItemMagRaw (0xC0)
#define kItemRotationEul (0xD0)
#define kItemRotationQuat (0xD1)
#define kItemPressure (0xF0)
#define KItemIMUSOL (0x91)

typedef struct __attribute__((packed))
{
    uint8_t tag;            /* data packet tag, shoue be 0x91 */
    uint16_t pps_sync_ms;   /* "The time elapsed from the last synchronization input transition signal to the moment this frame is sent, in ms.*/
    int8_t temp;            /* temperature */
    float prs;              /* pressure */
    uint32_t ts;            /* device time in ms */
    float acc[3];           /* acceleratmer (G, 1G= 9.8m/^(2)) */
    float gyr[3];           /* gyroscople (dps, degree per second) */
    float mag[3];           /* megnormator (uT) */
    float eul[3];           /* eular angles:Roll/Pitch/Yaw*/
    float quat[4];          /* quaternion */
} id0x91_t;

typedef struct
{
    int nbyte;              /* number of bytes in message buffer */
    int len;                /* message length (bytes) */
    uint8_t buf[MAXRAWLEN]; /* message raw buffer */
    id0x91_t imu;           /* imu data list, if (HI226/HI229/CH100/CH110, use imu[0]) */
} hipnuc_raw_t;

int ch_serial_input(hipnuc_raw_t *raw, uint8_t data);
int ch_imu_data2str(hipnuc_raw_t *raw, char *buf, size_t buf_size);

#endif
