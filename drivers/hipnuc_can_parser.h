#ifndef HIPNUC_CAN_PARSER_H
#define HIPNUC_CAN_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Platform-independent CAN frame structure
typedef struct {
    uint32_t can_id;    // CAN ID (with flags)
    uint8_t can_dlc;    // Data length code
    uint8_t data[8];    // CAN data
    uint64_t hw_ts_us;  
} hipnuc_can_frame_t;

// CAN flags (compatible with Linux CAN)
#define HIPNUC_CAN_EFF_FLAG    0x80000000U  // Extended frame flag
#define HIPNUC_CAN_SFF_MASK    0x000007FFU  // Standard frame mask
#define HIPNUC_CAN_EFF_MASK    0x1FFFFFFFU  // Extended frame mask

// Message types
#define CAN_MSG_ERROR       0
#define CAN_MSG_ACCEL       1
#define CAN_MSG_GYRO        2
#define CAN_MSG_MAG         3
#define CAN_MSG_TEMP        4
#define CAN_MSG_QUAT        5
#define CAN_MSG_EULER       6
#define CAN_MSG_PRESSURE    7
#define CAN_MSG_GNSS_POS    8
#define CAN_MSG_GNSS_VEL    9
#define CAN_MSG_INCLI       10
#define CAN_MSG_TIME        11
#define CAN_MSG_PITCH_ROLL  12
#define CAN_MSG_YAW         13
#define CAN_MSG_UNKNOWN     99

// CAN sensor data structure (aligned with Qt field names)
typedef struct {
    // Node ID
    uint8_t node_id;
    uint64_t hw_ts_us;
    
    // Accelerometer (m/s²)
    float acc_x;
    float acc_y;
    float acc_z;
    
    // Gyroscope (rad/s)
    float gyr_x;
    float gyr_y;
    float gyr_z;
    
    // Magnetometer (µT)
    float mag_x;
    float mag_y;
    float mag_z;
    
    // Quaternion (normalized)
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    
    // Euler angles (rad)
    float roll;
    float pitch;
    float imu_yaw;      // For IMU-only yaw
    
    // Inclination (degrees)
    float incli_x;
    float incli_y;
    
    // Environmental
    float temperature;  // °C
    float pressure;     // Pa
    
    // Time information
    uint8_t utc_year;   // Year (0-99)
    uint8_t utc_month;  // Month (1-12)
    uint8_t utc_day;    // Day (1-31)
    uint8_t hours;      // Hours (0-23)
    uint8_t minutes;    // Minutes (0-59)
    uint8_t seconds;    // Seconds (0-59)
    uint16_t milliseconds; // Milliseconds (0-999)
    uint32_t timestamp_ms; // Total milliseconds since midnight
    
    // GNSS position
    double ins_lat;     // Latitude (degrees)
    double ins_lon;     // Longitude (degrees)
    double ins_msl;     // Mean sea level altitude (m)
    
    // GNSS velocity (m/s)
    float ins_vel_e;    // East velocity
    float ins_vel_n;    // North velocity
    float ins_vel_u;    // Up velocity
} can_sensor_data_t;

// JSON output buffer structure
typedef struct {
    char buffer[512];   // JSON string buffer
    size_t length;      // Current length
} can_json_output_t;

// Function prototypes
int hipnuc_can_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data);
int hipnuc_can_to_json(const can_sensor_data_t *data, int msg_type, can_json_output_t *output);

#endif // HIPNUC_CAN_PARSER_H
