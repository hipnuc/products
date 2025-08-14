#ifndef HIPNUC_CAN_PARSER_H
#define HIPNUC_CAN_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Remove Linux-specific includes
// #include <linux/can.h>

// Define platform-independent CAN frame structure
typedef struct {
    uint32_t can_id;    // CAN ID (with flags)
    uint8_t can_dlc;    // Data length code
    uint8_t data[8];    // CAN data
} hipnuc_can_frame_t;

// Define CAN flags (compatible with Linux CAN)
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

// CAN sensor data structure (unchanged)
typedef struct {
    // Frame information
    uint32_t can_id;        // Original CAN ID
    bool is_extended;       // Extended frame flag
    uint8_t dlc;           // Data length code
    
    // Accelerometer (m/s²)
    float accel_x, accel_y, accel_z;
    
    // Gyroscope (rad/s)
    float gyro_x, gyro_y, gyro_z;
    
    // Magnetometer (µT)
    float mag_x, mag_y, mag_z;
    
    // Temperature (°C)
    float temperature;
    
    // Quaternion (unitless)
    float quat_w, quat_x, quat_y, quat_z;
    
    // Euler angles (rad)
    float roll, pitch, yaw;
    
    // inclination angle, (deg)
    float incli_x, incli_y;

    // Atmospheric pressure (Pa)
    float pressure;
    
    // Time information
    uint32_t timestamp_ms;
    uint8_t hours, minutes, seconds;
    uint16_t milliseconds;
    
    // J1939 specific time fields
    uint8_t utc_year;      // 0-99 (20 = 2020)
    uint8_t utc_month;     // 0-12
    uint8_t utc_day;       // 0-31

    // Status and metadata
    uint8_t node_id;
    
} can_sensor_data_t;

// Function prototypes (unchanged)
int hipnuc_can_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data, uint8_t target_node_id);
const char* hipnuc_can_get_msg_type_name(int msg_type);
void hipnuc_can_format_data(int msg_type, const can_sensor_data_t *data, char *buffer, size_t buf_size);

#endif // HIPNUC_CAN_PARSER_H
