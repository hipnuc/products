// hipnuc_can_parser.c
#include "hipnuc_can_parser.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

// CANopen TPDO base IDs
#define TPDO1_BASE  0x180  // Accelerometer
#define TPDO2_BASE  0x280  // Gyroscope  
#define TPDO3_BASE  0x380  // Pitch/Roll/Yaw
#define TPDO4_BASE  0x480  // Quaternion
#define TPDO6_BASE  0x680  // Pressure
#define TPDO7_BASE  0x780  // Inclination

// J1939 PGNs
#define J1939_PGN_TIME       0xFF2F  // PGN65327 - Time info
#define J1939_PGN_ACCEL      0xFF34  // PGN65332 - Acceleration
#define J1939_PGN_GYRO       0xFF37  // PGN65335 - Angular rate
#define J1939_PGN_MAG        0xFF3A  // PGN65338 - Magnetic field
#define J1939_PGN_PITCH_ROLL 0xFF3D  // PGN65341 - Pitch/Roll
#define J1939_PGN_YAW        0xFF41  // PGN65345 - Yaw
#define J1939_PGN_QUAT       0xFF46  // PGN65350 - Quaternion
#define J1939_PGN_INCLINE    0xFF4A  // PGN65354 - Inclination


// Extract node ID from CAN ID
static uint8_t extract_node_id(uint32_t can_id)
{
    if (can_id & HIPNUC_CAN_EFF_FLAG) {
        // J1939 extended frame: node ID in source address (bits 0-7)
        return (uint8_t)(can_id & 0xFF);
    } else {
        // CANopen standard frame: node ID in lower 7 bits
        return (uint8_t)(can_id & 0x7F);
    }
}

// Parse CANopen TPDO1 - Accelerometer
static int parse_canopen_accel(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->acc_x = raw[0] * 0.001f * 9.81f;  // mg to m/s²
    data->acc_y = raw[1] * 0.001f * 9.81f;
    data->acc_z = raw[2] * 0.001f * 9.81f;
    return CAN_MSG_ACCEL;
}

// Parse CANopen TPDO2 - Gyroscope
static int parse_canopen_gyro(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->gyr_x = raw[0] * 0.1f * 0.017453f;  // 0.1°/s to rad/s
    data->gyr_y = raw[1] * 0.1f * 0.017453f;
    data->gyr_z = raw[2] * 0.1f * 0.017453f;
    return CAN_MSG_GYRO;
}

// Parse CANopen TPDO3 - Euler angles
static int parse_canopen_euler(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->roll = raw[0] * 0.01f * 0.017453f;     // Roll
    data->pitch = raw[1] * 0.01f * 0.017453f;    // Pitch
    data->imu_yaw = raw[2] * 0.01f * 0.017453f;  // Yaw (IMU-only)
    return CAN_MSG_EULER;
}

// Parse CANopen TPDO4 - Quaternion
static int parse_canopen_quat(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->quat_w = raw[0] * 0.0001f;
    data->quat_x = raw[1] * 0.0001f;
    data->quat_y = raw[2] * 0.0001f;
    data->quat_z = raw[3] * 0.0001f;
    return CAN_MSG_QUAT;
}

// Parse CANopen TPDO6 - Pressure
static int parse_canopen_pressure(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->pressure = *raw;  // Unit: Pa
    return CAN_MSG_PRESSURE;
}

// Parse CANopen TPDO7 - Inclination
static int parse_canopen_inclination(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->incli_x = raw[0] * 0.01f;  // X-axis inclination (degrees)
    data->incli_y = raw[1] * 0.01f;  // Y-axis inclination (degrees)
    return CAN_MSG_INCLI;
}

// Parse J1939 Time Information - PGN65327(FF2F)
static int parse_j1939_time(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    const uint8_t *raw = frame->data;
    
    data->utc_year = raw[0];        // 0-99
    data->utc_month = raw[1];       // 0-12
    data->utc_day = raw[2];         // 0-31
    data->hours = raw[3];           // 0-23
    data->minutes = raw[4];         // 0-59
    data->seconds = raw[5];         // 0-59
    data->milliseconds = raw[6] | (raw[7] << 8);  // Milliseconds (LSB first)
    
    // Calculate timestamp_ms (time since midnight)
    data->timestamp_ms = (uint32_t)data->hours * 3600000UL + 
                        (uint32_t)data->minutes * 60000UL + 
                        (uint32_t)data->seconds * 1000UL + 
                        data->milliseconds;
    
    return CAN_MSG_TIME;
}

// Parse J1939 Acceleration - PGN65332(FF34)
static int parse_j1939_accel(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale: 0.00048828 G per bit -> convert to m/s²
    data->acc_x = raw[0] * 0.00048828f * 9.81f;
    data->acc_y = raw[1] * 0.00048828f * 9.81f;
    data->acc_z = raw[2] * 0.00048828f * 9.81f;
    return CAN_MSG_ACCEL;
}

// Parse J1939 Angular Rate - PGN65335(FF37)
static int parse_j1939_gyro(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale: 0.061035 deg/s per bit -> convert to rad/s
    data->gyr_x = raw[0] * 0.061035f * 0.017453f;
    data->gyr_y = raw[1] * 0.061035f * 0.017453f;
    data->gyr_z = raw[2] * 0.061035f * 0.017453f;
    return CAN_MSG_GYRO;
}

// Parse J1939 Magnetic Field - PGN65338(FF3A)
static int parse_j1939_mag(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale: 0.030517 µT per bit
    data->mag_x = raw[0] * 0.030517f;
    data->mag_y = raw[1] * 0.030517f;
    data->mag_z = raw[2] * 0.030517f;
    return CAN_MSG_MAG;
}

// Parse J1939 Pitch/Roll - PGN65341(FF3D)
static int parse_j1939_pitch_roll(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    // Scale: 0.001° per bit -> convert to rad
    data->roll = raw[0] * 0.001f * 0.017453f;
    data->pitch = raw[1] * 0.001f * 0.017453f;
    return CAN_MSG_PITCH_ROLL;
}

// Parse J1939 Yaw - PGN65345(FF41)
static int parse_j1939_yaw(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    // Scale: 0.001° per bit, 0-360° range -> convert to rad
    data->imu_yaw = raw[0] * 0.001f * 0.017453f;
    return CAN_MSG_YAW;
}

// Parse J1939 Quaternion - PGN65350(FF46)
static int parse_j1939_quat(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale: 0.0001 per bit
    data->quat_w = raw[0] * 0.0001f;
    data->quat_x = raw[1] * 0.0001f;
    data->quat_y = raw[2] * 0.0001f;
    data->quat_z = raw[3] * 0.0001f;
    return CAN_MSG_QUAT;
}

// Parse J1939 Inclination - PGN65354(FF4A)
static int parse_j1939_inclination(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    // Scale: 0.01° per bit
    data->incli_x = raw[0] * 0.01f;
    data->incli_y = raw[1] * 0.01f;
    return CAN_MSG_INCLI;
}

// Main parsing function
int hipnuc_can_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    if (!frame || !data) {
        return CAN_MSG_ERROR;
    }
    
    // Extract node ID
    data->node_id = extract_node_id(frame->can_id);
    data->hw_ts_us = frame->hw_ts_us;
    
    uint32_t id = frame->can_id & HIPNUC_CAN_EFF_MASK;
    
    // Check if extended frame (J1939)
    if (frame->can_id & HIPNUC_CAN_EFF_FLAG) {
        // Extract PGN from extended CAN ID
        uint32_t pgn = (id >> 8) & 0xFFFF;
        
        switch (pgn) {
            case J1939_PGN_TIME:       return parse_j1939_time(frame, data);
            case J1939_PGN_ACCEL:      return parse_j1939_accel(frame, data);
            case J1939_PGN_GYRO:       return parse_j1939_gyro(frame, data);
            case J1939_PGN_MAG:        return parse_j1939_mag(frame, data);
            case J1939_PGN_PITCH_ROLL: return parse_j1939_pitch_roll(frame, data);
            case J1939_PGN_YAW:        return parse_j1939_yaw(frame, data);
            case J1939_PGN_QUAT:       return parse_j1939_quat(frame, data);
            case J1939_PGN_INCLINE:    return parse_j1939_inclination(frame, data);
            default:                   return CAN_MSG_UNKNOWN;
        }
    } 
    // Standard frame (CANopen)
    else {
        uint32_t base_id = id & 0x780;  // Mask to get TPDO base
        
        switch (base_id) {
            case TPDO1_BASE: return parse_canopen_accel(frame, data);
            case TPDO2_BASE: return parse_canopen_gyro(frame, data);
            case TPDO3_BASE: return parse_canopen_euler(frame, data);
            case TPDO4_BASE: return parse_canopen_quat(frame, data);
            case TPDO6_BASE: return parse_canopen_pressure(frame, data);
            case TPDO7_BASE: return parse_canopen_inclination(frame, data);
            default:         return CAN_MSG_UNKNOWN;
        }
    }
}

// JSON formatting helper - append formatted string
static void json_append(can_json_output_t *out, const char *fmt, ...)
{
    if (out->length >= sizeof(out->buffer) - 1) return;
    
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(out->buffer + out->length, 
                           sizeof(out->buffer) - out->length, 
                           fmt, args);
    va_end(args);
    
    if (written > 0) {
        out->length += written;
    }
}

// Convert parsed data to JSON string
int hipnuc_can_to_json(const can_sensor_data_t *data, int msg_type, can_json_output_t *output)
{
    if (!data || !output) return -1;
    
    output->length = 0;
    output->buffer[0] = '\0';
    
    // Start JSON object
    json_append(output, "{\"node_id\":%d,\"hw_ts_us\":%llu,\"data\":{", data->node_id, (unsigned long long)data->hw_ts_us);
    
    // Add fields based on message type
    switch (msg_type) {
        case CAN_MSG_ACCEL:
            json_append(output, "\"acc_x\":%.6f,\"acc_y\":%.6f,\"acc_z\":%.6f",
                       data->acc_x, data->acc_y, data->acc_z);
            break;
            
        case CAN_MSG_GYRO:
            json_append(output, "\"gyr_x\":%.6f,\"gyr_y\":%.6f,\"gyr_z\":%.6f",
                       data->gyr_x, data->gyr_y, data->gyr_z);
            break;
            
        case CAN_MSG_MAG:
            json_append(output, "\"mag_x\":%.3f,\"mag_y\":%.3f,\"mag_z\":%.3f",
                       data->mag_x, data->mag_y, data->mag_z);
            break;
            
        case CAN_MSG_QUAT:
            json_append(output, "\"quat_w\":%.4f,\"quat_x\":%.4f,\"quat_y\":%.4f,\"quat_z\":%.4f",
                       data->quat_w, data->quat_x, data->quat_y, data->quat_z);
            break;
            
        case CAN_MSG_EULER:
        case CAN_MSG_PITCH_ROLL:
            json_append(output, "\"roll\":%.6f,\"pitch\":%.6f", data->roll, data->pitch);
            if (msg_type == CAN_MSG_EULER) {
                json_append(output, ",\"imu_yaw\":%.6f", data->imu_yaw);
            }
            break;
            
        case CAN_MSG_YAW:
            json_append(output, "\"imu_yaw\":%.6f", data->imu_yaw);
            break;
            
        case CAN_MSG_PRESSURE:
            json_append(output, "\"pressure\":%.2f", data->pressure);
            break;
            
        case CAN_MSG_INCLI:
            json_append(output, "\"incli_x\":%.2f,\"incli_y\":%.2f",
                       data->incli_x, data->incli_y);
            break;
            
        case CAN_MSG_TIME:
            json_append(output, "\"utc_year\":%d,\"utc_month\":%d,\"utc_day\":%d,"
                              "\"hours\":%d,\"minutes\":%d,\"seconds\":%d,"
                              "\"milliseconds\":%d,\"timestamp_ms\":%u",
                       data->utc_year, data->utc_month, data->utc_day,
                       data->hours, data->minutes, data->seconds,
                       data->milliseconds, data->timestamp_ms);
            break;
            
        default:
            json_append(output, "\"error\":\"unknown message type\"");
            break;
    }
    
    // Close JSON object
    json_append(output, "}}\n");
    
    return output->length;
}
