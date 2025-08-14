// hipnuc_can_parser.c
#include "hipnuc_can_parser.h"
#include <string.h>
#include <stdio.h> 

// CANopen TPDO base IDs
#define TPDO1_BASE  0x180  // Accelerometer
#define TPDO2_BASE  0x280  // Gyroscope  
#define TPDO3_BASE  0x380  // Pitch/Roll/Yaw
#define TPDO4_BASE  0x480  // Quaternion
#define TPDO6_BASE  0x680  // Pressure
#define TPDO7_BASE  0x780  // Inclination

// J1939 PGNs - complete list from PDF
#define J1939_PGN_TIME       0xFF2F  // PGN65327 - Time info
#define J1939_PGN_ACCEL      0xFF34  // PGN65332 - Acceleration
#define J1939_PGN_GYRO       0xFF37  // PGN65335 - Angular rate
#define J1939_PGN_MAG        0xFF3A  // PGN65338 - Magnetic field
#define J1939_PGN_PITCH_ROLL 0xFF3D  // PGN65341 - Pitch/Roll
#define J1939_PGN_YAW        0xFF41  // PGN65345 - Yaw
#define J1939_PGN_QUAT       0xFF46  // PGN65350 - Quaternion
#define J1939_PGN_INCLINE    0xFF4A  // PGN65354 - Inclination

static const char* msg_type_names[] = {
    "ERROR", "ACCEL", "GYRO", "MAG", "TEMP", "QUAT", "EULER",
    "PRESSURE", "GNSS_POS", "GNSS_VEL", "INCLINATION", "TIME", 
    "PITCH_ROLL", "YAW", "UNKNOWN"
};

const char* hipnuc_can_get_msg_type_name(int msg_type)
{
    if (msg_type >= 0 && msg_type < sizeof(msg_type_names)/sizeof(msg_type_names[0])) {
        return msg_type_names[msg_type];
    }
    return "INVALID";
}

// Parse CANopen TPDO1 - Accelerometer (加速度)
static int parse_canopen_accel(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->accel_x = raw[0] * 0.001f * 9.81f;  // mg to m/s²
    data->accel_y = raw[1] * 0.001f * 9.81f;
    data->accel_z = raw[2] * 0.001f * 9.81f;
    return CAN_MSG_ACCEL;
}

// Parse CANopen TPDO2 - Gyroscope (角速度)
static int parse_canopen_gyro(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->gyro_x = raw[0] * 0.1f * 0.017453f;  // 0.1°/s to rad/s
    data->gyro_y = raw[1] * 0.1f * 0.017453f;
    data->gyro_z = raw[2] * 0.1f * 0.017453f;
    return CAN_MSG_GYRO;
}

// Parse CANopen TPDO3 - Pitch/Roll/Yaw (俯仰横滚角)
static int parse_canopen_euler(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->roll = raw[0] * 0.01f * 0.017453f;    // Roll (横滚)
    data->pitch = raw[1] * 0.01f * 0.017453f;   // Pitch (俯仰)
    data->yaw = raw[2] * 0.01f * 0.017453f;     // Yaw (航向)
    return CAN_MSG_EULER;
}

// Parse CANopen TPDO4 - Quaternion (四元数)
static int parse_canopen_quat(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    data->quat_w = raw[0] * 0.0001f;
    data->quat_x = raw[1] * 0.0001f;
    data->quat_y = raw[2] * 0.0001f;
    data->quat_z = raw[3] * 0.0001f;
    return CAN_MSG_QUAT;
}

// Parse CANopen TPDO6 - Pressure (气压)
static int parse_canopen_pressure(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->pressure = *raw;  // Unit: Pa, no scaling needed
    return CAN_MSG_PRESSURE;
}

// Parse CANopen TPDO7 - Inclination (倾角仪输出)
static int parse_canopen_inclination(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    data->incli_x = raw[0] * 0.01f;   // X-axis inclination
    data->incli_y = raw[1] * 0.01f;  // Y-axis inclination  
    return CAN_MSG_INCLI;
}

// Parse J1939 Time Information - PGN65327(FF2F)
static int parse_j1939_time(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    const uint8_t *raw = frame->data;
    
    // Parse according to J1939 time format from PDF
    data->utc_year = raw[0];        // 0-99
    data->utc_month = raw[1];       // 0-12
    data->utc_day = raw[2];         // 0-31
    data->hours = raw[3];           // 0-23
    data->minutes = raw[4];         // 0-59
    data->seconds = raw[5];         // 0-59
    
    // Milliseconds from bytes 6-7 (uint16_t, LSB first)
    data->milliseconds = raw[6] | (raw[7] << 8);
    
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
    // Scale factor: 0.00048828 G per bit
    data->accel_x = raw[0] * 0.00048828f * 9.81f;  // Convert to m/s²
    data->accel_y = raw[1] * 0.00048828f * 9.81f;
    data->accel_z = raw[2] * 0.00048828f * 9.81f;
    return CAN_MSG_ACCEL;
}

// Parse J1939 Angular Rate - PGN65335(FF37)
static int parse_j1939_gyro(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale factor: 0.061035 deg/s per bit
    data->gyro_x = raw[0] * 0.061035f * 0.017453f;  // Convert to rad/s
    data->gyro_y = raw[1] * 0.061035f * 0.017453f;
    data->gyro_z = raw[2] * 0.061035f * 0.017453f;
    return CAN_MSG_GYRO;
}

// Parse J1939 Magnetic Field - PGN65338(FF3A)
static int parse_j1939_mag(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale factor: 0.030517 µT per bit
    data->mag_x = raw[0] * 0.030517f;
    data->mag_y = raw[1] * 0.030517f;
    data->mag_z = raw[2] * 0.030517f;
    return CAN_MSG_MAG;
}

// Parse J1939 Pitch/Roll - PGN65341(FF3D)
static int parse_j1939_pitch_roll(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    // Scale factor: 0.001° per bit
    data->roll = raw[0] * 0.001f * 0.017453f;   // Convert to rad
    data->pitch = raw[1] * 0.001f * 0.017453f;
    return CAN_MSG_PITCH_ROLL;
}

// Parse J1939 Yaw - PGN65345(FF41)
static int parse_j1939_yaw(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int32_t *raw = (int32_t*)frame->data;
    // Scale factor: 0.001° per bit, 0-360° range
    data->yaw = raw[0] * 0.001f * 0.017453f;    // Convert to rad
    return CAN_MSG_YAW;
}

// Parse J1939 Quaternion - PGN65350(FF46)
static int parse_j1939_quat(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    int16_t *raw = (int16_t*)frame->data;
    // Scale factor: 0.0001 per bit, range -1.0000 to 1.0000
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
    // Scale factor: 0.001° per bit
    data->incli_x = raw[0] * 0.001f;  // X-axis inclination
    data->incli_y = raw[1] * 0.001f;  // Y-axis inclination
    return CAN_MSG_INCLI;
}

// Parse CANopen frame
static int parse_canopen_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    uint16_t base_id = frame->can_id & 0x780;  // Extract TPDO base ID
    
    switch (base_id) {
        case TPDO1_BASE:
            return parse_canopen_accel(frame, data);
        case TPDO2_BASE:
            return parse_canopen_gyro(frame, data);
        case TPDO3_BASE:
            return parse_canopen_euler(frame, data);
        case TPDO4_BASE:
            return parse_canopen_quat(frame, data);
        case TPDO6_BASE:
            return parse_canopen_pressure(frame, data);
        case TPDO7_BASE:
            return parse_canopen_inclination(frame, data);
        default:
            return CAN_MSG_UNKNOWN;
    }
}

// Parse J1939 frame
static int parse_j1939_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data)
{
    uint32_t pgn = (frame->can_id >> 8) & 0xFFFF;
    
    switch (pgn) {
        case J1939_PGN_TIME:
            return parse_j1939_time(frame, data);
        case J1939_PGN_ACCEL:
            return parse_j1939_accel(frame, data);
        case J1939_PGN_GYRO:
            return parse_j1939_gyro(frame, data);
        case J1939_PGN_MAG:
            return parse_j1939_mag(frame, data);
        case J1939_PGN_PITCH_ROLL:
            return parse_j1939_pitch_roll(frame, data);
        case J1939_PGN_YAW:
            return parse_j1939_yaw(frame, data);
        case J1939_PGN_QUAT:
            return parse_j1939_quat(frame, data);
        case J1939_PGN_INCLINE:
            return parse_j1939_inclination(frame, data);
        default:
            return CAN_MSG_UNKNOWN;
    }
}

// Main parsing function with node ID filtering
int hipnuc_can_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data, uint8_t target_node_id)
{
    if (!frame || !data) {
        return CAN_MSG_ERROR;
    }
    
    // Store frame information
    data->can_id = frame->can_id;
    data->is_extended = (frame->can_id & HIPNUC_CAN_EFF_FLAG) ? true : false;
    data->dlc = frame->can_dlc;
    
    // Protocol-specific node ID extraction and filtering
    if (data->is_extended) {
        // J1939: Source address is in lower 8 bits
        data->node_id = frame->can_id & 0xFF;
        if (target_node_id != 0 && data->node_id != target_node_id) {
            return CAN_MSG_UNKNOWN;
        }
        return parse_j1939_frame(frame, data);
    } else {
        // CANopen: Node ID is in lower 7 bits
        data->node_id = frame->can_id & 0x7F;
        if (target_node_id != 0 && data->node_id != target_node_id) {
            return CAN_MSG_UNKNOWN;
        }
        return parse_canopen_frame(frame, data);
    }
}


// Format sensor data for display
void hipnuc_can_format_data(int msg_type, const can_sensor_data_t *data, char *buffer, size_t buf_size)
{
    if (!data || !buffer || buf_size == 0) {
        if (buffer && buf_size > 0) {
            buffer[0] = '\0';
        }
        return;
    }
    
    char data_str[256];
    char frame_info[32];
    
    // Format frame information
    if (data->is_extended) {
        snprintf(frame_info, sizeof(frame_info), "[0x%08X] ", data->can_id & HIPNUC_CAN_EFF_MASK);
    } else {
        snprintf(frame_info, sizeof(frame_info), "[0x%03X] ", data->can_id & HIPNUC_CAN_SFF_MASK);
    }
    
    // Format sensor data
    switch (msg_type) {
        case CAN_MSG_ACCEL:
            snprintf(data_str, sizeof(data_str), "X=%.3f Y=%.3f Z=%.3f m/s²", 
                    data->accel_x, data->accel_y, data->accel_z);
            break;
        case CAN_MSG_GYRO:
            snprintf(data_str, sizeof(data_str), "X=%.3f Y=%.3f Z=%.3f rad/s", 
                    data->gyro_x, data->gyro_y, data->gyro_z);
            break;
        case CAN_MSG_MAG:
            snprintf(data_str, sizeof(data_str), "X=%.2f Y=%.2f Z=%.2f µT", 
                    data->mag_x, data->mag_y, data->mag_z);
            break;
        case CAN_MSG_EULER:
            snprintf(data_str, sizeof(data_str), "R=%.3f P=%.3f Y=%.3f deg", 
                    data->roll * 57.32484f, data->pitch * 57.32484f, data->yaw * 57.32484f);
            break;
        case CAN_MSG_QUAT:
            snprintf(data_str, sizeof(data_str), "W=%.4f X=%.4f Y=%.4f Z=%.4f", 
                    data->quat_w, data->quat_x, data->quat_y, data->quat_z);
            break;
        case CAN_MSG_PRESSURE:
            snprintf(data_str, sizeof(data_str), "%.1f Pa", data->pressure);
            break;
        case CAN_MSG_INCLI:
            snprintf(data_str, sizeof(data_str), "X=%.3f Y=%.3f °", data->incli_x, data->incli_y);
            break;
        case CAN_MSG_TIME:
            if (data->utc_year != 0) {
                // J1939 format with date
                snprintf(data_str, sizeof(data_str), "20%02d-%02d-%02d %02d:%02d:%02d.%03d UTC", 
                        data->utc_year, data->utc_month, data->utc_day,
                        data->hours, data->minutes, data->seconds, data->milliseconds);
            } else {
                // Standard time format
                snprintf(data_str, sizeof(data_str), "%02d:%02d:%02d.%03d UTC", 
                        data->hours, data->minutes, data->seconds, data->milliseconds);
            }
            break;
        case CAN_MSG_PITCH_ROLL:
            snprintf(data_str, sizeof(data_str), "R=%.3f P=%.3f deg", 
                    data->roll * 57.32484f, data->pitch * 57.32484f);
            break;
        case CAN_MSG_YAW:
            snprintf(data_str, sizeof(data_str), "Y=%.3f deg", data->yaw * 57.32484f);
            break;
        case CAN_MSG_TEMP:
            snprintf(data_str, sizeof(data_str), "%.2f °C", data->temperature);
            break;
        default:
            snprintf(data_str, sizeof(data_str), "Unknown data");
            break;
    }
    
    // Combine frame info and data
    snprintf(buffer, buf_size, "%s%s", frame_info, data_str);
}
