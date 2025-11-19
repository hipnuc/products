#include "hipnuc_can_common.h"
#include <stdarg.h>
#include <stdio.h>

static void json_append(can_json_output_t *out, const char *fmt, ...)
{
    if (out->length >= sizeof(out->buffer) - 1) return;
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(out->buffer + out->length, sizeof(out->buffer) - out->length, fmt, args);
    va_end(args);
    if (written > 0) out->length += written;
}

int hipnuc_can_to_json(const can_sensor_data_t *data, int msg_type, can_json_output_t *output)
{
    if (!data || !output) return -1;
    output->length = 0;
    output->buffer[0] = '\0';
    json_append(output, "{\"node_id\":%d,\"hw_ts_us\":%llu,\"data\":{", data->node_id, (unsigned long long)data->hw_ts_us);
    switch (msg_type) {
        case CAN_MSG_ACCEL:
            json_append(output, "\"acc_x\":%.6f,\"acc_y\":%.6f,\"acc_z\":%.6f", data->acc_x, data->acc_y, data->acc_z);
            break;
        case CAN_MSG_GYRO:
            json_append(output, "\"gyr_x\":%.6f,\"gyr_y\":%.6f,\"gyr_z\":%.6f", data->gyr_x, data->gyr_y, data->gyr_z);
            break;
        case CAN_MSG_MAG:
            json_append(output, "\"mag_x\":%.3f,\"mag_y\":%.3f,\"mag_z\":%.3f", data->mag_x, data->mag_y, data->mag_z);
            break;
        case CAN_MSG_QUAT:
            json_append(output, "\"quat_w\":%.4f,\"quat_x\":%.4f,\"quat_y\":%.4f,\"quat_z\":%.4f", data->quat_w, data->quat_x, data->quat_y, data->quat_z);
            break;
        case CAN_MSG_EULER:
        case CAN_MSG_PITCH_ROLL:
            json_append(output, "\"roll\":%.6f,\"pitch\":%.6f", data->roll, data->pitch);
            if (msg_type == CAN_MSG_EULER) json_append(output, ",\"imu_yaw\":%.6f", data->imu_yaw);
            break;
        case CAN_MSG_YAW:
            json_append(output, "\"imu_yaw\":%.6f", data->imu_yaw);
            break;
        case CAN_MSG_PRESSURE:
            json_append(output, "\"temperature\":%.2f,\"pressure\":%.2f", data->temperature, data->pressure);
            break;
        case CAN_MSG_INCLI:
            json_append(output, "\"incli_x\":%.2f,\"incli_y\":%.2f", data->incli_x, data->incli_y);
            break;
        case CAN_MSG_TIME:
            json_append(output, "\"utc_year\":%d,\"utc_month\":%d,\"utc_day\":%d,\"hours\":%d,\"minutes\":%d,\"seconds\":%d,\"milliseconds\":%d,\"timestamp_ms\":%u", data->utc_year, data->utc_month, data->utc_day, data->hours, data->minutes, data->seconds, data->milliseconds, data->timestamp_ms);
            break;
        case CAN_MSG_GNSS_POS:
            json_append(output, "\"ins_lon\":%.8f,\"ins_lat\":%.8f,\"ins_msl\":%.3f,\"undulation\":%.2f,\"diff_age_s\":%.2f", data->ins_lon, data->ins_lat, data->ins_msl, data->undulation, data->diff_age_s);
            break;
        case CAN_MSG_GNSS_VEL:
            json_append(output, "\"ins_vel_e\":%.3f,\"ins_vel_n\":%.3f,\"ins_vel_u\":%.3f,\"ins_speed\":%.3f", data->ins_vel_e, data->ins_vel_n, data->ins_vel_u, data->ins_speed);
            break;
        case CAN_MSG_GNSS_STATUS:
            json_append(output, "\"solq_pos\":%d,\"solq_heading\":%d,\"nv_pos\":%d,\"nv_heading\":%d,\"ins_status\":%d", data->solq_pos, data->solq_heading, data->nv_pos, data->nv_heading, data->ins_status);
            break;
        default:
            json_append(output, "\"error\":\"unknown message type\"");
            break;
    }
    json_append(output, "}}\n");
    return output->length;
}

uint8_t hipnuc_can_extract_node_id(uint32_t can_id)
{
    if (can_id & HIPNUC_CAN_EFF_FLAG) return (uint8_t)(can_id & 0xFF);
    return (uint8_t)(can_id & 0x7F);
}