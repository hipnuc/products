#ifndef CANOPEN_PARSER_H
#define CANOPEN_PARSER_H

#include "hipnuc_can_common.h"

int canopen_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data);

#endif