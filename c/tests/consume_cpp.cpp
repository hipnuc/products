// A C++ application including every core header and linking the C targets.
#include <cstdio>
#include <cstring>

#include "hipnuc_can_frame.h"
#include "hipnuc_dec.h"
#include "hipnuc_j1939.h"
#include "hipnuc_json.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"

int main()
{
    hipnuc_raw_t raw;
    std::memset(&raw, 0, sizeof(raw));
    if (hipnuc_input(&raw, 0x5A) != 0) return 1;
    nmea_raw_t nmea = {};
    if (nmea_input(&nmea, '$') != 0) return 1;
    hipnuc_sample_t sample;
    hipnuc_sample_clear(&sample);
    char json[64];
    if (hipnuc_json_sample(&sample, json, sizeof(json)) <= 0) return 1;

    hipnuc_can_frame_t frame;
    std::memset(&frame, 0, sizeof(frame));
    if (hipnuc_j1939_parse(&frame, &sample, nullptr) != HIPNUC_J1939_MSG_NONE) return 1;

    std::printf("consume_cpp: ok %s\n", json);
    return 0;
}
