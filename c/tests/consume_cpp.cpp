// A C++ application including every public header and linking the targets.
#include <cstdio>
#include <cstring>

#include "hipnuc_can_frame.h"
#include "hipnuc_can_update.h"
#include "hipnuc_dec.h"
#include "hipnuc_j1939.h"
#include "hipnuc_json.h"
#include "hipnuc_kboot.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"

int main()
{
    hipnuc_raw_t raw;
    std::memset(&raw, 0, sizeof(raw));
    hipnuc_sample_t sample;
    hipnuc_sample_clear(&sample);
    char json[64];
    if (hipnuc_json_sample(&sample, json, sizeof(json)) <= 0) return 1;

    hipnuc_can_frame_t frame;
    std::memset(&frame, 0, sizeof(frame));
    if (hipnuc_j1939_parse(&frame, &sample, nullptr) != HIPNUC_J1939_MSG_NONE) return 1;

    hipnuc_kboot_ctx_t kboot;
    hipnuc_kboot_init(&kboot, nullptr);
    if (hipnuc_kboot_ping(&kboot) != HIPNUC_KBOOT_ERR_PARAM) return 1;

    hipnuc_can_update_ctx_t can;
    hipnuc_can_update_init(&can, nullptr);
    if (hipnuc_can_update_connect(&can, 8) != HIPNUC_CAN_UPDATE_ERR_PARAM) return 1;

    std::printf("consume_cpp: ok %s\n", json);
    return 0;
}
