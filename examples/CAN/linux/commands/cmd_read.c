// cmd_read.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/time.h>
#include <stdint.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "hipnuc_can_common.h"
#include "hipnuc_j1939_parser.h"
#include "canopen_parser.h"
#include "../config.h"

static volatile bool running = true;

static void signal_handler(int sig)
{
    (void)sig;
    running = false;
}

static unsigned long long now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long long)tv.tv_sec * 1000ULL + (unsigned long long)tv.tv_usec / 1000ULL;
}

int cmd_read(int argc, char *argv[])
{
    (void)argc; (void)argv;
    enum { MAX_MSG_TYPES = 32, JSON_CACHE_LEN = 512 };
    char last_json[MAX_MSG_TYPES][JSON_CACHE_LEN];
    bool has_json[MAX_MSG_TYPES];
    for (int i = 0; i < MAX_MSG_TYPES; ++i) has_json[i] = false;
    
    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        utils_print_can_setup_hint(ifname);
        return -1;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    log_info("Starting CAN read on interface: %s", ifname);
    unsigned long long last_print = 0ULL;
    
    while (running) {
        struct can_frame frame;
        uint64_t hw_ts_us = 0;
        int result = can_receive_frame_ts(sockfd, &frame, &hw_ts_us);

        if (result > 0) {
            // Convert Linux CAN frame to hipnuc CAN frame
            hipnuc_can_frame_t hipnuc_frame;
            utils_linux_can_to_hipnuc_can(&frame, hw_ts_us, &hipnuc_frame);
            
            can_sensor_data_t data = {0};
            data.node_id = hipnuc_can_extract_node_id(hipnuc_frame.can_id);
            data.hw_ts_us = hipnuc_frame.hw_ts_us;
            int msg_type = (hipnuc_frame.can_id & HIPNUC_CAN_EFF_FLAG)
                         ? hipnuc_j1939_parse_frame(&hipnuc_frame, &data)
                         : canopen_parse_frame(&hipnuc_frame, &data);
            
            // Convert to JSON and output
            if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
                can_json_output_t json_output;
                if (hipnuc_can_to_json(&data, msg_type, &json_output) > 0) {
                    unsigned long long t = now_ms();
                    if (msg_type >= 0 && msg_type < MAX_MSG_TYPES) {
                        snprintf(last_json[msg_type], JSON_CACHE_LEN, "%s", json_output.buffer);
                        has_json[msg_type] = true;
                    }
                    if (t - last_print >= 50ULL) {
                        for (int i = 0; i < MAX_MSG_TYPES; ++i) {
                            if (has_json[i]) {
                                fputs(last_json[i], stdout);
                            }
                        }
                        fflush(stdout);
                        last_print = t;
                    }
                }
            }
        } else if (result < 0) {
            log_error("Failed to receive CAN frame");
            break;
        }
    }
    
    log_info("Stopping CAN read");
    can_close_socket(sockfd);
    return 0;
}
