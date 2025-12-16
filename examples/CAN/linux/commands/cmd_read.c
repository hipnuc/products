#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "../help.h"
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

int cmd_read(int argc, char *argv[])
{
    (void)argc; (void)argv;
    
    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    log_info("Starting CAN read on interface: %s", ifname);
    
    while (running) {
        struct can_frame frame;
        uint64_t hw_ts_us = 0;
        int result = can_receive_frame_ts(sockfd, &frame, &hw_ts_us);

        if (result > 0) {
            hipnuc_can_frame_t hipnuc_frame;
            utils_linux_can_to_hipnuc_can(&frame, hw_ts_us, &hipnuc_frame);
            
            can_sensor_data_t data = {0};
            data.node_id = hipnuc_can_extract_node_id(hipnuc_frame.can_id);
            data.hw_ts_us = hipnuc_frame.hw_ts_us;
            int msg_type = (hipnuc_frame.can_id & HIPNUC_CAN_EFF_FLAG)
                         ? hipnuc_j1939_parse_frame(&hipnuc_frame, &data)
                         : canopen_parse_frame(&hipnuc_frame, &data);
            
            if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
                can_json_output_t json_output;
                if (hipnuc_can_to_json(&data, msg_type, &json_output) > 0) {
                    fwrite(json_output.buffer, 1, json_output.length, stdout);
                    fflush(stdout);
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
