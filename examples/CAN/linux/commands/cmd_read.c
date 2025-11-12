// cmd_read.c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <sys/time.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "hipnuc_can_parser.h"
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
    const char *json_path = NULL;
    FILE *json_fp = NULL;
    enum { MAX_MSG_TYPES = 32, JSON_CACHE_LEN = 512 };
    char last_json[MAX_MSG_TYPES][JSON_CACHE_LEN];
    bool has_json[MAX_MSG_TYPES];
    for (int i = 0; i < MAX_MSG_TYPES; ++i) has_json[i] = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--json-file") == 0) {
            if (i + 1 < argc) {
                json_path = argv[i + 1];
                i++;
            } else {
                log_error("Missing file path for %s", argv[i]);
                return -1;
            }
        }
    }
    if (json_path) {
        json_fp = fopen(json_path, "w");
        if (!json_fp) {
            log_error("Failed to open %s", json_path);
            return -1;
        }
        log_info("Recording JSON to %s", json_path);
    }
    
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
            
            // Parse CAN frame
            can_sensor_data_t data = {0};
            int msg_type = hipnuc_can_parse_frame(&hipnuc_frame, &data);
            
            // Convert to JSON and output
            if (msg_type != CAN_MSG_UNKNOWN && msg_type != CAN_MSG_ERROR) {
                can_json_output_t json_output;
                if (hipnuc_can_to_json(&data, msg_type, &json_output) > 0) {
                    unsigned long long t = now_ms();
                    if (msg_type >= 0 && msg_type < MAX_MSG_TYPES) {
                        snprintf(last_json[msg_type], JSON_CACHE_LEN, "%s", json_output.buffer);
                        has_json[msg_type] = true;
                    }
                    if (json_fp) {
                        fputs(json_output.buffer, json_fp);
                        fflush(json_fp);
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
    if (json_fp) fclose(json_fp);
    return 0;
}
