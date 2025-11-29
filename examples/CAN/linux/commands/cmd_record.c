#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "../help.h"
#include "../config.h"
#include "hipnuc_can_common.h"
#include "hipnuc_j1939_parser.h"
#include "canopen_parser.h"

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

int cmd_record(int argc, char *argv[])
{
    const char *out_path = NULL;
    
    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--out") == 0) && i + 1 < argc) {
            out_path = argv[++i];
        }
    }
    
    if (!out_path) {
        log_error("Missing output file (-o FILE)");
        return -1;
    }

    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }

    FILE *file = fopen(out_path, "w");
    if (!file) {
        log_error("Failed to open %s: %s", out_path, strerror(errno));
        can_close_socket(sockfd);
        return -1;
    }
    
    // Set large buffer for file I/O
    setvbuf(file, NULL, _IOFBF, 1024 * 1024);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    log_info("Recording JSON on %s -> %s", ifname, out_path);

    struct can_frame frames[256];
    uint64_t ts_us[256];
    size_t batch_cap = sizeof(frames) / sizeof(frames[0]);

    uint64_t rx_frames = 0;
    uint64_t dropped_frames = 0;
    uint64_t written_frames = 0;
    uint64_t last_print_rx = 0;
    uint32_t last_print_ms = utils_get_timestamp_ms();

    while (g_running) {
        int r = can_receive_frames_ts(sockfd, frames, ts_us, batch_cap, 100);

        if (r < 0) {
            log_error("Receive error");
            break;
        }

        if (r == 0) {
            uint32_t now_ms = utils_get_timestamp_ms();
            if (now_ms - last_print_ms >= 1000) {
                uint64_t delta_rx = rx_frames - last_print_rx;
                double fps = (double)delta_rx * 1000.0 / (double)(now_ms - last_print_ms);
                printf("\rrx=%llu written=%llu dropped=%llu fps=%.1f   ",
                       (unsigned long long)rx_frames,
                       (unsigned long long)written_frames,
                       (unsigned long long)dropped_frames,
                       fps);
                fflush(stdout);
                last_print_rx = rx_frames;
                last_print_ms = now_ms;
            }
            continue;
        }

        for (int i = 0; i < r; ++i) {
            hipnuc_can_frame_t hipnuc_frame;
            utils_linux_can_to_hipnuc_can(&frames[i], ts_us[i], &hipnuc_frame);

            can_sensor_data_t data;
            memset(&data, 0, sizeof(data));
            data.node_id = hipnuc_can_extract_node_id(hipnuc_frame.can_id);
            data.hw_ts_us = hipnuc_frame.hw_ts_us;

            int msg_type = (hipnuc_frame.can_id & HIPNUC_CAN_EFF_FLAG)
                         ? hipnuc_j1939_parse_frame(&hipnuc_frame, &data)
                         : canopen_parse_frame(&hipnuc_frame, &data);

            if (msg_type == CAN_MSG_UNKNOWN || msg_type == CAN_MSG_ERROR) {
                dropped_frames++;
                continue;
            }

            can_json_output_t json;
            if (hipnuc_can_to_json(&data, msg_type, &json) > 0) {
                fwrite(json.buffer, 1, json.length, file);
                written_frames++;
            }
        }
        rx_frames += r;

        uint32_t now_ms = utils_get_timestamp_ms();
        if (now_ms - last_print_ms >= 1000) {
            uint64_t delta_rx = rx_frames - last_print_rx;
            double fps = (double)delta_rx * 1000.0 / (double)(now_ms - last_print_ms);
            printf("\rrx=%llu written=%llu dropped=%llu fps=%.1f   ",
                   (unsigned long long)rx_frames,
                   (unsigned long long)written_frames,
                   (unsigned long long)dropped_frames,
                   fps);
            fflush(stdout);
            last_print_rx = rx_frames;
            last_print_ms = now_ms;
        }
    }

    printf("\n");
    log_info("Recorded %llu frames (%llu dropped)",
             (unsigned long long)written_frames,
             (unsigned long long)dropped_frames);

    fflush(file);
    fclose(file);
    can_close_socket(sockfd);
    return 0;
}
