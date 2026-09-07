/* stream record -o FILE: write decoded J1939 / CANFD83 frames of the target
 * nodes as JSON lines. Each line carries the receive time (rx_time_us,
 * hardware timestamp when the driver provides one). */
#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can_interface.h"
#include "commands.h"
#include "config.h"
#include "help.h"
#include "log.h"
#include "utils.h"
#include "hipnuc_j1939.h"
#include "hipnuc_json.h"

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

static void print_stats(uint64_t rx, uint64_t written, uint64_t dropped, double fps)
{
    printf("\rrx=%llu written=%llu dropped=%llu fps=%.1f   ",
           (unsigned long long)rx, (unsigned long long)written,
           (unsigned long long)dropped, fps);
    fflush(stdout);
}

int cmd_record(int argc, char *argv[])
{
    const char *out_path = NULL;
    uint8_t target_nodes[CONFIG_MAX_NODES];
    int target_count = config_get_target_nodes(target_nodes, CONFIG_MAX_NODES);

    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--out") == 0) && i + 1 < argc) {
            out_path = argv[++i];
        } else {
            help_print_arg_error_json("stream record", "usage: stream record -o <file>");
            return CANHOST_EXIT_INVALID_ARGS;
        }
    }
    if (!out_path) {
        help_print_arg_error_json("stream record", "missing output file (-o FILE)");
        return CANHOST_EXIT_INVALID_ARGS;
    }
    if (target_count <= 0) {
        help_print_arg_error_json("stream record", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    FILE *file = fopen(out_path, "w");
    if (!file) {
        log_error("Failed to open %s: %s", out_path, strerror(errno));
        can_close_socket(sockfd);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }
    setvbuf(file, NULL, _IOFBF, 1024 * 1024);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    log_info("Recording JSON lines from %s to %s", ifname, out_path);

    static can_rx_frame_t frames[256];
    const size_t batch_cap = sizeof(frames) / sizeof(frames[0]);

    uint64_t rx_frames = 0, dropped_frames = 0, written_frames = 0, last_print_rx = 0;
    uint64_t last_print_ms = utils_now_ms();
    int rc = CANHOST_EXIT_OK;

    while (g_running) {
        int r = can_receive_frames(sockfd, frames, batch_cap, 100);
        if (r < 0) {
            log_error("Receive error");
            rc = CANHOST_EXIT_RUNTIME_ERROR;
            break;
        }

        for (int i = 0; i < r; ++i) {
            hipnuc_sample_t part;
            int type = hipnuc_j1939_parse(&frames[i].frame, &part, NULL);
            if (type == 0) {
                continue;               /* not HiPNUC data */
            }
            if (type < 0) {
                dropped_frames++;       /* HiPNUC PGN with a bad payload */
                continue;
            }
            bool match = false;
            for (int k = 0; k < target_count; ++k) {
                if (part.node_id == target_nodes[k]) {
                    match = true;
                    break;
                }
            }
            if (!match) {
                continue;
            }

            char json[1024];
            if (hipnuc_json_sample(&part, json, sizeof(json)) <= 0) {
                dropped_frames++;
                continue;
            }
            /* Insert the receive time as the first member. */
            fprintf(file, "{\"rx_time_us\":%llu,%s\n",
                    (unsigned long long)frames[i].timestamp_us, json + 1);
            written_frames++;
        }
        rx_frames += (uint64_t)r;

        uint64_t now = utils_now_ms();
        if (now - last_print_ms >= 1000) {
            double fps = (double)(rx_frames - last_print_rx) * 1000.0 / (double)(now - last_print_ms);
            print_stats(rx_frames, written_frames, dropped_frames, fps);
            last_print_rx = rx_frames;
            last_print_ms = now;
        }
    }

    printf("\n");
    log_info("Recorded %llu samples (%llu dropped)",
             (unsigned long long)written_frames, (unsigned long long)dropped_frames);

    if (fclose(file) != 0) {
        log_error("Failed to write %s: %s", out_path, strerror(errno));
        rc = CANHOST_EXIT_RUNTIME_ERROR;
    }
    can_close_socket(sockfd);
    return rc;
}
