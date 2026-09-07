/* stream read: print every decoded J1939 / CANFD83 frame of the target
 * nodes as one JSON object per line. */
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
#include "hipnuc_j1939.h"
#include "hipnuc_json.h"

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

int cmd_read(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        help_print_arg_error_json("stream read", "usage: stream read");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    uint8_t target_nodes[CONFIG_MAX_NODES];
    int target_count = config_get_target_nodes(target_nodes, CONFIG_MAX_NODES);
    if (target_count <= 0) {
        help_print_arg_error_json("stream read", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *ifname = config_get_interface();
    int sockfd = can_open_socket(ifname);
    if (sockfd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    log_info("Reading J1939 frames on %s", ifname);

    int rc = CANHOST_EXIT_OK;
    while (running) {
        can_rx_frame_t rx;
        int result = can_receive_frame(sockfd, &rx);
        if (result < 0) {
            log_error("Failed to receive CAN frame");
            rc = CANHOST_EXIT_RUNTIME_ERROR;
            break;
        }
        if (result == 0) {
            continue;
        }

        hipnuc_sample_t part;
        if (hipnuc_j1939_parse(&rx.frame, &part, NULL) <= 0) {
            continue;
        }
        bool match = false;
        for (int i = 0; i < target_count; ++i) {
            if (part.node_id == target_nodes[i]) {
                match = true;
                break;
            }
        }
        if (!match) {
            continue;
        }

        char json[1024];
        if (hipnuc_json_sample(&part, json, sizeof(json)) > 0) {
            printf("%s\n", json);
            fflush(stdout);
        }
    }

    log_info("Stopping");
    can_close_socket(sockfd);
    return rc;
}
