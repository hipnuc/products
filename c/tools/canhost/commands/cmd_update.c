/* firmware update -f FILE [--bin] [--continue]: flash every target node
 * through the CAN bootloader (SDO transfers, see hipnuc_can_update.h). */
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
#include "hexfile.h"
#include "hipnuc_can_update.h"

typedef struct {
    int fd;
} update_port_ctx_t;

typedef struct {
    const char *file_path;
    bool raw_binary;
    bool continue_on_error;
} update_args_t;

static int update_send(void *user, const hipnuc_can_frame_t *frame)
{
    update_port_ctx_t *ctx = (update_port_ctx_t *)user;
    return can_send_frame(ctx->fd, frame);
}

/* Wait for a standard frame with exactly `id`; measurement traffic and
 * replies of other nodes are skipped. */
static int update_wait(void *user, uint32_t id, hipnuc_can_frame_t *frame, uint32_t timeout_ms)
{
    update_port_ctx_t *ctx = (update_port_ctx_t *)user;
    uint64_t deadline = utils_now_ms() + timeout_ms;

    for (;;) {
        uint64_t now = utils_now_ms();
        if (now > deadline) {
            return -1;
        }
        can_rx_frame_t rx;
        int ret = can_receive_frames(ctx->fd, &rx, 1, (int)(deadline - now));
        if (ret < 0) {
            return -1;
        }
        if (ret == 0) {
            continue;
        }
        if (rx.frame.is_extended || rx.frame.is_remote || rx.frame.is_error || rx.frame.id != id) {
            continue;
        }
        *frame = rx.frame;
        return 0;
    }
}

static void update_delay(void *user, uint32_t delay_ms)
{
    (void)user;
    utils_delay_ms(delay_ms);
}

static void update_progress(void *user, uint8_t node_id, uint8_t percent)
{
    (void)user;
    printf("\r[Node %u] %3u%%", (unsigned)node_id, (unsigned)percent);
    if (percent >= 100U) {
        printf("\n");
    }
    fflush(stdout);
}

static void update_log(void *user, uint8_t node_id, const char *msg)
{
    (void)user;
    log_info("[Node %u] %s", (unsigned)node_id, msg ? msg : "");
}

static int parse_args(int argc, char **argv, update_args_t *args)
{
    memset(args, 0, sizeof(*args));
    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0) && i + 1 < argc) {
            args->file_path = argv[++i];
        } else if (strcmp(argv[i], "--bin") == 0) {
            args->raw_binary = true;
        } else if (strcmp(argv[i], "--hex") == 0) {
            args->raw_binary = false;
        } else if (strcmp(argv[i], "--continue") == 0) {
            args->continue_on_error = true;
        } else {
            return -1;
        }
    }
    return args->file_path ? 0 : -1;
}

int cmd_update(int argc, char *argv[])
{
    update_args_t args;
    hex_image_t image;
    uint8_t nodes[CONFIG_MAX_NODES];
    int overall = CANHOST_EXIT_OK;

    if (parse_args(argc, argv, &args) < 0) {
        help_print_arg_error_json("firmware update",
                                  "usage: firmware update -f <file.hex> [--bin] [--continue]");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    int node_count = config_get_target_nodes(nodes, CONFIG_MAX_NODES);
    if (node_count <= 0) {
        help_print_arg_error_json("firmware update", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    int loaded = args.raw_binary ? binfile_load(args.file_path, &image)
                                 : hexfile_load(args.file_path, &image);
    if (loaded < 0) {
        return CANHOST_EXIT_RUNTIME_ERROR;
    }
    log_info("Firmware %s: %u bytes, start 0x%08X",
             args.file_path, (unsigned)image.size, (unsigned)image.start_addr);

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        hex_image_free(&image);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    update_port_ctx_t port_ctx = { fd };
    hipnuc_can_update_port_t port = {
        .send = update_send,
        .wait = update_wait,
        .delay_ms = update_delay,
        .progress = update_progress,
        .log = update_log,
        .user = &port_ctx
    };
    hipnuc_can_update_ctx_t updater;
    hipnuc_can_update_init(&updater, &port);

    for (int i = 0; i < node_count; ++i) {
        int ret = hipnuc_can_update_node(&updater, nodes[i], image.data, image.size);
        if (ret != HIPNUC_CAN_UPDATE_OK) {
            printf("\n");
            log_error("[Node %u] firmware update failed: %s", (unsigned)nodes[i],
                      hipnuc_can_update_strerror(ret));
            overall = CANHOST_EXIT_RUNTIME_ERROR;
            if (!args.continue_on_error) {
                break;
            }
        }
    }

    can_close_socket(fd);
    hex_image_free(&image);
    return overall;
}
