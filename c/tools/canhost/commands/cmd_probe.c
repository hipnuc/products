/* device probe: broadcast a J1939 Request for ADDRESS_CLAIMED and list the
 * source addresses that answer. */
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

#define J1939_PGN_REQUEST          0xEA00U   /* 59904 */
#define J1939_PGN_ADDRESS_CLAIMED  0xEE00U   /* 60928 */
#define J1939_BROADCAST_ADDRESS    0xFFU
#define J1939_HOST_ADDRESS         0xFEU
#define PROBE_IDLE_TIMEOUT_MS      500
#define MAX_J1939_ADDRESSES        256

static volatile sig_atomic_t keep_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    keep_running = 0;
}

/* 29-bit identifier: priority 3, PDU1 PGNs take the destination in PS. */
static uint32_t j1939_id(uint32_t pgn, uint8_t sa, uint8_t da)
{
    uint32_t pf = (pgn >> 8) & 0xFFU;
    uint32_t ps = pf < 240U ? da : (pgn & 0xFFU);
    return (3U << 26) | ((pgn & 0x30000U) << 8) | (pf << 16) | (ps << 8) | sa;
}

static int send_request_address_claimed(int sockfd)
{
    hipnuc_can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = j1939_id(J1939_PGN_REQUEST, J1939_HOST_ADDRESS, J1939_BROADCAST_ADDRESS);
    f.is_extended = 1;
    f.len = 8;   /* 3 PGN bytes, padded as the devices expect */
    f.data[0] = J1939_PGN_ADDRESS_CLAIMED & 0xFFU;
    f.data[1] = (J1939_PGN_ADDRESS_CLAIMED >> 8) & 0xFFU;
    f.data[2] = (J1939_PGN_ADDRESS_CLAIMED >> 16) & 0xFFU;
    if (can_send_frame(sockfd, &f) < 0) {
        log_error("Failed to send REQUEST for ADDRESS_CLAIMED");
        return -1;
    }
    return 0;
}

static bool parse_address_claimed(const hipnuc_can_frame_t *f, uint8_t *addr, uint64_t *name)
{
    if (!f->is_extended || f->is_remote || f->is_error || f->len != 8) {
        return false;
    }
    if (hipnuc_j1939_pgn(f->id) != J1939_PGN_ADDRESS_CLAIMED) {
        return false;
    }
    *addr = hipnuc_j1939_source_address(f->id);
    *name = 0;
    for (int i = 0; i < 8; ++i) {
        *name |= (uint64_t)f->data[i] << (i * 8);
    }
    return true;
}

int cmd_probe(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        help_print_arg_error_json("device probe", "usage: device probe");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    printf("Probing J1939 devices on %s (idle timeout %d ms), Ctrl+C stops early.\n",
           ifname, PROBE_IDLE_TIMEOUT_MS);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    static bool found[MAX_J1939_ADDRESSES];
    memset(found, 0, sizeof(found));
    int found_count = 0;

    if (send_request_address_claimed(fd) < 0) {
        can_close_socket(fd);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    int frames = 0;
    uint64_t last_claim_ms = utils_now_ms();

    while (keep_running && utils_now_ms() - last_claim_ms <= PROBE_IDLE_TIMEOUT_MS) {
        can_rx_frame_t rx;
        int r = can_receive_frames(fd, &rx, 1, PROBE_IDLE_TIMEOUT_MS);
        if (r < 0) {
            break;
        }
        if (r == 0) {
            continue;
        }
        ++frames;
        uint8_t addr;
        uint64_t name;
        if (parse_address_claimed(&rx.frame, &addr, &name) && !found[addr]) {
            found[addr] = true;
            found_count++;
            log_info("Device: address=%u (0x%02X) name=0x%016llX",
                     (unsigned)addr, (unsigned)addr, (unsigned long long)name);
            last_claim_ms = utils_now_ms();
        }
    }

    printf("Frames received: %d\n", frames);
    if (found_count == 0) {
        log_info("No J1939 device answered");
    } else {
        log_info("J1939 devices found: %d", found_count);
    }
    can_close_socket(fd);
    return CANHOST_EXIT_OK;
}
