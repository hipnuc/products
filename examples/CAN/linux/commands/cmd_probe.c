#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <linux/can.h>

#include "../can_interface.h"
#include "../log.h"
#include "../utils.h"
#include "../help.h"
#include "../commands.h"
#include "../config.h"

#define J1939_PGN_REQUEST_PGN         59904
#define J1939_PGN_ADDRESS_CLAIMED     60928
#define J1939_BROADCAST_ADDRESS       255
#define J1939_HOST_ADDRESS            254
#define J1939_PDU1_FORMAT_CUTOFF      240
#define PROBE_WINDOW_SECONDS          2
#define MAX_J1939_ADDRESSES           256

typedef union {
    uint32_t val;
    struct {
        uint8_t source_address;
        uint8_t pdu_specific;
        uint8_t pdu_format;
        uint8_t data_page:1;
        uint8_t extended_data_page:1;
        uint8_t priority:3;
        uint8_t reserved:3;
    } bit;
} j1939_can_id_t;

typedef struct {
    uint8_t address;
    uint64_t name;
    bool found;
} discovered_device_t;

static volatile sig_atomic_t keep_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    keep_running = 0;
}

static uint32_t j1939_pgn_to_can_id(uint16_t pgn, uint8_t sa, uint8_t da)
{
    j1939_can_id_t id = {0};
    id.bit.priority = 3;
    id.bit.data_page = (pgn >> 16) & 1;
    id.bit.extended_data_page = (pgn >> 17) & 1;
    id.bit.pdu_format = (pgn >> 8) & 0xFF;
    id.bit.pdu_specific = (id.bit.pdu_format < J1939_PDU1_FORMAT_CUTOFF) ? da : (pgn & 0xFF);
    id.bit.source_address = sa;
    return id.val;
}

static uint16_t j1939_can_id_to_pgn(uint32_t can_id)
{
    j1939_can_id_t id = {.val = can_id};
    uint16_t pgn = ((id.bit.data_page & 1) << 16) |
                   ((id.bit.extended_data_page & 1) << 17) |
                   (id.bit.pdu_format << 8);
    if (id.bit.pdu_format >= J1939_PDU1_FORMAT_CUTOFF)
        pgn |= id.bit.pdu_specific;
    return pgn;
}

static uint8_t j1939_can_id_to_sa(uint32_t can_id)
{
    return ((j1939_can_id_t){.val = can_id}).bit.source_address;
}

static int send_request_address_claimed(int sockfd)
{
    struct can_frame f = {
        .can_id = j1939_pgn_to_can_id(J1939_PGN_REQUEST_PGN, J1939_HOST_ADDRESS, J1939_BROADCAST_ADDRESS) | CAN_EFF_FLAG,
        .can_dlc = 8
    };
    f.data[0] = J1939_PGN_ADDRESS_CLAIMED & 0xFF;
    f.data[1] = (J1939_PGN_ADDRESS_CLAIMED >> 8) & 0xFF;
    f.data[2] = (J1939_PGN_ADDRESS_CLAIMED >> 16) & 0xFF;
    memset(&f.data[3], 0, 5);

    if (write(sockfd, &f, sizeof(f)) != sizeof(f)) {
        log_error("Failed to send REQUEST_PGN");
        return -1;
    }
    log_info("Sent REQUEST_PGN for ADDRESS_CLAIMED (PGN=0x%04X)", J1939_PGN_ADDRESS_CLAIMED);
    return 0;
}

static bool parse_address_claimed(const struct can_frame *f, uint8_t *addr, uint64_t *name)
{
    if (!(f->can_id & CAN_EFF_FLAG) || f->can_dlc != 8)
        return false;
    uint32_t id = f->can_id & CAN_EFF_MASK;
    if (j1939_can_id_to_pgn(id) != J1939_PGN_ADDRESS_CLAIMED)
        return false;
    *addr = j1939_can_id_to_sa(id);
    *name = 0;
    for (int i = 0; i < 8; ++i)
        *name |= (uint64_t)f->data[i] << (i * 8);
    return true;
}

static void record_device(discovered_device_t *devices, uint8_t addr, uint64_t name)
{
    discovered_device_t *slot = &devices[addr];
    slot->address = addr;
    slot->name = name;
    slot->found = true;
}
static void display_devices(const discovered_device_t *devices, size_t len)
{
    // 为每一个发现的设备单独输出一条 INFO 日志
    int cnt = 0;
    for (size_t i = 0; i < len; ++i) {
        if (devices[i].found) {
            log_info("Discovered J1939 device: addr=%u name=0x%016llX",
                     (unsigned int)devices[i].address,
                     (unsigned long long)devices[i].name);
            ++cnt;
        }
    }

    if (cnt == 0) {
        log_info("No J1939 devices discovered");
    }
}
static void update_progress(time_t now, time_t end, int *last_remaining)
{
    int remaining = (int)(end - now);
    if (remaining < 0) remaining = 0;

    if (remaining != *last_remaining) {
        printf("\rScanning ... %ds left ", remaining);
        fflush(stdout);
        *last_remaining = remaining;
    }
}

int cmd_probe(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Interface readiness already validated by the dispatcher (commands.c)
    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }

    printf("Probing J1939 devices on %s (window: %ds)\n", ifname, PROBE_WINDOW_SECONDS);
    printf("Press Ctrl+C to stop early.\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    keep_running = 1;

    discovered_device_t devices[MAX_J1939_ADDRESSES] = {0};

    if (send_request_address_claimed(fd) < 0) {
        can_close_socket(fd);
        return -1;
    }

    time_t end = time(NULL) + PROBE_WINDOW_SECONDS;
    int frames = 0;
    int last_remaining = -1;

    while (keep_running) {
        time_t now = time(NULL);
        if (now >= end) {
            break;
        }

        update_progress(now, end, &last_remaining);

        struct can_frame f;
        uint64_t hw_ts_us = 0;
        int r = can_receive_frame_ts(fd, &f, &hw_ts_us);
        if (r > 0) {
            uint8_t addr; uint64_t name;
            if (parse_address_claimed(&f, &addr, &name)) {
                record_device(devices, addr, name);
            }
            ++frames;
        } else if (r < 0) {
            break;
        }
    }

    printf("\rScanning ... done          \n");
    printf("Frames received: %d\n", frames);
    display_devices(devices, MAX_J1939_ADDRESSES);
    can_close_socket(fd);
    keep_running = 1;
    return 0;
}
