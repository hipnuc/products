#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../can_interface.h"
#include "../config.h"
#include "../help.h"
#include "../log.h"
#include "../commands.h"
#include "j1939_reg_api.h"

typedef struct {
    bool do_read;
    bool do_write;
    uint16_t addr;
    uint32_t value;
    uint8_t target_nodes[32];
    int target_count;
    uint8_t host_sa;
} reg_args_t;

static int parse_u32(const char *s, uint32_t *out)
{
    if (!s || !*s || !out) {
        return -1;
    }
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || *end != '\0') {
        return -1;
    }
    *out = (uint32_t)v;
    return 0;
}

static int parse_args(int argc, char **argv, reg_args_t *out)
{
    memset(out, 0, sizeof(*out));
    out->target_count = config_get_target_nodes(out->target_nodes, 32);
    out->host_sa = config_get_sync_sa();

    if (argc == 3 && strcmp(argv[1], "read") == 0) {
        uint32_t addr = 0;
        if (parse_u32(argv[2], &addr) < 0 || addr > 0xFFFFu) {
            return -1;
        }
        out->do_read = true;
        out->addr = (uint16_t)addr;
        return 0;
    }

    if (argc == 4 && strcmp(argv[1], "write") == 0) {
        uint32_t addr = 0;
        uint32_t value = 0;
        if (parse_u32(argv[2], &addr) < 0 || addr > 0xFFFFu) {
            return -1;
        }
        if (parse_u32(argv[3], &value) < 0) {
            return -1;
        }
        out->do_write = true;
        out->addr = (uint16_t)addr;
        out->value = value;
        return 0;
    }

    return -1;
}

int cmd_reg(int argc, char *argv[])
{
    reg_args_t args;
    if (parse_args(argc, argv, &args) < 0) {
        help_print_arg_error_json("config reg", "usage: config reg read <addr> | config reg write <addr> <value>");
        return CANHOST_EXIT_INVALID_ARGS;
    }
    if (args.target_count <= 0) {
        help_print_arg_error_json("config reg", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    int overall = CANHOST_EXIT_OK;

    if (args.do_read) {
        for (int i = 0; i < args.target_count; ++i) {
            uint8_t node = args.target_nodes[i];
            j1939_reg_result_t res;
            if (args.target_count > 1) {
                printf("[Node %d] ", node);
            }

            int got = j1939_reg_read(fd, node, args.host_sa, args.addr, 1000, &res);
            if (got < 0) {
                printf("Receive failed\n");
                overall = CANHOST_EXIT_RUNTIME_ERROR;
            } else if (got == 0) {
                printf("Timeout\n");
                overall = CANHOST_EXIT_RUNTIME_ERROR;
            } else {
                uint16_t u16 = (uint16_t)(res.value & 0xFFFF);
                printf("addr=0x%04X val=%u(0x%04X)\n", args.addr, (unsigned)u16, (unsigned)u16);
            }
        }
    } else if (args.do_write) {
        for (int i = 0; i < args.target_count; ++i) {
            uint8_t node = args.target_nodes[i];
            j1939_reg_result_t res;
            if (args.target_count > 1) {
                printf("[Node %d] ", node);
            }

            int got = j1939_reg_write(fd, node, args.host_sa, args.addr, args.value, 1000, &res);
            if (got <= 0) {
                printf("No write ack (timeout)\n");
                overall = CANHOST_EXIT_RUNTIME_ERROR;
            } else {
                printf("addr=0x%04X, val=0x%X, write_ack status=%u echo_val=0x%08X\n",
                       args.addr, args.value, (unsigned)res.status, (unsigned)res.value);
            }
        }
    }

    can_close_socket(fd);
    return overall;
}
