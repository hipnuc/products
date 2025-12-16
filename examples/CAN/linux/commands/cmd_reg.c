#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/can.h>

#include "../can_interface.h"
#include "../log.h"
#include "../help.h"
#include "../utils.h"
#include "../config.h"
#include "hipnuc_can_common.h"
#include "j1939_reg_api.h"

typedef struct {
    int do_read;
    int do_write;
    uint16_t addr;
    uint32_t value;
    uint8_t node_id;
    uint8_t host_sa;
} reg_args_t;

static int parse_args(int argc, char **argv, reg_args_t *out)
{
    memset(out, 0, sizeof(*out));
    out->node_id = config_get_sync_node();
    out->host_sa = config_get_sync_sa();

    if (argc >= 3 && strcmp(argv[1], "read") == 0) {
        out->do_read = 1;
        out->addr = (uint16_t)strtoul(argv[2], NULL, 0);
        // optional: parse trailing -n/--node
        for (int i = 3; i < argc; ++i) {
            if ((strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--node") == 0) && i + 1 < argc) {
                out->node_id = (uint8_t)strtoul(argv[++i], NULL, 0);
            }
        }
        return 0;
    }

    if (argc >= 4 && strcmp(argv[1], "write") == 0) {
        out->do_write = 1;
        out->addr = (uint16_t)strtoul(argv[2], NULL, 0);
        out->value = (uint32_t)strtoul(argv[3], NULL, 0);
        // optional: parse trailing -n/--node
        for (int i = 4; i < argc; ++i) {
            if ((strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--node") == 0) && i + 1 < argc) {
                out->node_id = (uint8_t)strtoul(argv[++i], NULL, 0);
            }
        }
        return 0;
    }

    return -1;
}

 

int cmd_reg(int argc, char *argv[])
{
    reg_args_t args;
    if (parse_args(argc, argv, &args) < 0) {
        log_error("Usage: canhost reg read <addr> [-n node] | write <addr> <value> [-n node]");
        return -1;
    }

    const char *ifname = config_get_interface();
    int fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        return -1;
    }

    if (args.do_read) {
        j1939_reg_result_t res;
        int got = j1939_reg_read(fd, args.node_id, args.host_sa, args.addr, 1000, &res);
        if (got < 0) {
            log_error("Receive failed");
            can_close_socket(fd);
            return -1;
        }
        if (got == 0) {
            log_error("Timeout waiting for read response");
            can_close_socket(fd);
            return -1;
        }
        uint16_t u16 = (uint16_t)(res.value & 0xFFFF);
        printf("addr=0x%04X val=%u(0x%04X)\n", args.addr, (unsigned)u16, (unsigned)u16);

    } else if (args.do_write) {
        j1939_reg_result_t res;
        int got = j1939_reg_write(fd, args.node_id, args.host_sa, args.addr, args.value, 1000, &res);
        if (got <= 0) {
            log_error("No write ack (timeout)");
            can_close_socket(fd);
            return -1;
        }
        printf("addr=0x%04X, val=0x%X, write_ack status=%u echo_val=0x%08X\n", args.addr, args.value, (unsigned)res.status, (unsigned)res.value);
    }

    can_close_socket(fd);
    return 0;
}
