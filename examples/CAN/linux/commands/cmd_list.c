// cmd_list.c
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../can_interface.h"
#include "../log.h"
#include "../help.h"
#include "../commands.h"

// bitrate column removed for a simpler list output

static void print_table_header(void)
{
    printf("  %-8s %-8s\n", "IFACE", "STATE");
    printf("  %-8s %-8s\n", "------", "------");
}

static void print_interface_row(const can_interface_info_t *iface)
{
    const char *state = iface->state[0] ? iface->state : "unknown";
    printf("  %-8s %-8s\n",
           iface->name,
           state);
}

static void print_setup_hint(void)
{
    help_print_no_interfaces_hint();
}

int cmd_list(int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        help_print_arg_error_json("device list", "usage: device list");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    can_interface_info_t interfaces[16];
    int count = can_list_interfaces(interfaces, (int)(sizeof(interfaces) / sizeof(interfaces[0])));
    if (count < 0) {
        printf("Error: unable to enumerate CAN interfaces (check permissions).\n");
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    if (count == 0) {
        print_setup_hint();
        return CANHOST_EXIT_OK;
    }

    printf("\nDetected CAN interfaces (%d):\n", count);
    print_table_header();
    for (int i = 0; i < count; ++i) {
        print_interface_row(&interfaces[i]);
    }

    printf("\nExamples:\n");
    printf("  canhost stream read\n");
    printf("  canhost device probe\n");
    printf("\n");

    return CANHOST_EXIT_OK;
}
