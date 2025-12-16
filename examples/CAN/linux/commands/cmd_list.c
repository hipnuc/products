// cmd_list.c
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../can_interface.h"
#include "../log.h"
#include "../help.h"

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
    (void)argc;
    (void)argv;

    can_interface_info_t interfaces[16];
    int count = can_list_interfaces(interfaces, (int)(sizeof(interfaces) / sizeof(interfaces[0])));
    if (count < 0) {
        printf("Error: unable to enumerate CAN interfaces (check permissions).\n");
        return -1;
    }

    if (count == 0) {
        print_setup_hint();
        return 0;
    }

    printf("\nDetected CAN interfaces (%d):\n", count);
    print_table_header();
    for (int i = 0; i < count; ++i) {
        print_interface_row(&interfaces[i]);
    }

    printf("\nExamples:\n");
    printf("  canhost read\n");
    printf("  canhost probe\n");
    printf("\n");

    return 0;
}
