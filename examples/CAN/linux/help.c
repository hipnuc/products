#include <stdio.h>
#include <string.h>

#include "help.h"

#define PROGRAM_NAME "canhost"
#define VERSION "2.0.0"

static const char *examples[] = {
    PROGRAM_NAME " device list",
    PROGRAM_NAME " device probe",
    PROGRAM_NAME " stream read",
    PROGRAM_NAME " stream record -o imu.jsonl",
    PROGRAM_NAME " trigger sync --count 1",
    PROGRAM_NAME " trigger sync --pgn 0xff34 --count 10",
    PROGRAM_NAME " config reg read 0x70",
    PROGRAM_NAME " config reg write 0x06 1",
    PROGRAM_NAME " action run reset --yes"
};

void help_print_version(void)
{
    printf("%s %s - HiPNUC CAN host utility\n", PROGRAM_NAME, VERSION);
    printf("Copyright (C) 2026 HiPNUC\n");
    printf("Website: https://www.hipnuc.com\n");
}

void help_print_usage(const char *program_name)
{
    const char *pn = program_name && *program_name ? program_name : PROGRAM_NAME;
    help_print_version();
    printf("Usage: %s [OPTIONS] <group> <command> [args]\n", pn);
    printf("Options:\n");
    printf("  -h, --help              Show this help\n");
    printf("  -v, --version           Show version information\n");
    printf("  -n, --node ID[,ID...]   Override target node list from config\n");
    printf("Groups:\n");
    printf("  device list             Show detected CAN interfaces\n");
    printf("  device probe            Probe online J1939 devices\n");
    printf("  stream read             Print parsed JSON stream to stdout\n");
    printf("  stream record -o FILE   Record parsed NDJSON to file\n");
    printf("  trigger sync            Send periodic sync trigger frames\n");
    printf("    --count N             Send each configured PGN N times then exit\n");
    printf("    --pgn PGN             Only trigger one PGN (hex or decimal)\n");
    printf("  config reg read ADDR    Read register via J1939 config frame\n");
    printf("  config reg write ADDR V Write register via J1939 config frame\n");
    printf("  action run NAME --yes   Execute predefined action sequence\n");
    printf("  help                    Show this help\n");
    printf("  version                 Show version\n");
    printf("Examples:\n");
    for (size_t i = 0; i < sizeof(examples)/sizeof(examples[0]); ++i) {
        printf("  %s\n", examples[i]);
    }
    printf("\nCAN setup quick-start:\n");
    printf("  sudo ip link set can0 down\n");
    printf("  sudo ip link set can0 type can bitrate <125000|250000|500000|1000000>\n");
    printf("  sudo ip link set can0 up\n");
    printf("  ip -details link show can0\n");
    printf("Config search order:\n");
    printf("  $CANHOST_CONF | ./canhost.ini | ~/.canhost.ini | /etc/canhost.ini\n");
}

void help_print_unknown_command(const char *program_name)
{
    const char *pn = program_name && *program_name ? program_name : PROGRAM_NAME;
    fprintf(stderr, "Unknown command.\n");
    fprintf(stderr, "Run '%s help' for usage.\n", pn);
}

void help_print_can_setup(const char *ifname)
{
    const char *iface = (ifname && *ifname) ? ifname : "can0";
    fprintf(stderr, "Error: cannot open CAN interface '%s'\n", ifname ? ifname : "(null)");
    fprintf(stderr, "Quick setup (classical CAN):\n");
    fprintf(stderr, "  sudo ip link set %s down\n", iface);
    fprintf(stderr, "  sudo ip link set %s type can bitrate <125000|250000|500000|1000000>\n", iface);
    fprintf(stderr, "  sudo ip link set %s up\n", iface);
    fprintf(stderr, "  ip -details link show %s\n", iface);
}

void help_print_no_interfaces_hint(void)
{
    printf("\nNo physical CAN interfaces detected.\n");
    printf("Quick checklist:\n");
    printf("  sudo ip link set can0 down\n");
    printf("  sudo ip link set can0 type can bitrate <125000|250000|500000|1000000>\n");
    printf("  sudo ip link set can0 up\n");
    printf("  ip -details link show can0\n");
}

void help_print_arg_error_json(const char *command, const char *message)
{
    const char *cmd = (command && *command) ? command : "unknown";
    const char *msg = (message && *message) ? message : "invalid arguments";
    fprintf(stderr,
            "{\"error\":{\"type\":\"invalid_args\",\"command\":\"%s\",\"message\":\"%s\"}}\n",
            cmd, msg);
}

