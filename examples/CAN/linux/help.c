#include <stdio.h>
#include <string.h>

#include "help.h"

#define PROGRAM_NAME "canhost"
#define VERSION "1.0.2"

static const char *examples[] = {
    PROGRAM_NAME " list",
    PROGRAM_NAME " probe",
    PROGRAM_NAME " read",
    PROGRAM_NAME " record -o imu.json",
    PROGRAM_NAME " sync",
    PROGRAM_NAME " sync -c 1"
};

void help_print_version(void)
{
    printf("%s %s - HiPNUC CAN evaluation utility\n", PROGRAM_NAME, VERSION);
    printf("Copyright (C) 2025 HiPNUC\n");
    printf("Website: https://www.hipnuc.com\n");
}

void help_print_usage(const char *program_name)
{
    const char *pn = program_name && *program_name ? program_name : PROGRAM_NAME;
    help_print_version();
    printf("Usage: %s [OPTIONS] <command> [command options]\n", pn);
    printf("Options:\n");
    printf("  -h, --help              Show this help\n");
    printf("  -v, --version           Show version information\n");
    printf("Commands:\n");
    printf("  list    Show detected CAN interfaces with state\n");
    printf("  probe   Send a 2 s probe and print responding nodes\n");
    printf("  read    Stream and format HiPNUC sensor data frames\n");
    printf("  record  Record parsed JSON (NDJSON) to file, print frames/fps\n");
    printf("  sync    Periodically trigger data via J1939 config PGN sync\n");
    printf("Sync options:\n");
    printf("  -c, --count N          Trigger each configured PGN N times then exit\n");
    printf("  -p, --pgn PGN          Restrict to a single PGN (hex or dec)\n");
    printf("Record options:\n");
    printf("  -o, --out FILE          Output JSON file\n");
    printf("Examples:\n");
    for (size_t i = 0; i < sizeof(examples)/sizeof(examples[0]); ++i) {
    printf("  %s\n", examples[i]);
    }
    printf("\n");
    printf("CAN interface quick setup:\n");
    printf("  sudo ip link set can0 down\n");
    printf("  sudo ip link set can0 type can bitrate <125000|250000|500000|1000000>\n");
    printf("  sudo ip link set can0 up\n");
    printf("  ip -details link show can0\n");
    printf("Notes:\n");
    printf("Tip: run '%s list' to see available interfaces\n", PROGRAM_NAME);
    printf("Configuration file (ini-style):\n");
    printf("  Search order: $CANHOST_CONF | ./canhost.ini | ~/.canhost.ini | /etc/canhost.ini\n");
    printf("  Example: interface=can0\n");
}

void help_print_unknown_command(const char *program_name)
{
    const char *pn = program_name && *program_name ? program_name : PROGRAM_NAME;
    printf("Available commands:\n");
    printf("  list\n  probe\n  read\n  record\n  sync\n");
    printf("\nHint: run '%s --help' for usage.\n", pn);
    printf("Examples: %s list | %s probe | %s read | %s record | %s sync\n", pn, pn, pn, pn, pn);
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
    fprintf(stderr, "Notes:\n");
    fprintf(stderr, "Tip: run '%s list' to see available interfaces\n", PROGRAM_NAME);
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
