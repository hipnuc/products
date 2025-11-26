// main.c
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/stat.h>
#include <stdbool.h>

#include "commands.h"
#include "log.h"
#include "config.h"

#define PROGRAM_NAME "canhost"
#define VERSION "1.0.1"

static void print_version(void);
static void print_usage(const char *program_name);
static void signal_handler(int signum);

/**
 * Entry point for canhost.
 * Parses minimal CLI options (help/version), loads configuration, and dispatches the selected command.
 */
int main(int argc, char *argv[])
{
    log_set_level(LOG_INFO);
    config_init();
    // Print config source and values on startup
    config_log_summary();

    int opt;
    static struct option long_options[] = {
        {"help",         no_argument,       0, 'h'},
        {"version",      no_argument,       0, 'v'},
        {"interface",    no_argument,       0, 0},
        {"node-id",      no_argument,       0, 0},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "+hv", long_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'v':
                print_version();
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: missing command\n\n");
        print_usage(argv[0]);
        return 1;
    }

    const char *command = argv[optind];
    int command_argc = argc - optind;
    char **command_argv = &argv[optind];

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int result = execute_command(command, command_argc, command_argv);

    return result;
}

static void print_version(void)
{
    printf("%s %s - HiPNUC CAN evaluation utility\n", PROGRAM_NAME, VERSION);
    printf("Copyright (C) 2024 HiPNUC\n");
    printf("Website: https://www.hipnuc.com\n\n");
}

static void print_usage(const char *program_name)
{
    print_version();

    printf("Usage: %s [OPTIONS] <command> [command options]\n\n", program_name);

    printf("Options:\n");
    printf("  -h, --help              Show this help\n");
    printf("  -v, --version           Show version information\n\n");

    printf("Commands:\n");
    printf("  list    Show detected CAN interfaces with state\n");
    printf("  probe   Send a 2 s probe and print responding nodes\n");
    printf("  read    Stream and format HiPNUC sensor data frames\n");
    printf("  record  Record parsed JSON (NDJSON) to file, print frames/fps\n\n");

    printf("Read options:\n\n");
    
    printf("Record options:\n");
    printf("  -o, --out FILE          Output NDJSON file\n\n");

    printf("Examples:\n");
    printf("  %s list\n", program_name);
    printf("  %s probe\n", program_name);
    printf("  %s read\n", program_name);
    printf("  %s record -o out.jsonl\n\n", program_name);

    printf("CAN interface quick setup:\n");
    printf("  sudo ip link set can0 down\n");
    printf("  sudo ip link set can0 type can bitrate 500000\n");
    printf("  sudo ip link set can0 up\n");
    printf("Tip: run 'canhost list' to see available interfaces\n\n");

    printf("Configuration file (ini-style):\n");
    printf("  Search order: $CANHOST_CONF | ./canhost.ini | ~/.canhost.ini | /etc/canhost.ini\n");
    printf("  Example: interface=can0\n\n");
}

static void signal_handler(int signum)
{
    (void)signum;
}
