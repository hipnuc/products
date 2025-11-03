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
#include "global_options.h"
#include "log.h"

#define PROGRAM_NAME "canhost"
#define VERSION "1.0.0"

static void print_version(void);
static void print_usage(const char *program_name);
static void signal_handler(int signum);

/**
 * Entry point for canhost.
 * Parses CLI options, applies defaults, and dispatches the selected command.
 */
int main(int argc, char *argv[])
{
    GlobalOptions opts = {
        .can_interface = NULL,
        .node_id = 8
    };
    bool interface_from_file = false;

    log_set_level(LOG_INFO);

    FILE *tmp_file = fopen(TMP_CONFIG_FILE, "r");
    if (tmp_file) {
        char interface_buf[32];
        int node_id;
        if (fscanf(tmp_file, "%31s %d", interface_buf, &node_id) == 2) {
            opts.can_interface = strdup(interface_buf);
            opts.node_id = node_id;
            interface_from_file = true;
        }
        fclose(tmp_file);
    }

    int opt;
    static struct option long_options[] = {
        {"help",         no_argument,       0, 'h'},
        {"version",      no_argument,       0, 'v'},
        {"interface",    required_argument, 0, 'i'},
        {"node-id",      required_argument, 0, 'n'},
        {0, 0, 0, 0}
    };

    while ((opt = getopt_long(argc, argv, "hvi:n:", long_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'v':
                print_version();
                return 0;
            case 'i':
                if (interface_from_file && opts.can_interface) {
                    free(opts.can_interface);
                    interface_from_file = false;
                }
                opts.can_interface = optarg;
                break;
            case 'n':
            {
                char *endptr;
                errno = 0;
                long node = strtol(optarg, &endptr, 10);
                if (errno != 0 || *endptr != '\0' || node < 0 || node > 255) {
                    fprintf(stderr, "Invalid node ID: %s\n", optarg);
                    return 1;
                }
                opts.node_id = (uint8_t)node;
                break;
            }
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

    int result = execute_command(command, &opts, command_argc, command_argv);

    if (interface_from_file && opts.can_interface) {
        free(opts.can_interface);
    }

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
    printf("  -i, --interface IFACE   Select CAN interface (e.g. can0)\n");
    printf("  -n, --node-id ID        Target node ID (0-255, default 8)\n");
    printf("  -h, --help              Show this help\n");
    printf("  -v, --version           Show version information\n\n");

    printf("Commands:\n");
    printf("  list    Show detected CAN interfaces with state/bitrate\n");
    printf("  probe   Send a 2 s probe and print responding nodes\n");
    printf("  read    Stream and format HiPNUC sensor data frames\n");
    printf("  stats   Monitor frame rates per CAN ID (no parsing)\n\n");

    printf("Examples:\n");
    printf("  %s list\n", program_name);
    printf("  %s -i can0 probe\n", program_name);
    printf("  %s -i can0 -n 8 read\n", program_name);
    printf("  %s -i can0 stats\n\n", program_name);

    printf("CAN interface quick setup:\n");
    printf("  sudo ip link set can0 down\n");
    printf("  sudo ip link set can0 type can bitrate 500000\n");
    printf("  sudo ip link set can0 up\n");
    printf("  ip -details link show can0\n");
    printf("Tip: run 'canhost list' to see available interfaces\n\n");
}

static void signal_handler(int signum)
{
    (void)signum;
}
