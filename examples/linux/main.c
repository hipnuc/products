#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <sys/stat.h>

#include "commands.h"
#include "global_options.h"
#include "log.h"


#define PROGRAM_NAME "hihost"
#define VERSION "1.0.3"


static void print_usage(const char *program_name);
static void signal_handler(int signum);

/**
 * The main entry point of the HiPNUC Configuration and Monitoring Tool.
 * This function parses the command-line arguments, sets up the global options,
 * and executes the requested command.
 *
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return The exit status of the program.
 */
int main(int argc, char *argv[]) {
    GlobalOptions opts = {
        .port_name = NULL,
        .baud_rate = 115200
    };

    FILE *tmp_file = fopen(TMP_CONFIG_FILE, "r");
    if (tmp_file) {
        char port_buf[256];
        int baud;
        if (fscanf(tmp_file, "%255s %d", port_buf, &baud) == 2) {
            opts.port_name = strdup(port_buf);
            opts.baud_rate = baud;
            printf("opts.port_name:%s, %d\r\n", opts.port_name, opts.baud_rate);
        }
        fclose(tmp_file);
    }

    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"port", required_argument, 0, 'p'},
        {"baud", required_argument, 0, 'b'},
        {"version", no_argument, 0, 'v'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "hvp:b:", long_options, NULL)) != -1) {
        switch (opt) {
            case 'h':
                print_usage(argv[0]);
                return 0;
            case 'v':
                print_usage(argv[0]);
                return 0;
            case 'p':
                opts.port_name = optarg;
                break;
            case 'b':
                {
                    char *endptr;
                    errno = 0;
                    long baud = strtol(optarg, &endptr, 10);
                    if (errno != 0 || *endptr != '\0' || baud <= 0) {
                        fprintf(stderr, "Invalid baud rate: %s\n", optarg);
                        return 1;
                    }
                    opts.baud_rate = (int)baud;
                }
                break;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Expected command\n");
        print_usage(argv[0]);
        return 1;
    }

    const char *command = argv[optind];
    optind++;
    return execute_command(command, &opts, argc - optind, argv + optind);
}


static void print_usage(const char *program_name) {
    printf("%s %s - HiPNUC Products Evaluation Tool\n", PROGRAM_NAME, VERSION);
    printf("Copyright (C) 2024 HiPNUC. All rights reserved.\n");
    printf("Website: www.hipnuc.com\n\n");

    printf("Usage: %s [GLOBAL OPTIONS] COMMAND [COMMAND OPTIONS]\n\n", program_name);

    printf("Global Options:\n");
    printf("  -p, --port PORT         Specify the serial port (e.g., /dev/ttyUSB0)\n");
    printf("  -b, --baud RATE         Set the baud rate (default: 115200)\n");
    printf("  -h, --help              Display this help message and exit\n");
    printf("  -v, --version           Display version information and exit\n\n");

    printf("Commands:\n");
    printf("  list                    List all available serial ports\n");
    printf("  probe                   Automatically probe for device on all serial port\n");
    printf("  read                    Enter read mode to display IMU data\n");
    printf("  update <HEX_FILE>       Device firmware update\n");
    printf("  write <COMMAND>         Send a single command to the device\n");
    printf("  write <CONFIG_FILE>     Execute commands from a configuration file\n");
    printf("  example                 Process example static fix data\n\n");

    printf("Examples:\n");
    printf("  %s  list\n", program_name);
    printf("  %s  probe\n", program_name);
    printf("  %s  example\n", program_name);
    printf("  %s -p /dev/ttyUSB0 -b 115200 read\n", program_name);
    printf("  %s -p /dev/ttyUSB0 -b 115200 write \"AT+INFO\"\n", program_name);
    printf("  %s -p /dev/ttyUSB0 -b 115200 write <FILE>\n", program_name);
    printf("  %s -p /dev/ttyUSB0 -b 115200 update <HEX_FILE>\n\n", program_name);
}

static void signal_handler(int signum) {
    printf("\nReceived signal %d. Exiting...\n", signum);
    exit(0);
}
