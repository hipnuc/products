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
#include "cmd_utils.h"
#include "log.h"


#define PROGRAM_NAME "hihost"
#define VERSION "1.0.5"


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
        .baud_rate = 0,
        .record_raw_file = NULL,
        .record_json_file = NULL
    };

    {
        char port_buf[256];
        int baud = 0;
        if (ini_load_serial(HIHOST_INI_ABS_PATH, port_buf, sizeof(port_buf), &baud) == 0) {
            opts.port_name = strdup(port_buf);
            opts.baud_rate = baud;
        }
    }

    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"port", required_argument, 0, 'p'},
        {"baud", required_argument, 0, 'b'},
        {"version", no_argument, 0, 'v'},
        {"record-raw", required_argument, 0, 'r'},
        {"record-json", required_argument, 0, 'j'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "hvp:b:r:j:", long_options, NULL)) != -1) {
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
            case 'r':
                opts.record_raw_file = optarg;
                break;
            case 'j':
                opts.record_json_file = optarg;
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
    if ((!opts.port_name || opts.baud_rate <= 0) && strcmp(command, "probe") != 0 && strcmp(command, "list") != 0) {
        fprintf(stderr, "Missing port/baud. Please edit %s or provide -p/-b.\n", HIHOST_INI_ABS_PATH);
        return 1;
    }
    return execute_command(command, &opts, argc - optind, argv + optind);
}


static void print_usage(const char *program_name) {
    printf("%s %s - HiPNUC Products Evaluation Tool\n", PROGRAM_NAME, VERSION);
    printf("Copyright (C) 2024 HiPNUC. All rights reserved.\n");
    printf("Website: www.hipnuc.com\n\n");

    printf("Usage: %s [GLOBAL OPTIONS] COMMAND [COMMAND OPTIONS]\n\n", program_name);

    printf("Global Options:\n");
    printf("  -p, --port PORT         Specify the serial port (e.g., /dev/ttyUSB0)\n");
    printf("  -b, --baud RATE         Set the baud rate\n");
    printf("  -r, --record-raw FILE   Record raw serial data to binary file\n");
    printf("  -j, --record-json FILE  Record parsed JSON data to text file\n");
    printf("  -h, --help              Display this help message and exit\n");
    printf("  -v, --version           Display version information and exit\n\n");

    printf("Recording Options:\n");
    printf("  Raw data recording (-r): Saves all received serial bytes to a binary file\n");
    printf("                          for later analysis or replay. No data loss.\n");
    printf("  JSON data recording (-j): Saves parsed IMU/GNSS data in JSON format,\n");
    printf("                           one JSON object per line. Human-readable.\n");
    printf("  Both options can be used simultaneously for complete data capture.\n\n");

    printf("Commands:\n");
    printf("  list                    List all available serial ports\n");
    printf("  probe                   Automatically probe for device on all serial port\n");
    printf("  read                    Enter read mode to display IMU data\n");
    printf("  update <HEX_FILE>       Device firmware update\n");
    printf("  write <COMMAND>         Send a single command to the device\n");
    printf("  write <CONFIG_FILE>     Execute commands from a configuration file\n");
    printf("  example                 Process example static fix data\n\n");

    printf("Examples:\n");
    printf("  Basic usage:\n");
    printf("    %s list                                    # List available ports\n", program_name);
    printf("    %s probe                                   # Auto-detect device\n", program_name);
    printf("    %s -p /dev/ttyUSB0 read                    # Read and display data\n", program_name);
    printf("\n");
    printf("  Data recording:\n");
    printf("    %s -p /dev/ttyUSB0 -r imu_raw.bin read     # Record raw data only\n", program_name);
    printf("    %s -p /dev/ttyUSB0 -j imu_data.json read   # Record JSON data only\n", program_name);
    printf("    %s -p /dev/ttyUSB0 -r raw.bin -j data.json read  # Record both formats\n", program_name);
    printf("\n");
    printf("  Device configuration:\n");
    printf("    %s -p /dev/ttyUSB0 write \"AT+INFO\"         # Send single command\n", program_name);
    printf("    %s -p /dev/ttyUSB0 write config.txt        # Execute command file\n", program_name);
    printf("    %s -p /dev/ttyUSB0 update firmware.hex     # Update firmware\n", program_name);
    printf("\n");
    printf("  Other:\n");
    printf("    %s example                                 # Process example data\n\n", program_name);
}

static void signal_handler(int signum) {
    printf("\nReceived signal %d. Exiting...\n", signum);
    exit(0);
}
