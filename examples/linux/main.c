#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#include "commands.h"
#include "global_options.h"
#include "log.h"


#define PROGRAM_NAME "hihost"
#define VERSION "1.0.1"

static void print_usage(const char *program_name);
static void signal_handler(int signum);

int main(int argc, char *argv[]) {
    GlobalOptions opts = {
        .port_name = NULL,
        .baud_rate = 115200
    };

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
    printf("\n%s %s  %s\n", PROGRAM_NAME, VERSION, "www.hipnuc.com");
    fprintf(stderr, "Usage: %s [GLOBAL OPTIONS] <COMMAND> [COMMAND OPTIONS]\n", program_name);
    fprintf(stderr, "Global Options:\n");
    fprintf(stderr, "  -p, --port PORT         Specify the serial port (e.g., /dev/ttyUSB0)\n");
    fprintf(stderr, "  -b, --baud RATE         Specify the baud rate (default: 115200)\n");
    fprintf(stderr, "  -h, --help              Display this help message\n");
    fprintf(stderr, "  -v, --version           Display version\n");
    fprintf(stderr, "Commands:\n");
    fprintf(stderr, "  list                    List all available serial ports\n");
    fprintf(stderr, "  read                    Enter read mode to display IMU data\n");
    fprintf(stderr, "  write <COMMAND>         Send a command to the device\n");
    fprintf(stderr, "  example                 Process example static fix data\n");
}

static void signal_handler(int signum) {
    printf("\nReceived signal %d. Exiting...\n", signum);
    exit(0);
}
