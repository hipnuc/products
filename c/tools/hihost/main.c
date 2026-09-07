/*
 * hihost: HiPNUC serial command line tool.
 *
 *   hihost [GLOBAL OPTIONS] COMMAND [COMMAND OPTIONS]
 */
#include <errno.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "global_options.h"
#include "hihost_config.h"

#define PROGRAM_NAME "hihost"
#define VERSION "2.0.0"

static void print_version(void)
{
    printf("%s %s\n", PROGRAM_NAME, VERSION);
}

static void print_usage(const char *program_name)
{
    printf("%s %s - HiPNUC serial command line tool\n\n", PROGRAM_NAME, VERSION);
    printf("Usage: %s [GLOBAL OPTIONS] COMMAND [COMMAND OPTIONS]\n\n", program_name);
    printf("Global options:\n");
    printf("  -p, --port PORT         Serial port (e.g. /dev/ttyUSB0)\n");
    printf("  -b, --baud RATE         Baud rate\n");
    printf("  -r, --record-raw FILE   Record raw serial bytes to a binary file (read)\n");
    printf("  -j, --record-json FILE  Record decoded samples as JSON lines (read)\n");
    printf("  -h, --help              Show this help and exit\n");
    printf("  -v, --version           Show the version and exit\n\n");
    printf("Commands:\n");
    printf("  list                    List serial ports\n");
    printf("  probe [--save]          Find the device on every port; --save writes ./hihost.ini\n");
    printf("  read                    Decode and display the data stream\n");
    printf("  write COMMAND...        Send one ASCII command (e.g. \"LOG VERSION\")\n");
    printf("  write FILE              Send the commands listed in FILE, one per line\n");
    printf("  update FILE             Update the firmware from an Intel HEX file\n\n");
    printf("Port and baud come from -p/-b or from the first of $HIHOST_CONF,\n");
    printf("./hihost.ini, ~/.hihost.ini (keys: port=, baud=).\n\n");
    printf("Examples:\n");
    printf("  %s probe --save\n", program_name);
    printf("  %s -p /dev/ttyUSB0 -b 115200 read\n", program_name);
    printf("  %s -r raw.bin -j data.jsonl read\n", program_name);
    printf("  %s write \"LOG VERSION\"\n", program_name);
    printf("  %s write device_setup.ini\n", program_name);
    printf("  %s update firmware.hex\n", program_name);
}

int main(int argc, char *argv[])
{
    GlobalOptions opts = { NULL, 0, NULL, NULL };
    char ini_port[256] = {0};
    char ini_source[512] = {0};
    int ini_baud = 0;
    int have_ini = 0;

    static const struct option long_options[] = {
        {"help",        no_argument,       0, 'h'},
        {"version",     no_argument,       0, 'v'},
        {"port",        required_argument, 0, 'p'},
        {"baud",        required_argument, 0, 'b'},
        {"record-raw",  required_argument, 0, 'r'},
        {"record-json", required_argument, 0, 'j'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "+hvp:b:r:j:", long_options, NULL)) != -1) {
        switch (opt) {
        case 'h':
            print_usage(argv[0]);
            return 0;
        case 'v':
            print_version();
            return 0;
        case 'p':
            opts.port_name = optarg;
            break;
        case 'b': {
            char *endptr = NULL;
            long baud;
            errno = 0;
            baud = strtol(optarg, &endptr, 10);
            if (errno != 0 || !endptr || *endptr != '\0' || baud <= 0 || baud > 10000000L) {
                fprintf(stderr, "Invalid baud rate: %s\n", optarg);
                return 1;
            }
            opts.baud_rate = (int)baud;
            break;
        }
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
        fprintf(stderr, "Expected a command\n\n");
        print_usage(argv[0]);
        return 1;
    }

    const char *command = argv[optind++];

    if (!opts.port_name || opts.baud_rate <= 0) {
        have_ini = hihost_config_load(ini_port, sizeof(ini_port), &ini_baud,
                                      ini_source, sizeof(ini_source)) == 0;
        if (have_ini) {
            if (!opts.port_name) opts.port_name = ini_port;
            if (opts.baud_rate <= 0) opts.baud_rate = ini_baud;
        }
    }

    if (command_needs_port(command) && (!opts.port_name || opts.baud_rate <= 0)) {
        fprintf(stderr, "Missing port or baud rate. Use -p/-b, or run `%s probe --save`"
                        " to write ./hihost.ini.\n", argv[0]);
        return 1;
    }

    int rc = execute_command(command, &opts, argc - optind, argv + optind);
    return rc == 0 ? 0 : 1;
}
