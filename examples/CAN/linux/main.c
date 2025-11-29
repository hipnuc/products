#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>

#include "commands.h"
#include "log.h"
#include "config.h"
#include "help.h"

/**
 * Entry point for canhost.
 * Parses CLI options and dispatches the selected command.
 */
int main(int argc, char *argv[])
{
    log_set_level(LOG_INFO);
    config_init();
    config_log_summary();

    static struct option opts[] = {
        {"help",    no_argument, 0, 'h'},
        {"version", no_argument, 0, 'v'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "+hv", opts, NULL)) != -1) {
        switch (opt) {
            case 'h': help_print_usage(argv[0]); return 0;
            case 'v': help_print_version();      return 0;
            default:  help_print_usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: missing command\n\n");
        help_print_usage(argv[0]);
        return 1;
    }

    return execute_command(argv[optind], argc - optind, &argv[optind]);
}
