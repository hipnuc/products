#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
        {"node",    required_argument, 0, 'n'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "+hvn:", opts, NULL)) != -1) {
        switch (opt) {
            case 'h': help_print_usage(argv[0]); return 0;
            case 'v': help_print_version();      return 0;
            case 'n': {
                static int node_cleared = 0;
                if (!node_cleared) {
                    config_clear_target_nodes();
                    node_cleared = 1;
                }
                char *arg_copy = strdup(optarg);
                if (arg_copy) {
                    char *token = strtok(arg_copy, ",");
                    while (token) {
                        config_add_target_node((uint8_t)strtoul(token, NULL, 0));
                        token = strtok(NULL, ",");
                    }
                    free(arg_copy);
                }
                break;
            }
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
