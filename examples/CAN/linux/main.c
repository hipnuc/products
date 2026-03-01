#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "config.h"
#include "help.h"
#include "log.h"

static int apply_node_override(const char *arg)
{
    if (!arg || !*arg) {
        help_print_arg_error_json("global", "empty --node argument");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    config_clear_target_nodes();
    char *arg_copy = strdup(arg);
    if (!arg_copy) {
        log_error("Out of memory");
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    char *token = strtok(arg_copy, ",");
    while (token) {
        config_add_target_node((uint8_t)strtoul(token, NULL, 0));
        token = strtok(NULL, ",");
    }
    free(arg_copy);
    return CANHOST_EXIT_OK;
}

static int dispatch_group_command(int argc, char *argv[], int pos)
{
    if (pos >= argc) {
        help_print_arg_error_json("main", "missing command");
        help_print_usage(argv[0]);
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *group = argv[pos];
    if (strcmp(group, "help") == 0) {
        help_print_usage(argv[0]);
        return CANHOST_EXIT_OK;
    }
    if (strcmp(group, "version") == 0) {
        help_print_version();
        return CANHOST_EXIT_OK;
    }

    if (pos + 1 >= argc) {
        help_print_arg_error_json(group, "missing subcommand");
        help_print_usage(argv[0]);
        return CANHOST_EXIT_INVALID_ARGS;
    }

    const char *sub = argv[pos + 1];

    if (strcmp(group, "device") == 0) {
        if (strcmp(sub, "list") == 0 || strcmp(sub, "probe") == 0) {
            return execute_command(sub, argc - (pos + 1), &argv[pos + 1]);
        }
        help_print_arg_error_json("device", "supported subcommands: list|probe");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (strcmp(group, "stream") == 0) {
        if (strcmp(sub, "read") == 0 || strcmp(sub, "record") == 0) {
            return execute_command(sub, argc - (pos + 1), &argv[pos + 1]);
        }
        help_print_arg_error_json("stream", "supported subcommands: read|record");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (strcmp(group, "trigger") == 0) {
        if (strcmp(sub, "sync") == 0) {
            return execute_command("sync", argc - (pos + 1), &argv[pos + 1]);
        }
        help_print_arg_error_json("trigger", "supported subcommand: sync");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (strcmp(group, "config") == 0) {
        if (strcmp(sub, "reg") == 0) {
            return execute_command("reg", argc - (pos + 1), &argv[pos + 1]);
        }
        help_print_arg_error_json("config", "supported subcommand: reg");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (strcmp(group, "action") == 0) {
        if (strcmp(sub, "run") == 0) {
            return execute_command("action", argc - pos, &argv[pos]);
        }
        help_print_arg_error_json("action", "supported subcommand: run");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    help_print_arg_error_json("main", "unknown command group");
    help_print_unknown_command(argv[0]);
    return CANHOST_EXIT_INVALID_ARGS;
}

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
            case 'h':
                help_print_usage(argv[0]);
                return CANHOST_EXIT_OK;
            case 'v':
                help_print_version();
                return CANHOST_EXIT_OK;
            case 'n': {
                int rc = apply_node_override(optarg);
                if (rc != CANHOST_EXIT_OK) {
                    return rc;
                }
                break;
            }
            default:
                help_print_arg_error_json("global", "invalid global option");
                help_print_usage(argv[0]);
                return CANHOST_EXIT_INVALID_ARGS;
        }
    }

    return dispatch_group_command(argc, argv, optind);
}

