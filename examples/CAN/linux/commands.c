// commands.c
#include "commands.h"
#include "command_handlers.h"
#include "log.h"
#include "help.h"
#include <string.h>


// Command registry entry
typedef struct {
    const char *name;
    int (*func)(int, char**);
} command_t;

// Available commands
static command_t commands[] = {
    {"list",   cmd_list},
    {"probe",  cmd_probe},
    {"read",   cmd_read},
    {"record", cmd_record},
    {"sync",   cmd_sync},
    {"reg",    cmd_reg},
    {"action", cmd_action},
    {NULL, NULL}
};

// Dispatches the requested command
int execute_command(const char *command_name, int argc, char *argv[])
{
    if (!command_name) {
        log_error("Command name cannot be null");
        return CANHOST_EXIT_INVALID_ARGS;
    }
    
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(command_name, commands[i].name) == 0) {
            log_debug("Executing command: %s", command_name);
            return commands[i].func(argc, argv);
        }
    }
    
    log_error("Unknown command: %s", command_name);
    help_print_arg_error_json("command", "unknown command");
    help_print_unknown_command("canhost");

    return CANHOST_EXIT_INVALID_ARGS;
}
