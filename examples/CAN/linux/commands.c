// commands.c
#include "commands.h"
#include "command_handlers.h"
#include "log.h"
#include "can_interface.h"
#include "utils.h"
#include <string.h>


// Command registry entry
typedef struct {
    const char *name;
    int (*func)(GlobalOptions*, int, char**);
    int needs_interface; // 0: interface optional; 1: enforce readiness
} command_t;

// Available commands
static command_t commands[] = {
    {"list",   cmd_list, 0},
    {"probe",  cmd_probe, 1},
    {"read",   cmd_read,  1},
    {"stats",  cmd_stats, 1},
    {NULL, NULL, 0}
};

// Dispatches the requested command
int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[])
{
    if (!command_name) {
        log_error("Command name cannot be null");
        return -1;
    }
    
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(command_name, commands[i].name) == 0) {
            log_debug("Executing command: %s", command_name);
            if (commands[i].needs_interface) {
                if (can_ensure_interface_ready(opts->can_interface) < 0) {
                    utils_print_can_setup_hint(opts->can_interface);
                    return -1;
                }
            }
            return commands[i].func(opts, argc, argv);
        }
    }
    
    log_error("Unknown command: %s", command_name);
    printf("Available commands:\n");
    for (int i = 0; commands[i].name != NULL; i++) {
        printf("  %s\n", commands[i].name);
    }
    printf("\nHint: run 'canhost --help' for usage.\n");
    printf("Examples: canhost list | canhost -i can0 probe | canhost -i can0 stats\n");
    
    return -1;
}

