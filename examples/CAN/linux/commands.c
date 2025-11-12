// commands.c
#include "commands.h"
#include "command_handlers.h"
#include "log.h"
#include "can_interface.h"
#include "utils.h"
#include "config.h"
#include <string.h>


// Command registry entry
typedef struct {
    const char *name;
    int (*func)(int, char**);
    int needs_interface; // 0: interface optional; 1: enforce readiness
} command_t;

// Available commands
static command_t commands[] = {
    {"list",   cmd_list, 0},
    {"probe",  cmd_probe, 1},
    {"read",   cmd_read,  1},
    {NULL, NULL, 0}
};

// Dispatches the requested command
int execute_command(const char *command_name, int argc, char *argv[])
{
    if (!command_name) {
        log_error("Command name cannot be null");
        return -1;
    }
    
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(command_name, commands[i].name) == 0) {
            log_debug("Executing command: %s", command_name);
            if (commands[i].needs_interface) {
                const char *ifname = config_get_interface();
                if (can_ensure_interface_ready(ifname) < 0) {
                    utils_print_can_setup_hint(ifname);
                    return -1;
                }
            }
            return commands[i].func(argc, argv);
        }
    }
    
    log_error("Unknown command: %s", command_name);
    printf("Available commands:\n");
    for (int i = 0; commands[i].name != NULL; i++) {
        printf("  %s\n", commands[i].name);
    }
    printf("\nHint: run 'canhost --help' for usage.\n");
    printf("Examples: canhost list | canhost probe | canhost read\n");

    return -1;
}

