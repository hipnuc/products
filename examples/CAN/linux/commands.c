// commands.c
#include "commands.h"
#include "command_handlers.h"
#include "log.h"
#include "can_interface.h"
#include "utils.h"
#include "help.h"
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
    {"record", cmd_record, 1},
    {"sync",   cmd_sync,  1},
    {"reg",    cmd_reg,   1},
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
                    help_print_can_setup(ifname);
                    return -1;
                }
            }
            return commands[i].func(argc, argv);
        }
    }
    
    log_error("Unknown command: %s", command_name);
    help_print_unknown_command("canhost");

    return -1;
}
