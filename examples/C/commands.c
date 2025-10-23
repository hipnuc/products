#include "commands.h"
#include "command_handlers.h"
#include "log.h"
#include <string.h>

// Command registry entry
typedef struct {
    const char *name;
    int (*func)(GlobalOptions*, int, char**);
} command_t;

// Array of available commands
static command_t commands[] = {
    {"list",   cmd_list},
    {"probe",  cmd_probe},
    {"read",   cmd_read},
    {"write",  cmd_write},
    {"update", cmd_update},
    {"example",cmd_example},
    {NULL, NULL}
};

int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[]) {
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(command_name, commands[i].name) == 0) {
            return commands[i].func(opts, argc, argv);
        }
    }
    log_error("Unknown command: %s", command_name);
    return -1;
}









