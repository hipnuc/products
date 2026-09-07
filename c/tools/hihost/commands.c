#include "commands.h"

#include <string.h>

#include "command_handlers.h"
#include "log.h"

typedef struct {
    const char *name;
    int (*func)(GlobalOptions *, int, char **);
    int needs_port;
} command_t;

static const command_t commands[] = {
    {"list",   cmd_list,   0},
    {"probe",  cmd_probe,  0},
    {"read",   cmd_read,   1},
    {"write",  cmd_write,  1},
    {"update", cmd_update, 1},
    {NULL, NULL, 0}
};

static const command_t *find(const char *name)
{
    for (int i = 0; commands[i].name != NULL; i++) {
        if (strcmp(name, commands[i].name) == 0) {
            return &commands[i];
        }
    }
    return NULL;
}

int command_needs_port(const char *command_name)
{
    const command_t *c = find(command_name);
    return c ? c->needs_port : 0;
}

int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[])
{
    const command_t *c = find(command_name);
    if (!c) {
        log_error("Unknown command: %s", command_name);
        return -1;
    }
    return c->func(opts, argc, argv);
}
