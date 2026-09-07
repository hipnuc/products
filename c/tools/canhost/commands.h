#ifndef COMMANDS_H
#define COMMANDS_H

#define CANHOST_EXIT_OK 0
#define CANHOST_EXIT_RUNTIME_ERROR 1
#define CANHOST_EXIT_INVALID_ARGS 2

/* Command dispatcher; configuration comes from the config module. */
int execute_command(const char *command_name, int argc, char *argv[]);

#endif /* COMMANDS_H */
