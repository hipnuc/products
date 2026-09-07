#ifndef COMMANDS_H
#define COMMANDS_H

#include "global_options.h"

/* Run `command_name` with the remaining arguments. Returns the command's
 * exit status, or -1 for an unknown command. */
int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[]);

/* 1 when the command needs an open serial port (port and baud). */
int command_needs_port(const char *command_name);

#endif /* COMMANDS_H */
