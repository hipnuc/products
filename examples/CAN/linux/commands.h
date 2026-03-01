// commands.h
#ifndef COMMANDS_H
#define COMMANDS_H

#include <linux/can.h>
#include "hipnuc_can_common.h"

#define CANHOST_EXIT_OK 0
#define CANHOST_EXIT_RUNTIME_ERROR 1
#define CANHOST_EXIT_INVALID_ARGS 2

// Command dispatcher (configuration comes from config module)
int execute_command(const char *command_name, int argc, char *argv[]);

#endif // COMMANDS_H
