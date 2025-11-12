// commands.h
#ifndef COMMANDS_H
#define COMMANDS_H

#include <linux/can.h>
#include "hipnuc_can_parser.h"

// Command dispatcher (configuration comes from config module)
int execute_command(const char *command_name, int argc, char *argv[]);

#endif // COMMANDS_H
