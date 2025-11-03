// commands.h
#ifndef COMMANDS_H
#define COMMANDS_H

#include "global_options.h"
#include <linux/can.h>
#include "hipnuc_can_parser.h"


// Command dispatcher
int execute_command(const char *command_name, GlobalOptions *opts, int argc, char *argv[]);

#endif // COMMANDS_H
