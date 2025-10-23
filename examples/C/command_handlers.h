// command_handlers.h
#ifndef COMMAND_HANDLERS_H
#define COMMAND_HANDLERS_H

#include "global_options.h"

int cmd_list(GlobalOptions *opts, int argc, char *argv[]);
int cmd_read(GlobalOptions *opts, int argc, char *argv[]);
int cmd_write(GlobalOptions *opts, int argc, char *argv[]);
int cmd_probe(GlobalOptions *opts, int argc, char *argv[]);
int cmd_update(GlobalOptions *opts, int argc, char *argv[]);
int cmd_example(GlobalOptions *opts, int argc, char *argv[]);

#endif // COMMAND_HANDLERS_H