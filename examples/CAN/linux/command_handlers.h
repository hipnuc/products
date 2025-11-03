// command_handlers.h
#ifndef COMMAND_HANDLERS_H
#define COMMAND_HANDLERS_H

#include "global_options.h"

// Command entry-points
int cmd_list(GlobalOptions *opts, int argc, char *argv[]);
int cmd_probe(GlobalOptions *opts, int argc, char *argv[]);
int cmd_read(GlobalOptions *opts, int argc, char *argv[]);
int cmd_stats(GlobalOptions *opts, int argc, char *argv[]);

#endif // COMMAND_HANDLERS_H
