// command_handlers.h
#ifndef COMMAND_HANDLERS_H
#define COMMAND_HANDLERS_H

// Command entry-points (configuration is global via config module)
int cmd_list(int argc, char *argv[]);
int cmd_probe(int argc, char *argv[]);
int cmd_read(int argc, char *argv[]);
int cmd_record(int argc, char *argv[]);
int cmd_sync(int argc, char *argv[]);

#endif // COMMAND_HANDLERS_H
