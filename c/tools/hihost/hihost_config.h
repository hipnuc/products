/*
 * hihost.ini handling and small helpers.
 *
 * Lookup order: $HIHOST_CONF, ./hihost.ini, ~/.hihost.ini. The file holds
 * `port=` and `baud=`; `probe --save` writes ./hihost.ini.
 */
#ifndef HIHOST_CONFIG_H
#define HIHOST_CONFIG_H

#include <stddef.h>

#define HIHOST_INI_FILE "hihost.ini"

/* Sleep for the given microseconds, restarting after EINTR. */
int safe_sleep(unsigned long usec);

/* Load port/baud from the first readable configuration file. Returns 0 and
 * fills `source` (may be NULL) with the path used; -1 when none was found
 * or the file lacks port or baud. */
int hihost_config_load(char *port_out, size_t port_size, int *baud_out,
                       char *source, size_t source_size);

/* Write port/baud to `path`. Returns 0 or -1. */
int hihost_config_save(const char *path, const char *port, int baud);

#endif /* HIHOST_CONFIG_H */
