// global_options.h
#ifndef GLOBAL_OPTIONS_H
#define GLOBAL_OPTIONS_H

#define TMP_CONFIG_FILE "/tmp/hihost_config.tmp"

typedef struct {
    char *port_name;
    int baud_rate;
} GlobalOptions;

#endif
