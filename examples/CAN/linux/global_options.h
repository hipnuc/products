// global_options.h
#ifndef GLOBAL_OPTIONS_H
#define GLOBAL_OPTIONS_H

#include <stdint.h>

#define TMP_CONFIG_FILE "/tmp/canhost_config.tmp"

typedef struct {
    char *can_interface;        // Selected CAN interface (e.g. can0)
    uint8_t node_id;            // Target node ID (default 8)
} GlobalOptions;

#endif // GLOBAL_OPTIONS_H
