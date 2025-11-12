#ifndef CANHOST_CONFIG_H
#define CANHOST_CONFIG_H

#include <stdint.h>

// Initialize configuration by loading from a simple ini-style file.
// Search order:
// 1) Environment variable `CANHOST_CONF`
// 2) `./canhost.ini`
// 3) `$HOME/.canhost.ini`
// 4) `/etc/canhost.ini`
// Defaults: interface="can0"
int config_init(void);

// Accessors for loaded configuration
const char* config_get_interface(void);

// Configuration source path (NULL if defaults were used)
const char* config_get_source(void);

// Log a one-line summary of config source and values
void config_log_summary(void);

#endif // CANHOST_CONFIG_H
