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

typedef struct {
    uint32_t pgn;
    uint32_t period_ms;
} config_sync_item_t;

int config_get_sync_items(config_sync_item_t *items, int max_count);
int config_get_sync_count(void);

// Returns the number of target nodes. Fills `nodes` up to `max_count`.
int config_get_target_nodes(uint8_t *nodes, int max_count);

// Clears all configured target nodes.
void config_clear_target_nodes(void);

// Adds a target node.
void config_add_target_node(uint8_t node_id);

// Legacy: returns the first configured node or default.
uint8_t config_get_target_node(void);

uint8_t config_get_sync_sa(void);

int config_get_canfd_enable(void);
int config_get_canfd_brs(void);
uint32_t config_get_canfd_data_bitrate(void);

#endif // CANHOST_CONFIG_H
