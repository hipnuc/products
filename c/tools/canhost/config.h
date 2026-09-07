/*
 * canhost.ini: loaded from the first of $CANHOST_CONF, ./canhost.ini,
 * ~/.canhost.ini, /etc/canhost.ini. Defaults: interface=can0, node_id=8,
 * sync.sa=0x55, canfd=1.
 */
#ifndef CANHOST_CONFIG_H
#define CANHOST_CONFIG_H

#include <stdint.h>

#define CONFIG_MAX_NODES 32
#define CONFIG_MAX_SYNC_ITEMS 32

int config_init(void);

const char *config_get_interface(void);

/* Path of the file that was loaded, NULL when defaults are in use. */
const char *config_get_source(void);

void config_log_summary(void);

typedef struct {
    uint32_t pgn;
    uint32_t period_ms;
} config_sync_item_t;

int config_get_sync_items(config_sync_item_t *items, int max_count);
int config_get_sync_count(void);

/* Target nodes (J1939 source addresses). */
int config_get_target_nodes(uint8_t *nodes, int max_count);
void config_clear_target_nodes(void);
void config_add_target_node(uint8_t node_id);

/* Host source address used in configuration frames. */
uint8_t config_get_sync_sa(void);

int config_get_canfd_enable(void);
int config_get_canfd_brs(void);
uint32_t config_get_canfd_data_bitrate(void);

#endif /* CANHOST_CONFIG_H */
