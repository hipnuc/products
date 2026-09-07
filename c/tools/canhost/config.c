#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ini.h"
#include "log.h"

typedef struct {
    char interface[32];
    uint8_t target_nodes[CONFIG_MAX_NODES];
    int target_node_count;
    uint8_t sync_sa;
    int canfd_enable;
    int canfd_brs;
    uint32_t canfd_data_bitrate;
    config_sync_item_t sync_items[CONFIG_MAX_SYNC_ITEMS];
    int sync_count;
} canhost_config_t;

static canhost_config_t G = {
    .interface = "can0",
    .target_nodes = {8},
    .target_node_count = 1,
    .sync_sa = 0x55,
    .canfd_enable = 1,
    .canfd_brs = 1,
    .canfd_data_bitrate = 4000000,
    .sync_count = 0
};
static int initialized = 0;
static char source_path[512] = {0};

void config_add_target_node(uint8_t node_id)
{
    if (G.target_node_count >= CONFIG_MAX_NODES) {
        return;
    }
    for (int i = 0; i < G.target_node_count; ++i) {
        if (G.target_nodes[i] == node_id) {
            return;
        }
    }
    G.target_nodes[G.target_node_count++] = node_id;
}

void config_clear_target_nodes(void)
{
    G.target_node_count = 0;
}

static void apply_kv(void *user, const char *key, const char *val_in)
{
    char val[128];
    (void)user;
    snprintf(val, sizeof(val), "%s", val_in);

    if (strcmp(key, "interface") == 0) {
        if (strlen(val) >= sizeof(G.interface)) {
            log_warn("canhost.ini: interface name too long, ignored");
        } else {
            strcpy(G.interface, val);
        }
    } else if (strcmp(key, "node_id") == 0) {
        config_clear_target_nodes();
        char *token = strtok(val, ",");
        while (token) {
            config_add_target_node((uint8_t)strtoul(token, NULL, 0));
            token = strtok(NULL, ",");
        }
    } else if (strcmp(key, "sync.sa") == 0 || strcmp(key, "sync_sa") == 0) {
        G.sync_sa = (uint8_t)strtoul(val, NULL, 0);
    } else if (strcmp(key, "canfd") == 0 || strcmp(key, "canfd.enable") == 0) {
        G.canfd_enable = strtol(val, NULL, 0) ? 1 : 0;
    } else if (strcmp(key, "canfd.brs") == 0) {
        G.canfd_brs = strtol(val, NULL, 0) ? 1 : 0;
    } else if (strcmp(key, "canfd.data_bitrate") == 0) {
        G.canfd_data_bitrate = (uint32_t)strtoul(val, NULL, 0);
    } else if (strncmp(key, "sync.", 5) == 0) {
        unsigned long pgn = strtoul(key + 5, NULL, 0);
        unsigned long period = strtoul(val, NULL, 0);
        if (pgn != 0 && period >= 5 && G.sync_count < CONFIG_MAX_SYNC_ITEMS) {
            G.sync_items[G.sync_count].pgn = (uint32_t)pgn;
            G.sync_items[G.sync_count].period_ms = (uint32_t)period;
            G.sync_count++;
        }
    } else {
        log_warn("canhost.ini: unknown key '%s'", key);
    }
}

static int load_from_path(const char *path)
{
    if (ini_parse_file(path, apply_kv, NULL) != 0) {
        return -1;
    }
    snprintf(source_path, sizeof(source_path), "%s", path);
    return 0;
}

int config_init(void)
{
    if (initialized) {
        return 0;
    }
    initialized = 1;

    char path[512];
    const char *env = getenv("CANHOST_CONF");
    if (env && *env && ini_expand_home(env, path, sizeof(path)) == 0 && load_from_path(path) == 0) {
        return 0;
    }
    if (load_from_path("canhost.ini") == 0) {
        return 0;
    }
    if (ini_expand_home("~/.canhost.ini", path, sizeof(path)) == 0 && load_from_path(path) == 0) {
        return 0;
    }
    if (load_from_path("/etc/canhost.ini") == 0) {
        return 0;
    }
    return 0;   /* defaults */
}

const char *config_get_interface(void)
{
    return G.interface;
}

const char *config_get_source(void)
{
    return source_path[0] ? source_path : NULL;
}

void config_log_summary(void)
{
    const char *src = config_get_source();
    if (src) {
        log_info("Config: %s | interface=%s", src, G.interface);
    } else {
        log_info("Config: defaults | interface=%s", G.interface);
    }

    char nodes_buf[CONFIG_MAX_NODES * 4 + 1];
    size_t off = 0;
    nodes_buf[0] = '\0';
    for (int i = 0; i < G.target_node_count && off < sizeof(nodes_buf); i++) {
        int n = snprintf(nodes_buf + off, sizeof(nodes_buf) - off, "%s%u", i ? "," : "", G.target_nodes[i]);
        if (n < 0 || (size_t)n >= sizeof(nodes_buf) - off) {
            break;
        }
        off += (size_t)n;
    }
    if (G.target_node_count > 1) {
        log_info("Target nodes: [%s]", nodes_buf);
    } else if (G.target_node_count == 1) {
        log_info("Target node: %s", nodes_buf);
    }

    if (G.sync_count > 0) {
        log_info("Sync: sa=0x%02X items=%d", (unsigned)G.sync_sa, G.sync_count);
    }
    log_info("CAN FD: enable=%d brs=%d data_bitrate=%u", G.canfd_enable, G.canfd_brs,
             (unsigned)G.canfd_data_bitrate);
}

int config_get_sync_items(config_sync_item_t *items, int max_count)
{
    if (!items || max_count <= 0) {
        return 0;
    }
    int n = G.sync_count < max_count ? G.sync_count : max_count;
    memcpy(items, G.sync_items, (size_t)n * sizeof(items[0]));
    return n;
}

int config_get_sync_count(void)
{
    return G.sync_count;
}

int config_get_target_nodes(uint8_t *nodes, int max_count)
{
    if (!nodes || max_count <= 0) {
        return 0;
    }
    int n = G.target_node_count < max_count ? G.target_node_count : max_count;
    memcpy(nodes, G.target_nodes, (size_t)n);
    return n;
}

uint8_t config_get_sync_sa(void)
{
    return G.sync_sa;
}

int config_get_canfd_enable(void)
{
    return G.canfd_enable;
}

int config_get_canfd_brs(void)
{
    return G.canfd_brs;
}

uint32_t config_get_canfd_data_bitrate(void)
{
    return G.canfd_data_bitrate;
}
