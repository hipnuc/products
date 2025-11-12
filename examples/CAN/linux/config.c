#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "log.h"

typedef struct {
    char interface[32];
} canhost_config_t;

static canhost_config_t G = { .interface = "can0" };
static int initialized = 0;
static char source_path[256] = {0};

static char* trim(char *s)
{
    if (!s) return s;
    // left trim
    while (*s == ' ' || *s == '\t' || *s == '\r') s++;
    // right trim
    char *end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) --end;
    *end = '\0';
    return s;
}

static void apply_kv(const char *key_in, const char *val_in)
{
    if (!key_in || !val_in) return;
    char key[64];
    char val[128];
    snprintf(key, sizeof(key), "%s", key_in);
    snprintf(val, sizeof(val), "%s", val_in);
    // normalize
    for (char *p = key; *p; ++p) {
        if (*p >= 'A' && *p <= 'Z') *p = (char)(*p - 'A' + 'a');
    }
    if (strcmp(key, "interface") == 0) {
        snprintf(G.interface, sizeof(G.interface), "%.*s", (int)(sizeof(G.interface) - 1), val);
    }
}

static int load_from_path(const char *path)
{
    if (!path || !*path) return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        // strip newline
        char *nl = strchr(p, '\n');
        if (nl) *nl = '\0';
        // skip comments and empty
        p = trim(p);
        if (!*p || *p == '#' || *p == ';') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);
        apply_kv(key, val);
    }
    fclose(f);
    // record source path when successfully parsed
    snprintf(source_path, sizeof(source_path), "%s", path);
    return 0;
}

static int expand_home_path(const char *in, char *out, size_t outlen)
{
    if (!in || !out || outlen == 0) return -1;
    if (in[0] == '~') {
        const char *home = getenv("HOME");
        if (!home) return -1;
        int n = snprintf(out, outlen, "%s%s", home, in + 1);
        return (n >= 0 && (size_t)n < outlen) ? 0 : -1;
    }
    int n = snprintf(out, outlen, "%s", in);
    return (n >= 0 && (size_t)n < outlen) ? 0 : -1;
}

int config_init(void)
{
    if (initialized) return 0;
    // 1) env var
    const char *envp = getenv("CANHOST_CONF");
    if (envp && *envp) {
        char path[256];
        if (expand_home_path(envp, path, sizeof(path)) == 0) {
            if (load_from_path(path) == 0) {
                initialized = 1;
                return 0;
            }
        }
    }
    // 2) ./canhost.ini
    #ifdef CANHOST_SOURCE_DIR
    {
        char src_path[256];
        int n = snprintf(src_path, sizeof(src_path), "%s/canhost.ini", CANHOST_SOURCE_DIR);
        if (n >= 0 && (size_t)n < sizeof(src_path)) {
            if (load_from_path(src_path) == 0) { initialized = 1; return 0; }
        }
    }
    #endif
    if (load_from_path("canhost.ini") == 0) { initialized = 1; return 0; }
    // 3) ~/.canhost.ini
    char home_path[256];
    if (expand_home_path("~/.canhost.ini", home_path, sizeof(home_path)) == 0) {
        if (load_from_path(home_path) == 0) { initialized = 1; return 0; }
    }
    // 4) /etc/canhost.ini
    if (load_from_path("/etc/canhost.ini") == 0) { initialized = 1; return 0; }
    initialized = 1;
    return 0;
}

const char* config_get_interface(void)
{
    return G.interface;
}


const char* config_get_source(void)
{
    return (source_path[0] != '\0') ? source_path : NULL;
}

void config_log_summary(void)
{
    const char *src = config_get_source();
    if (src) {
        log_info("Config loaded: %s | interface=%s", src, G.interface);
    } else {
        log_info("Config defaults used | interface=%s", G.interface);
    }
}
