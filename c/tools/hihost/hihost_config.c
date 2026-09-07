#include "hihost_config.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ini.h"

int safe_sleep(unsigned long usec)
{
    struct timespec ts;
    ts.tv_sec = (time_t)(usec / 1000000UL);
    ts.tv_nsec = (long)(usec % 1000000UL) * 1000L;
    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}

typedef struct {
    char *port;
    size_t port_size;
    int baud;
    int have_port;
    int have_baud;
} serial_kv_t;

static void on_kv(void *user, const char *key, const char *value)
{
    serial_kv_t *c = (serial_kv_t *)user;
    if (strcmp(key, "port") == 0 && *value) {
        snprintf(c->port, c->port_size, "%s", value);
        c->have_port = 1;
    } else if (strcmp(key, "baud") == 0 && *value) {
        char *end = NULL;
        long b = strtol(value, &end, 10);
        if (end && *end == '\0' && b > 0) {
            c->baud = (int)b;
            c->have_baud = 1;
        }
    }
}

static int try_load(const char *path, serial_kv_t *c)
{
    c->have_port = 0;
    c->have_baud = 0;
    if (!path || !*path || ini_parse_file(path, on_kv, c) != 0) {
        return -1;
    }
    return (c->have_port && c->have_baud) ? 0 : -1;
}

int hihost_config_load(char *port_out, size_t port_size, int *baud_out,
                       char *source, size_t source_size)
{
    serial_kv_t c = { port_out, port_size, 0, 0, 0 };
    char path[512];
    const char *env = getenv("HIHOST_CONF");
    const char *candidates[3];
    int n = 0;

    if (!port_out || port_size == 0 || !baud_out) {
        return -1;
    }
    if (env && *env) {
        candidates[n++] = env;
    }
    candidates[n++] = HIHOST_INI_FILE;
    if (ini_expand_home("~/." HIHOST_INI_FILE, path, sizeof(path)) == 0) {
        candidates[n++] = path;
    }
    for (int i = 0; i < n; ++i) {
        if (try_load(candidates[i], &c) == 0) {
            *baud_out = c.baud;
            if (source && source_size) {
                snprintf(source, source_size, "%s", candidates[i]);
            }
            return 0;
        }
    }
    return -1;
}

int hihost_config_save(const char *path, const char *port, int baud)
{
    FILE *f;
    if (!path || !port || baud <= 0) {
        return -1;
    }
    f = fopen(path, "w");
    if (!f) {
        return -1;
    }
    fprintf(f, "# hihost configuration, written by `hihost probe --save`\n");
    fprintf(f, "port=%s\n", port);
    fprintf(f, "baud=%d\n", baud);
    return fclose(f) == 0 ? 0 : -1;
}
