// cmd_utils.c
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int safe_sleep(unsigned long usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;

    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}

static char* trim(char *s) {
    if (!s) return s;
    while (*s == ' ' || *s == '\t' || *s == '\r') s++;
    char *end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r')) --end;
    *end = '\0';
    return s;
}


int ini_load_serial(const char *path, char *port_out, size_t port_out_sz, int *baud_out) {
    if (!path || !port_out || port_out_sz == 0 || !baud_out) return -1;
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[256];
    int have_port = 0, have_baud = 0;
    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        char *nl = strchr(p, '\n');
        if (nl) *nl = '\0';
        p = trim(p);
        if (!*p || *p == '#' || *p == ';') continue;
        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(p);
        char *val = trim(eq + 1);
        for (char *k = key; *k; ++k) {
            if (*k >= 'A' && *k <= 'Z') *k = (char)(*k - 'A' + 'a');
        }
        if (strcmp(key, "port") == 0) {
            if (*val) {
                snprintf(port_out, port_out_sz, "%s", val);
                have_port = 1;
            }
        } else if (strcmp(key, "baud") == 0) {
            if (*val) {
                char *endp = NULL;
                long b = strtol(val, &endp, 10);
                if (endp && *endp == '\0' && b > 0) {
                    *baud_out = (int)b;
                    have_baud = 1;
                }
            }
        }
    }
    fclose(f);
    return (have_port && have_baud) ? 0 : -1;
}

int ini_update_serial(const char *path, const char *port, int baud) {
    if (!path || !port || baud <= 0) return -1;
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    fprintf(f, "# HiHost runtime configuration\n");
    fprintf(f, "port=%s\n", port);
    fprintf(f, "baud=%d\n", baud);
    fclose(f);
    return 0;
}