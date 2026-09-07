/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal INI reader. See ini.h.
 */

#include "ini.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char *ini_trim(char *s)
{
    char *end;

    if (!s) {
        return s;
    }
    while (*s && isspace((unsigned char)*s)) {
        s++;
    }
    /* A full-line comment, or an inline comment introduced by whitespace. */
    if (*s == '#' || *s == ';') {
        *s = '\0';
        return s;
    }
    for (char *p = s; *p; ++p) {
        if ((*p == '#' || *p == ';') && p > s && isspace((unsigned char)p[-1])) {
            *p = '\0';
            break;
        }
    }
    end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) {
        --end;
    }
    *end = '\0';
    return s;
}

int ini_parse_file(const char *path, ini_kv_fn fn, void *user)
{
    FILE *f;
    char line[512];

    if (!path || !*path || !fn) {
        return -1;
    }
    f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    while (fgets(line, sizeof(line), f)) {
        char *p = ini_trim(line);
        char *eq;
        char *key;
        char *val;

        if (!*p || *p == '[') {
            continue;
        }
        eq = strchr(p, '=');
        if (!eq) {
            continue;
        }
        *eq = '\0';
        key = ini_trim(p);
        val = ini_trim(eq + 1);
        for (char *k = key; *k; ++k) {
            *k = (char)tolower((unsigned char)*k);
        }
        if (*key) {
            fn(user, key, val);
        }
    }
    fclose(f);
    return 0;
}

int ini_expand_home(const char *in, char *out, size_t out_size)
{
    int n;

    if (!in || !out || out_size == 0) {
        return -1;
    }
    if (in[0] == '~') {
        const char *home = getenv("HOME");
        if (!home || !*home) {
            return -1;
        }
        n = snprintf(out, out_size, "%s%s", home, in + 1);
    } else {
        n = snprintf(out, out_size, "%s", in);
    }
    return (n >= 0 && (size_t)n < out_size) ? 0 : -1;
}
