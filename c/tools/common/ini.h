/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal INI reader shared by hihost and canhost.
 *
 * Format: one `key=value` per line; keys are lower-cased; whitespace around
 * keys and values is removed; lines starting with `#` or `;` and anything
 * after an unquoted ` #` / `;` on a line are comments. Sections (`[name]`)
 * are ignored.
 */

#ifndef HIPNUC_TOOLS_INI_H
#define HIPNUC_TOOLS_INI_H

#include <stddef.h>

/* Called once per key/value pair. */
typedef void (*ini_kv_fn)(void *user, const char *key, const char *value);

/* Strip comments and surrounding whitespace in place; returns the trimmed
 * start of the string. */
char *ini_trim(char *s);

/* Parse `path`. Returns 0 when the file was read, -1 when it could not be
 * opened. */
int ini_parse_file(const char *path, ini_kv_fn fn, void *user);

/* Expand a leading `~` with $HOME into `out`. Returns 0, or -1 when HOME is
 * unset or the result does not fit. */
int ini_expand_home(const char *in, char *out, size_t out_size);

#endif /* HIPNUC_TOOLS_INI_H */
