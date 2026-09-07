/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Firmware image loader for the HiPNUC command line tools: Intel HEX and
 * raw binary files. Strict parsing: every HEX record must have a valid
 * checksum and length, data records must arrive in ascending address order
 * and only record types 00-05 are accepted. Gaps between data records are
 * filled with 0xFF (erased flash).
 */

#ifndef HIPNUC_TOOLS_HEXFILE_H
#define HIPNUC_TOOLS_HEXFILE_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;          /* malloc'd image, hex_image_free() releases it */
    uint32_t size;          /* bytes in data */
    uint32_t start_addr;    /* absolute address of data[0]; 0 for binary files */
} hex_image_t;

/* Load an Intel HEX file. Returns 0 on success, -1 on any error (a message
 * is written to stderr). On failure *img is left empty. */
int hexfile_load(const char *path, hex_image_t *img);

/* Load a raw binary file; start_addr is set to 0. Returns 0 or -1. */
int binfile_load(const char *path, hex_image_t *img);

/* Release the image data and clear the structure. Safe on an empty image. */
void hex_image_free(hex_image_t *img);

#endif /* HIPNUC_TOOLS_HEXFILE_H */
