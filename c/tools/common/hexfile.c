/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Intel HEX / binary firmware image loader. See hexfile.h.
 */

#include "hexfile.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEX_MAX_IMAGE_SIZE (64U * 1024U * 1024U)   /* refuse absurd address spans */

static int hex_nibble(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int hex_byte(const char *s, uint8_t *out)
{
    int hi = hex_nibble((unsigned char)s[0]);
    int lo = hex_nibble((unsigned char)s[1]);
    if (hi < 0 || lo < 0) return -1;
    *out = (uint8_t)((hi << 4) | lo);
    return 0;
}

/* ":" + 2*n hex digits + optional CR/LF -> n bytes. */
static int parse_hex_line(const char *line, uint8_t *bytes, size_t max_bytes, size_t *out_len)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        len--;
    }
    if (len < 1 || line[0] != ':' || ((len - 1U) % 2U) != 0U) {
        return -1;
    }
    size_t n = (len - 1U) / 2U;
    if (n > max_bytes) {
        return -1;
    }
    for (size_t i = 0; i < n; ++i) {
        if (hex_byte(&line[1U + i * 2U], &bytes[i]) < 0) {
            return -1;
        }
    }
    *out_len = n;
    return 0;
}

/* Grow the image to `needed` bytes, filling new space with 0xFF. */
static int ensure_capacity(hex_image_t *img, uint32_t needed)
{
    if (needed <= img->size) {
        return 0;
    }
    if (needed > HEX_MAX_IMAGE_SIZE) {
        fprintf(stderr, "hexfile: image larger than %u bytes\n", (unsigned)HEX_MAX_IMAGE_SIZE);
        return -1;
    }
    uint8_t *next = realloc(img->data, needed);
    if (!next) {
        fprintf(stderr, "hexfile: out of memory\n");
        return -1;
    }
    memset(next + img->size, 0xFF, needed - img->size);
    img->data = next;
    img->size = needed;
    return 0;
}

int hexfile_load(const char *path, hex_image_t *img)
{
    FILE *fp;
    char line[1024];
    uint32_t upper = 0;
    uint32_t origin = 0;
    int have_origin = 0;
    int line_no = 0;
    int ok = 0;

    if (!path || !img) {
        return -1;
    }
    memset(img, 0, sizeof(*img));

    fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "hexfile: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }

    while (fgets(line, sizeof(line), fp)) {
        uint8_t bytes[300];
        size_t n = 0;
        uint8_t data_len, type, sum = 0;
        uint16_t addr;

        line_no++;
        /* Skip blank lines (some tools leave a trailing empty line). */
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') {
            continue;
        }
        if (parse_hex_line(line, bytes, sizeof(bytes), &n) < 0 || n < 5) {
            fprintf(stderr, "hexfile: invalid record at line %d\n", line_no);
            goto out;
        }
        data_len = bytes[0];
        if (n != (size_t)data_len + 5U) {
            fprintf(stderr, "hexfile: length mismatch at line %d\n", line_no);
            goto out;
        }
        for (size_t i = 0; i < n; ++i) {
            sum = (uint8_t)(sum + bytes[i]);
        }
        if (sum != 0) {
            fprintf(stderr, "hexfile: checksum failed at line %d\n", line_no);
            goto out;
        }

        addr = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[2]);
        type = bytes[3];

        switch (type) {
        case 0x00: {   /* data */
            uint32_t abs_addr = upper + addr;
            uint32_t offset;
            if (!have_origin) {
                origin = abs_addr;
                img->start_addr = origin;
                have_origin = 1;
            }
            if (abs_addr < origin + img->size) {
                fprintf(stderr, "hexfile: records must be ascending and must not overlap (line %d)\n", line_no);
                goto out;
            }
            offset = abs_addr - origin;
            if (offset + data_len < offset) {
                fprintf(stderr, "hexfile: address overflow at line %d\n", line_no);
                goto out;
            }
            if (ensure_capacity(img, offset + data_len) < 0) {
                goto out;
            }
            memcpy(img->data + offset, &bytes[4], data_len);
            break;
        }
        case 0x01:     /* end of file */
            ok = img->size > 0U;
            if (!ok) {
                fprintf(stderr, "hexfile: no data records before end of file\n");
            }
            goto out;
        case 0x02:     /* extended segment address */
            if (data_len != 2U) {
                fprintf(stderr, "hexfile: bad type 02 record at line %d\n", line_no);
                goto out;
            }
            upper = (uint32_t)(((uint16_t)bytes[4] << 8) | bytes[5]) << 4;
            break;
        case 0x04:     /* extended linear address */
            if (data_len != 2U) {
                fprintf(stderr, "hexfile: bad type 04 record at line %d\n", line_no);
                goto out;
            }
            upper = (uint32_t)(((uint16_t)bytes[4] << 8) | bytes[5]) << 16;
            break;
        case 0x03:     /* start segment address: ignored */
        case 0x05:     /* start linear address: ignored */
            break;
        default:
            fprintf(stderr, "hexfile: unsupported record type 0x%02X at line %d\n", type, line_no);
            goto out;
        }
    }

    /* Reached the end without a type 01 record. */
    if (img->size > 0U) {
        fprintf(stderr, "hexfile: missing end-of-file record\n");
    } else {
        fprintf(stderr, "hexfile: %s contains no data\n", path);
    }

out:
    fclose(fp);
    if (!ok) {
        hex_image_free(img);
        return -1;
    }
    return 0;
}

int binfile_load(const char *path, hex_image_t *img)
{
    FILE *fp;
    long len;

    if (!path || !img) {
        return -1;
    }
    memset(img, 0, sizeof(*img));

    fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "hexfile: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    if (fseek(fp, 0, SEEK_END) != 0 || (len = ftell(fp)) < 0) {
        fprintf(stderr, "hexfile: cannot determine size of %s\n", path);
        fclose(fp);
        return -1;
    }
    if (len == 0 || (unsigned long)len > HEX_MAX_IMAGE_SIZE) {
        fprintf(stderr, "hexfile: invalid firmware size %ld\n", len);
        fclose(fp);
        return -1;
    }
    rewind(fp);

    img->data = malloc((size_t)len);
    if (!img->data) {
        fprintf(stderr, "hexfile: out of memory\n");
        fclose(fp);
        return -1;
    }
    if (fread(img->data, 1, (size_t)len, fp) != (size_t)len) {
        fprintf(stderr, "hexfile: failed to read %s\n", path);
        fclose(fp);
        hex_image_free(img);
        return -1;
    }
    fclose(fp);
    img->size = (uint32_t)len;
    img->start_addr = 0;
    return 0;
}

void hex_image_free(hex_image_t *img)
{
    if (!img) {
        return;
    }
    free(img->data);
    memset(img, 0, sizeof(*img));
}
