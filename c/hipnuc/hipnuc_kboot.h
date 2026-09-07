/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial firmware update client for the HiPNUC bootloader (kboot / KPTL
 * framing). Portable C99: the caller supplies serial read/write callbacks.
 *
 * Flow (see hipnuc_kboot_update()):
 *   application "REBOOT BL" (sent by the caller) -> ping -> get properties
 *   -> flash erase region -> write memory + data packets -> reset.
 *
 * Every command reply is checked for framing, CRC, response tag, parameter
 * count, status code and echoed command tag. Any failure stops the update;
 * nothing is retried automatically.
 */

#ifndef HIPNUC_KBOOT_H
#define HIPNUC_KBOOT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HIPNUC_KBOOT_OK           = 0,
    HIPNUC_KBOOT_ERR_PARAM    = -1,   /* bad argument or missing callback */
    HIPNUC_KBOOT_ERR_WRITE    = -2,   /* serial write failed */
    HIPNUC_KBOOT_ERR_TIMEOUT  = -3,   /* no or incomplete reply in time */
    HIPNUC_KBOOT_ERR_FRAME    = -4,   /* bad start byte, CRC, length or packet type */
    HIPNUC_KBOOT_ERR_NAK      = -5,   /* device answered NAK */
    HIPNUC_KBOOT_ERR_ABORT    = -6,   /* device answered AckAbort */
    HIPNUC_KBOOT_ERR_RESPONSE = -7,   /* response tag, parameter count or command echo mismatch */
    HIPNUC_KBOOT_ERR_STATUS   = -8,   /* device reported a nonzero status code (see ctx->last_status) */
    HIPNUC_KBOOT_ERR_SIZE     = -9    /* image or packet too large for the device */
} hipnuc_kboot_status_t;

/* Serial port callbacks supplied by the application. */
typedef struct {
    /* Write `len` bytes; return the number written or < 0 on failure. */
    int  (*write)(void *user, const uint8_t *data, size_t len);
    /* Read up to `cap` bytes, waiting at most `timeout_ms` for the first
     * byte; return bytes read, 0 on timeout, < 0 on failure. */
    int  (*read)(void *user, uint8_t *data, size_t cap, uint32_t timeout_ms);
    /* Optional: discard pending received bytes. */
    void (*flush)(void *user);
    /* Optional: block for `ms` milliseconds. */
    void (*delay_ms)(void *user, uint32_t ms);
    /* Optional: called with bytes written so far and the image size. */
    void (*progress)(void *user, uint32_t written, uint32_t total);
    /* Optional: human readable diagnostics, one message per call. */
    void (*log)(void *user, const char *message);
    void *user;
} hipnuc_kboot_port_t;

typedef struct {
    hipnuc_kboot_port_t port;
    uint32_t command_timeout_ms;   /* generic commands, default 1000 */
    uint32_t erase_timeout_ms;     /* flash erase, default 10000 */
    uint32_t data_timeout_ms;      /* each data packet, default 2000 */
    uint32_t ping_timeout_ms;      /* default 50 */
    uint8_t  ping_retries;         /* default 10 */

    /* Filled by hipnuc_kboot_connect() */
    uint32_t max_packet_size;      /* property 0x0B */
    uint32_t flash_size;           /* property 0x04 */
    uint32_t flash_sector_size;    /* property 0x05 */
    uint32_t bootloader_version;   /* property 0x10 */

    uint32_t last_status;          /* status code of the last GenericResponse */
} hipnuc_kboot_ctx_t;

/* Initialize with defaults and the given port. */
void hipnuc_kboot_init(hipnuc_kboot_ctx_t *ctx, const hipnuc_kboot_port_t *port);

/* Single ping exchange (no retries). */
int hipnuc_kboot_ping(hipnuc_kboot_ctx_t *ctx);

/* Read a bootloader property (GetProperty). */
int hipnuc_kboot_get_property(hipnuc_kboot_ctx_t *ctx, uint32_t property, uint32_t *value);

/* Ping with retries, then read the properties into ctx. */
int hipnuc_kboot_connect(hipnuc_kboot_ctx_t *ctx);

int hipnuc_kboot_erase_region(hipnuc_kboot_ctx_t *ctx, uint32_t address, uint32_t length);

/* WriteMemory command followed by the data phase; checks every ACK and the
 * final GenericResponse. */
int hipnuc_kboot_write_memory(hipnuc_kboot_ctx_t *ctx, uint32_t address, const uint8_t *image, uint32_t length);

int hipnuc_kboot_reset(hipnuc_kboot_ctx_t *ctx);

/* connect + erase + write + reset. Stops at the first failure. */
int hipnuc_kboot_update(hipnuc_kboot_ctx_t *ctx, uint32_t address, const uint8_t *image, uint32_t length);

/* Text for a hipnuc_kboot_status_t value. */
const char *hipnuc_kboot_strerror(int status);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_KBOOT_H */
