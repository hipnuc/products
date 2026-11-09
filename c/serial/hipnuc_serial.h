/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC device on a desktop serial port (Windows, Linux): 8N1, no flow
 * control, synchronous calls. Decodes binary frames and NMEA sentences into
 * hipnuc_data_t and sends ASCII commands.
 *
 *     hipnuc_serial_t dev = {0};
 *     hipnuc_data_t imu;
 *     if (hipnuc_serial_open(&dev, "COM3", 115200) == 0)
 *         while (hipnuc_serial_read(&dev, &imu, 1000) >= 0) { ... }
 *     hipnuc_serial_close(&dev);
 *
 * One owner per object: do not copy an open object or use it from two threads.
 */

#ifndef HIPNUC_SERIAL_H
#define HIPNUC_SERIAL_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_data.h"
#include "hipnuc_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Read-only. The counters start again at each open and survive close. */
    hipnuc_decoder_t decoder;   /* decoder.frames, .crc_errors, .unsupported, .frame_size */
    int is_open;
    uint64_t bytes_read;        /* bytes taken from the port */
    uint64_t rx_errors;         /* overrun, framing and parity events the driver reported (bytes were lost) */
    uint32_t nmea_errors;       /* GGA/RMC sentences with a bad checksum or field */

    /* Private */
    intptr_t handle;
    uint32_t error_snapshot;
    uint64_t last_byte_ms;
    int flushed;
    size_t rx_pos, rx_len;
    size_t reply_len;
    char error[256];
    char value[128];
    char reply[512];
    uint8_t rx[1024];
} hipnuc_serial_t;

/* Explicit port ("COM3", "/dev/ttyUSB0") and baud rate. Returns 0, or -1 with
 * the reason in hipnuc_serial_last_error(). Opening does not check that a
 * device answers. On Linux the port is opened exclusively and input received
 * before opening is discarded. */
int hipnuc_serial_open(hipnuc_serial_t *dev, const char *port, int baud);

/* Returns 1 with one new message in *out (a binary frame or a GGA/RMC
 * sentence), 0 when none arrived within timeout_ms, -1 on a port error.
 * timeout_ms is the whole call's deadline; 0 returns at once. Bytes read
 * beyond the message stay in the object for the next call. */
int hipnuc_serial_read(hipnuc_serial_t *dev, hipnuc_data_t *out, int timeout_ms);

/* Sends one ASCII command (1..126 printable characters, without CR/LF) and
 * waits up to timeout_ms (0: 1000 ms) for its OK or ERR. Returns 0 for OK, 1
 * for ERR (the device answered and refused), -1 without an answer or on a port
 * error (hipnuc_serial_last_error() says which). Messages that arrive while
 * waiting are dropped. Commands without an answer (REBOOT) return -1. */
int hipnuc_serial_command(hipnuc_serial_t *dev, const char *command, int timeout_ms);

/* VALUE of KEY=VALUE in the last command's reply ("PNAME", "APP_VER", ...),
 * or NULL. Valid until the next call on this object. */
const char *hipnuc_serial_reply_value(hipnuc_serial_t *dev, const char *key);

/* Bytes received but not yet reported: read from the port, or held by the
 * decoder after the last message. Divide by the byte rate to back-date. */
size_t hipnuc_serial_pending(const hipnuc_serial_t *dev);

/* The reason for the last -1; valid until the next call on this object. */
const char *hipnuc_serial_last_error(const hipnuc_serial_t *dev);

/* Releases the port. Safe on a zero-filled or closed object; keeps the
 * counters and the last error. */
void hipnuc_serial_close(hipnuc_serial_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_SERIAL_H */
