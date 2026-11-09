/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * HiPNUC device on a desktop serial port; see hipnuc_serial.h.
 */

#include "hipnuc_serial.h"
#include "hipnuc_serial_io.h"
#include "hipnuc_nmea.h"

#include <stdio.h>
#include <string.h>

#define QUIET_MS 50U          /* this long without bytes: give up an incomplete frame */
#define COMMAND_MAX 126U
#define COMMAND_WAIT_MS 1000

/* One event from the bytes already read. 0 when they are used up. */
static int next_event(hipnuc_serial_t *dev, hipnuc_data_t *out)
{
    const uint8_t *p = dev->rx + dev->rx_pos;
    size_t n = dev->rx_len - dev->rx_pos;
    int ev = hipnuc_decoder_feed(&dev->decoder, &p, &n, out);
    dev->rx_pos = dev->rx_len - n;
    return ev;
}

/* Read more bytes, waiting up to wait_ms. 1 when bytes came, 0 when not, -1 on error.
 * After QUIET_MS without bytes the frame the decoder still waits for is given
 * up, once the port holds no more bytes either. */
static int fill(hipnuc_serial_t *dev, int wait_ms)
{
    int got, quiet_long = 0;
    if (hipnuc_decoder_backlog(&dev->decoder) > 0 && !dev->flushed) {
        uint64_t quiet = hipnuc_serial_io_ms() - dev->last_byte_ms;
        if (quiet >= QUIET_MS) {
            quiet_long = 1;
            wait_ms = 0;   /* bytes the port already holds come first */
        } else if (wait_ms > (int)(QUIET_MS - quiet)) {
            wait_ms = (int)(QUIET_MS - quiet);
        }
    }
    got = hipnuc_serial_io_read(dev, dev->rx, sizeof(dev->rx), wait_ms);
    if (got < 0) return -1;
    if (got == 0) {
        if (!quiet_long) return 0;
        hipnuc_decoder_flush(&dev->decoder);
        dev->flushed = 1;
        return 1;   /* the decoder has events to give now */
    }
    dev->rx_pos = 0;
    dev->rx_len = (size_t)got;
    dev->bytes_read += (uint64_t)got;
    dev->last_byte_ms = hipnuc_serial_io_ms();
    dev->flushed = 0;
    return 1;
}

static int remaining(uint64_t deadline)
{
    uint64_t now = hipnuc_serial_io_ms();
    return now >= deadline ? 0 : (int)(deadline - now);
}

int hipnuc_serial_open(hipnuc_serial_t *dev, const char *port, int baud)
{
    if (!dev) return -1;
    if (dev->is_open) {
        snprintf(dev->error, sizeof(dev->error), "The port is already open");
        return -1;
    }
    if (!port || !port[0] || baud <= 0) {
        snprintf(dev->error, sizeof(dev->error), "Invalid port name or baud rate");
        return -1;
    }
    hipnuc_decoder_init(&dev->decoder);
    dev->bytes_read = dev->rx_errors = 0;
    dev->nmea_errors = 0;
    dev->error_snapshot = 0;
    dev->rx_pos = dev->rx_len = dev->reply_len = 0;
    dev->flushed = 0;
    dev->error[0] = dev->reply[0] = '\0';
    if (hipnuc_serial_io_open(dev, port, baud) != 0) return -1;
    dev->last_byte_ms = hipnuc_serial_io_ms();
    return 0;
}

int hipnuc_serial_read(hipnuc_serial_t *dev, hipnuc_data_t *out, int timeout_ms)
{
    uint64_t deadline;
    hipnuc_data_t scratch;
    int filled = 0;
    if (!dev || !dev->is_open || timeout_ms < 0) {
        if (dev) snprintf(dev->error, sizeof(dev->error), "The port is not open, or the timeout is negative");
        return -1;
    }
    if (!out) out = &scratch;
    deadline = hipnuc_serial_io_ms() + (uint64_t)timeout_ms;
    for (;;) {
        int ev, r;
        while ((ev = next_event(dev, out)) != 0) {
            if (ev == HIPNUC_EV_DATA) return 1;
            r = hipnuc_nmea_decode(hipnuc_decoder_line(&dev->decoder), out);
            if (r == 1) return 1;
            if (r < 0) dev->nmea_errors++;
        }
        if (filled && remaining(deadline) == 0) return 0;   /* bytes may keep coming: the deadline still holds */
        if (fill(dev, remaining(deadline)) < 0) return -1;
        filled = 1;
    }
}

/* Lines of the reply are kept one per row in dev->reply. */
static void keep_line(hipnuc_serial_t *dev, const char *line)
{
    size_t n = strlen(line);
    if (dev->reply_len + n + 2 > sizeof(dev->reply)) return;   /* full: the rest is dropped */
    memcpy(dev->reply + dev->reply_len, line, n);
    dev->reply_len += n;
    dev->reply[dev->reply_len++] = '\n';
    dev->reply[dev->reply_len] = '\0';
}

int hipnuc_serial_command(hipnuc_serial_t *dev, const char *command, int timeout_ms)
{
    char line[COMMAND_MAX + 5];
    size_t n, sent = 0, i;
    uint64_t deadline;
    hipnuc_data_t scratch;

    if (!dev || !dev->is_open) {
        if (dev) snprintf(dev->error, sizeof(dev->error), "The port is not open");
        return -1;
    }
    dev->error[0] = '\0';
    n = command ? strlen(command) : 0;
    for (i = 0; i < n; ++i) {
        if (command[i] < 0x20 || command[i] > 0x7E) break;
    }
    if (n == 0 || n > COMMAND_MAX || i != n) {
        snprintf(dev->error, sizeof(dev->error), "A command is 1 to %u printable characters", COMMAND_MAX);
        return -1;
    }
    if (timeout_ms < 0) {
        snprintf(dev->error, sizeof(dev->error), "The timeout is negative");
        return -1;
    }

    /* What arrived before the command, such as a late answer to an earlier
     * one, does not belong to its reply. */
    while (next_event(dev, &scratch) != 0) {
    }
    if (fill(dev, 0) < 0) return -1;   /* and what the port already holds */
    while (next_event(dev, &scratch) != 0) {
    }
    dev->reply_len = 0;
    dev->reply[0] = '\0';

    /* The leading CR LF ends whatever partial line the device holds. */
    n = (size_t)snprintf(line, sizeof(line), "\r\n%s\r\n", command);
    deadline = hipnuc_serial_io_ms() + 100U;
    while (sent < n) {
        size_t done = hipnuc_serial_io_write(dev, (const uint8_t *)line + sent, n - sent);
        sent += done;
        if (done == 0 && remaining(deadline) == 0) {
            if (!dev->error[0]) snprintf(dev->error, sizeof(dev->error), "The port did not take the command");
            return -1;
        }
    }

    deadline = hipnuc_serial_io_ms() + (uint64_t)(timeout_ms ? timeout_ms : COMMAND_WAIT_MS);
    for (;;) {
        int ev;
        while ((ev = next_event(dev, &scratch)) != 0) {
            const char *l;
            if (ev != HIPNUC_EV_LINE) continue;
            l = hipnuc_decoder_line(&dev->decoder);
            if (strcmp(l, "OK") == 0) return 0;
            if (strcmp(l, "ERR") == 0) return 1;
            if (l[0] != '$') keep_line(dev, l);   /* NMEA output is not part of the reply */
        }
        if (remaining(deadline) == 0) {
            snprintf(dev->error, sizeof(dev->error), "No OK or ERR for \"%s\" within %d ms", command,
                     timeout_ms ? timeout_ms : COMMAND_WAIT_MS);
            return -1;
        }
        if (fill(dev, remaining(deadline)) < 0) return -1;
    }
}

const char *hipnuc_serial_reply_value(hipnuc_serial_t *dev, const char *key)
{
    const char *row;
    if (!dev || !key) return NULL;
    for (row = dev->reply; *row;) {
        const char *end = strchr(row, '\n');
        char one[HIPNUC_LINE_MAX + 1];
        size_t len = end ? (size_t)(end - row) : strlen(row);
        if (len < sizeof(one)) {
            memcpy(one, row, len);
            one[len] = '\0';
            if (hipnuc_line_value(one, key, dev->value, sizeof(dev->value))) return dev->value;
        }
        if (!end) break;
        row = end + 1;
    }
    return NULL;
}

size_t hipnuc_serial_pending(const hipnuc_serial_t *dev)
{
    if (!dev) return 0;
    return dev->rx_len - dev->rx_pos + hipnuc_decoder_backlog(&dev->decoder);
}

const char *hipnuc_serial_last_error(const hipnuc_serial_t *dev)
{
    return dev && dev->error[0] ? dev->error : "No error";
}

void hipnuc_serial_close(hipnuc_serial_t *dev)
{
    if (dev) hipnuc_serial_io_close(dev);
}
