/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Decoder for the HiPNUC serial byte stream: binary frames (5A A5 ...: HI91,
 * HI81, HI83) and the text lines between them (command replies, NMEA).
 *
 * Portable C99: no heap, no stdio, no global state. Copy hipnuc_data.h/.c,
 * hipnuc_internal.h and hipnuc_decoder.h/.c into your project, keep one
 * hipnuc_decoder_t per serial port and pass every received byte through
 * hipnuc_decoder_feed(), in order:
 *
 *     const uint8_t *p = buf;
 *     size_t n = count;
 *     int ev;
 *     while ((ev = hipnuc_decoder_feed(&dec, &p, &n, &imu)) != 0) {
 *         if (ev == HIPNUC_EV_DATA) { ... imu.roll ... }
 *         else { ... hipnuc_decoder_line(&dec) ... }
 *     }
 *
 * The decoder needs no time base and never blocks. A damaged frame costs only
 * itself: the bytes it swallowed are scanned again, so good frames and reply
 * lines inside its span are still found.
 */

#ifndef HIPNUC_DECODER_H
#define HIPNUC_DECODER_H

#include <stddef.h>
#include <stdint.h>

#include "hipnuc_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HIPNUC_FRAME_MAX 512U   /* largest frame: 6-byte header + 506-byte payload */
#define HIPNUC_LINE_MAX  127U   /* longest text line kept; longer lines are dropped */

/* hipnuc_decoder_feed() results */
#define HIPNUC_EV_DATA 1        /* *out holds a newly decoded frame */
#define HIPNUC_EV_LINE 2        /* hipnuc_decoder_line() holds a newly completed text line */

typedef struct {
    /* Read-only counters */
    uint32_t bytes;             /* bytes taken from the input */
    uint32_t frames;            /* frames decoded */
    uint32_t crc_errors;        /* damaged frames: wrong CRC, impossible length, or cut off (flush) */
    uint32_t unsupported;       /* frames with a correct CRC that this SDK does not decode */
    uint16_t frame_size;        /* size in bytes of the last decoded frame, header included */

    /* Private state; all zero is a valid initial state */
    uint16_t lo, n;             /* window: win[lo .. lo + n) */
    uint32_t pos;               /* stream position of win[lo] */
    uint32_t span_end;          /* end of the latest rejected candidate */
    uint32_t taint_end;         /* end of the latest rejected candidate that looked like a real frame */
    uint8_t  span_open;         /* bytes before span_end are being scanned again */
    uint8_t  taint_open;        /* taint_end is still ahead */
    uint8_t  give_up;           /* hipnuc_decoder_flush() was called */
    uint8_t  line_len;
    uint8_t  line_over;         /* the current line is too long: drop it */
    char     line[HIPNUC_LINE_MAX + 1];
    uint8_t  win[HIPNUC_FRAME_MAX];
} hipnuc_decoder_t;

/* Reset the decoder and its counters. A zero-filled object is equivalent. */
void hipnuc_decoder_init(hipnuc_decoder_t *dec);

/*
 * Take bytes from *buf until the first event, advance *buf and *len past
 * them, and return that event: HIPNUC_EV_DATA or HIPNUC_EV_LINE. Returns 0
 * when the input is used up without an event. Call it again until it
 * returns 0: one call reports one event, and bytes that a damaged frame had
 * swallowed are reported on the calls that follow, even with *len == 0.
 *
 * out    written only when HIPNUC_EV_DATA is returned (cleared first, then
 *        filled); may be NULL to check and count frames without decoding.
 *
 * Text: bytes outside frames form lines. Printable characters (0x20-0x7E)
 * are collected, CR or LF ends a non-empty line, any other byte discards the
 * line so far, and a line longer than HIPNUC_LINE_MAX is dropped.
 */
int hipnuc_decoder_feed(hipnuc_decoder_t *dec, const uint8_t **buf, size_t *len, hipnuc_data_t *out);

/* The line reported by the last HIPNUC_EV_LINE, without CR/LF. Valid until
 * the next hipnuc_decoder_feed() or hipnuc_decoder_flush() call. */
const char *hipnuc_decoder_line(const hipnuc_decoder_t *dec);

/* Give up a frame that is still incomplete because its remaining bytes will
 * never come; its bytes are scanned again by the following
 * hipnuc_decoder_feed() calls with *len == 0, which report what they hold.
 * A call with new bytes cancels the flush. Call it only after the line has
 * been quiet for at least 50 ms: during a transfer it destroys the frame in
 * progress. */
void hipnuc_decoder_flush(hipnuc_decoder_t *dec);

/* Bytes taken from the input that are still held after the last reported
 * event (they belong to frames or lines reported later). */
size_t hipnuc_decoder_backlog(const hipnuc_decoder_t *dec);

/* Check and decode exactly one whole frame (header, CRC, payload); no state.
 * Returns 1, -1 (not one frame of that length), -2 (wrong CRC) or -3 (a
 * frame this SDK does not decode). out is written only on 1. */
int hipnuc_frame_decode(const uint8_t *frame, size_t len, hipnuc_data_t *out);

/* Find KEY=VALUE in a reply line ("PNAME=HI05", or "MSG=HI91, ONTIME=10")
 * and copy VALUE (up to a comma or the end of the line) into value[size].
 * The key must start the line or follow a space or comma. Returns value, or
 * NULL when the key is absent or the value does not fit. */
const char *hipnuc_line_value(const char *line, const char *key, char *value, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* HIPNUC_DECODER_H */
