/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-side tests for hipnuc_kboot.c and hipnuc_can_update.c using scripted
 * fake ports. No framework: CHECK() reports file:line and the process exits
 * nonzero when any check fails.
 */

#include "hipnuc_can_update.h"
#include "hipnuc_dec.h"
#include "hipnuc_kboot.h"

#include <stdio.h>
#include <string.h>

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        ++g_checks;                                                          \
        if (!(cond)) {                                                       \
            ++g_failures;                                                    \
            printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                                    \
    } while (0)

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* ------------------------------------------------------------------------ */
/* Fake serial port scripted as a kboot bootloader                          */
/* ------------------------------------------------------------------------ */

#define FAKE_RX_CAP   4096
#define FAKE_TX_CAP   8192
#define FAKE_PKT_MAX  64

enum {
    KB_ACK = 0xA1, KB_NAK = 0xA2, KB_ABORT = 0xA3, KB_COMMAND = 0xA4,
    KB_DATA = 0xA5, KB_PING = 0xA6, KB_PING_RESP = 0xA7
};
enum {
    TAG_ERASE = 0x02, TAG_WRITE = 0x04, TAG_GET_PROP = 0x07, TAG_RESET = 0x0B,
    TAG_GENERIC = 0xA0, TAG_PROP_RESP = 0xA7
};

typedef struct {
    /* bytes queued for the host to read */
    uint8_t rx[FAKE_RX_CAP];
    size_t rx_head;
    size_t rx_len;
    size_t read_chunk;          /* max bytes returned per read call */

    /* everything the host wrote, split into packets */
    uint8_t tx[FAKE_TX_CAP];
    size_t tx_len;
    size_t pkt_off[FAKE_PKT_MAX];
    size_t pkt_len[FAKE_PKT_MAX];
    int pkt_count;

    /* device model */
    uint32_t max_packet_size;
    uint32_t flash_size;
    uint32_t sector_size;
    uint32_t bl_version;
    uint32_t write_remaining;   /* bytes still expected after WriteMemory */
    int data_packets;

    /* scenario switches */
    int no_ping_reply;
    uint32_t erase_status;
    int nak_on_data;            /* 1-based data packet index, 0 = never */
    int abort_on_data;          /* 1-based data packet index, 0 = never */
    int truncate_response;      /* only 3 bytes of the framing packet follow the ACK */
    int wrong_echo;
    int bad_crc;
    int drop_final_response;    /* no GenericResponse after the last data packet */
    int read_fails;             /* read() returns -1 */

    /* callbacks seen */
    int progress_calls;
    uint32_t progress_written[8];
    uint32_t progress_total[8];
    int delay_calls;
    uint32_t delay_total_ms;
    int flush_calls;
    int log_calls;
} fake_serial_t;

static void fake_serial_reset(fake_serial_t *f)
{
    memset(f, 0, sizeof(*f));
    f->read_chunk = 3;
    f->max_packet_size = 512;
    f->flash_size = 0x40000;
    f->sector_size = 0x800;
    f->bl_version = 0x00010203;
}

static void fake_enqueue(fake_serial_t *f, const uint8_t *data, size_t len)
{
    if (f->rx_len + len > FAKE_RX_CAP) {
        printf("fake rx queue overflow\n");
        ++g_failures;
        return;
    }
    memcpy(f->rx + f->rx_head + f->rx_len, data, len);
    f->rx_len += len;
}

static void fake_enqueue_frame(fake_serial_t *f, uint8_t type, const uint8_t *payload, size_t len, int corrupt_crc,
                               int truncate)
{
    uint8_t frame[6 + 64];
    uint16_t crc;

    frame[0] = 0x5A;
    frame[1] = type;
    frame[2] = (uint8_t)(len & 0xFF);
    frame[3] = (uint8_t)(len >> 8);
    memcpy(frame + 6, payload, len);
    crc = hipnuc_crc16(0, frame, 4);
    crc = hipnuc_crc16(crc, payload, len);
    if (corrupt_crc) {
        crc ^= 0x5555;
    }
    frame[4] = (uint8_t)(crc & 0xFF);
    frame[5] = (uint8_t)(crc >> 8);
    fake_enqueue(f, frame, truncate ? 3 : 6 + len);
}

static void fake_enqueue_ack(fake_serial_t *f, uint8_t type)
{
    uint8_t ack[2];
    ack[0] = 0x5A;
    ack[1] = type;
    fake_enqueue(f, ack, 2);
}

static void fake_enqueue_generic_response(fake_serial_t *f, uint32_t status, uint32_t echo)
{
    uint8_t payload[12];
    payload[0] = TAG_GENERIC;
    payload[1] = 0;
    payload[2] = 0;
    payload[3] = 2;
    wr_u32(payload + 4, status);
    wr_u32(payload + 8, echo);
    fake_enqueue_frame(f, KB_COMMAND, payload, sizeof(payload), f->bad_crc, f->truncate_response);
}

static void fake_on_command(fake_serial_t *f, const uint8_t *payload, size_t len)
{
    uint8_t tag = payload[0];
    uint8_t count = payload[3];

    if (len < 4 + (size_t)count * 4) {
        return;
    }
    fake_enqueue_ack(f, KB_ACK);

    if (tag == TAG_GET_PROP) {
        uint8_t resp[12];
        uint32_t value = 0;
        uint32_t code = rd_u32(payload + 4);
        switch (code) {
        case 0x0B: value = f->max_packet_size; break;
        case 0x04: value = f->flash_size; break;
        case 0x05: value = f->sector_size; break;
        case 0x10: value = f->bl_version; break;
        default: break;
        }
        resp[0] = TAG_PROP_RESP;
        resp[1] = 0;
        resp[2] = 0;
        resp[3] = 2;
        wr_u32(resp + 4, 0);
        wr_u32(resp + 8, value);
        fake_enqueue_frame(f, KB_COMMAND, resp, sizeof(resp), f->bad_crc, f->truncate_response);
        return;
    }

    if (tag == TAG_WRITE) {
        f->write_remaining = rd_u32(payload + 8);
    }
    fake_enqueue_generic_response(f, tag == TAG_ERASE ? f->erase_status : 0,
                                  f->wrong_echo ? (uint32_t)(tag ^ 0x01) : tag);
}

static void fake_on_data(fake_serial_t *f, size_t len)
{
    ++f->data_packets;
    if (f->nak_on_data == f->data_packets) {
        fake_enqueue_ack(f, KB_NAK);
        return;
    }
    if (f->abort_on_data == f->data_packets) {
        fake_enqueue_ack(f, KB_ABORT);
        return;
    }
    fake_enqueue_ack(f, KB_ACK);
    if (len >= f->write_remaining) {
        f->write_remaining = 0;
        if (!f->drop_final_response) {
            fake_enqueue_generic_response(f, 0, TAG_WRITE);
        }
    } else {
        f->write_remaining -= (uint32_t)len;
    }
}

static int fake_serial_write(void *user, const uint8_t *data, size_t len)
{
    fake_serial_t *f = (fake_serial_t *)user;

    if (f->tx_len + len > FAKE_TX_CAP || f->pkt_count >= FAKE_PKT_MAX) {
        return -1;
    }
    memcpy(f->tx + f->tx_len, data, len);
    f->pkt_off[f->pkt_count] = f->tx_len;
    f->pkt_len[f->pkt_count] = len;
    ++f->pkt_count;
    f->tx_len += len;

    if (len < 2 || data[0] != 0x5A) {
        return (int)len;
    }
    if (data[1] == KB_PING) {
        if (!f->no_ping_reply) {
            uint8_t resp[10] = { 0x5A, KB_PING_RESP, 0x00, 0x00, 0x02, 'P', 0x00, 0x00, 0, 0 };
            uint16_t crc = hipnuc_crc16(0, resp, 8);
            resp[8] = (uint8_t)(crc & 0xFF);
            resp[9] = (uint8_t)(crc >> 8);
            fake_enqueue(f, resp, sizeof(resp));
        }
        return (int)len;
    }
    if (len >= 6) {
        size_t plen = (size_t)data[2] | ((size_t)data[3] << 8);
        if (6 + plen == len) {
            if (data[1] == KB_COMMAND && plen >= 4) {
                fake_on_command(f, data + 6, plen);
            } else if (data[1] == KB_DATA) {
                fake_on_data(f, plen);
            }
        }
    }
    return (int)len;
}

static int fake_serial_read(void *user, uint8_t *data, size_t cap, uint32_t timeout_ms)
{
    fake_serial_t *f = (fake_serial_t *)user;
    size_t n;

    (void)timeout_ms;
    if (f->read_fails) {
        return -1;
    }
    if (f->rx_len == 0) {
        return 0;
    }
    n = f->rx_len;
    if (n > cap) {
        n = cap;
    }
    if (n > f->read_chunk) {
        n = f->read_chunk;
    }
    memcpy(data, f->rx + f->rx_head, n);
    f->rx_head += n;
    f->rx_len -= n;
    if (f->rx_len == 0) {
        f->rx_head = 0;
    }
    return (int)n;
}

static void fake_serial_flush(void *user)
{
    fake_serial_t *f = (fake_serial_t *)user;
    ++f->flush_calls;
    /* Replies are queued synchronously by write(); a real port would have
     * nothing pending here, so the queue is left alone. */
}

static void fake_serial_delay(void *user, uint32_t ms)
{
    fake_serial_t *f = (fake_serial_t *)user;
    ++f->delay_calls;
    f->delay_total_ms += ms;
}

static void fake_serial_progress(void *user, uint32_t written, uint32_t total)
{
    fake_serial_t *f = (fake_serial_t *)user;
    if (f->progress_calls < 8) {
        f->progress_written[f->progress_calls] = written;
        f->progress_total[f->progress_calls] = total;
    }
    ++f->progress_calls;
}

static void fake_serial_log(void *user, const char *message)
{
    fake_serial_t *f = (fake_serial_t *)user;
    (void)message;
    ++f->log_calls;
}

static void kboot_setup(hipnuc_kboot_ctx_t *ctx, fake_serial_t *f)
{
    hipnuc_kboot_port_t port;
    memset(&port, 0, sizeof(port));
    port.write = fake_serial_write;
    port.read = fake_serial_read;
    port.flush = fake_serial_flush;
    port.delay_ms = fake_serial_delay;
    port.progress = fake_serial_progress;
    port.log = fake_serial_log;
    port.user = f;
    hipnuc_kboot_init(ctx, &port);
}

/* Return a pointer to TX packet `i`, or NULL. */
static const uint8_t *tx_pkt(const fake_serial_t *f, int i, size_t *len)
{
    if (i < 0 || i >= f->pkt_count) {
        *len = 0;
        return NULL;
    }
    *len = f->pkt_len[i];
    return f->tx + f->pkt_off[i];
}

/* Check that TX packet `i` is a Command with the given tag and parameters. */
static int tx_is_command(const fake_serial_t *f, int i, uint8_t tag, const uint32_t *params, uint8_t count)
{
    size_t len;
    const uint8_t *p = tx_pkt(f, i, &len);
    uint8_t k;
    uint16_t crc;

    if (!p || len != 6U + 4U + (size_t)count * 4U) return 0;
    if (p[0] != 0x5A || p[1] != KB_COMMAND) return 0;
    if (p[2] != (uint8_t)(4 + count * 4) || p[3] != 0) return 0;
    crc = hipnuc_crc16(0, p, 4);
    crc = hipnuc_crc16(crc, p + 6, len - 6);
    if (p[4] != (uint8_t)(crc & 0xFF) || p[5] != (uint8_t)(crc >> 8)) return 0;
    if (p[6] != tag || p[7] != 0 || p[8] != 0 || p[9] != count) return 0;
    for (k = 0; k < count; ++k) {
        if (rd_u32(p + 10 + (size_t)k * 4) != params[k]) return 0;
    }
    return 1;
}

static int tx_is_data(const fake_serial_t *f, int i, const uint8_t *expect, size_t n)
{
    size_t len;
    const uint8_t *p = tx_pkt(f, i, &len);
    uint16_t crc;

    if (!p || len != 6 + n) return 0;
    if (p[0] != 0x5A || p[1] != KB_DATA) return 0;
    if (p[2] != (uint8_t)(n & 0xFF) || p[3] != (uint8_t)(n >> 8)) return 0;
    crc = hipnuc_crc16(0, p, 4);
    crc = hipnuc_crc16(crc, p + 6, n);
    if (p[4] != (uint8_t)(crc & 0xFF) || p[5] != (uint8_t)(crc >> 8)) return 0;
    return memcmp(p + 6, expect, n) == 0;
}

static int tx_count_type(const fake_serial_t *f, uint8_t type)
{
    int i;
    int n = 0;
    for (i = 0; i < f->pkt_count; ++i) {
        if (f->pkt_len[i] >= 2 && f->tx[f->pkt_off[i]] == 0x5A && f->tx[f->pkt_off[i] + 1] == type) {
            ++n;
        }
    }
    return n;
}

static int tx_count_tag(const fake_serial_t *f, uint8_t tag)
{
    int i;
    int n = 0;
    for (i = 0; i < f->pkt_count; ++i) {
        const uint8_t *p = f->tx + f->pkt_off[i];
        if (f->pkt_len[i] >= 10 && p[0] == 0x5A && p[1] == KB_COMMAND && p[6] == tag) {
            ++n;
        }
    }
    return n;
}

static void fill_image(uint8_t *image, size_t len)
{
    size_t i;
    for (i = 0; i < len; ++i) {
        image[i] = (uint8_t)((i * 7U + 3U) & 0xFFU);
    }
}

#define IMAGE_LEN   1000U
#define IMAGE_ADDR  0x08004000U

static void test_kboot_happy_path(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    uint32_t params[2];
    int ret;

    fake_serial_reset(&f);
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_OK);
    CHECK(ctx.max_packet_size == 512);
    CHECK(ctx.flash_size == 0x40000);
    CHECK(ctx.flash_sector_size == 0x800);
    CHECK(ctx.bootloader_version == 0x00010203);
    CHECK(ctx.last_status == 0);

    /* ping, 4 GetProperty, erase, WriteMemory, 2 data, reset */
    CHECK(f.pkt_count == 10);
    {
        size_t len;
        const uint8_t *p = tx_pkt(&f, 0, &len);
        CHECK(p && len == 2 && p[0] == 0x5A && p[1] == KB_PING);
    }
    params[0] = 0x0B; CHECK(tx_is_command(&f, 1, TAG_GET_PROP, params, 1));
    params[0] = 0x04; CHECK(tx_is_command(&f, 2, TAG_GET_PROP, params, 1));
    params[0] = 0x05; CHECK(tx_is_command(&f, 3, TAG_GET_PROP, params, 1));
    params[0] = 0x10; CHECK(tx_is_command(&f, 4, TAG_GET_PROP, params, 1));
    params[0] = IMAGE_ADDR; params[1] = IMAGE_LEN;
    CHECK(tx_is_command(&f, 5, TAG_ERASE, params, 2));
    CHECK(tx_is_command(&f, 6, TAG_WRITE, params, 2));
    CHECK(tx_is_data(&f, 7, image, 512));
    CHECK(tx_is_data(&f, 8, image + 512, IMAGE_LEN - 512));
    CHECK(tx_is_command(&f, 9, TAG_RESET, NULL, 0));

    CHECK(f.progress_calls == 2);
    CHECK(f.progress_written[0] == 512 && f.progress_total[0] == IMAGE_LEN);
    CHECK(f.progress_written[1] == IMAGE_LEN && f.progress_total[1] == IMAGE_LEN);
    CHECK(f.delay_calls >= 1);       /* the 300 ms settle delay after ping */
    CHECK(f.flush_calls >= 1);
    CHECK(f.rx_len == 0);            /* nothing left unread */
}

static void test_kboot_small_packet_size(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.max_packet_size = 64;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_OK);
    CHECK(tx_count_type(&f, KB_DATA) == 16);   /* 15 * 64 + 40 */
    CHECK(f.progress_calls == 16);
    CHECK(f.progress_written[0] == 64);
    CHECK(tx_is_data(&f, 7 + 15, image + 960, 40));
}

static void test_kboot_ping_timeout(void)
{
    static fake_serial_t f;
    static uint8_t image[16];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.no_ping_reply = 1;
    kboot_setup(&ctx, &f);
    ctx.ping_retries = 4;

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, sizeof(image));
    CHECK(ret == HIPNUC_KBOOT_ERR_TIMEOUT);
    CHECK(tx_count_type(&f, KB_PING) == 4);
    CHECK(f.pkt_count == 4);
    CHECK(f.delay_calls == 3);
}

static void test_kboot_erase_status(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.erase_status = 0x0D;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_STATUS);
    CHECK(ctx.last_status == 0x0D);
    CHECK(tx_count_tag(&f, TAG_ERASE) == 1);
    CHECK(tx_count_tag(&f, TAG_WRITE) == 0);
    CHECK(tx_count_type(&f, KB_DATA) == 0);
    CHECK(tx_count_tag(&f, TAG_RESET) == 0);
}

static void test_kboot_nak_on_data(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.nak_on_data = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_NAK);
    CHECK(tx_count_type(&f, KB_DATA) == 1);
    CHECK(f.progress_calls == 0);
    CHECK(tx_count_tag(&f, TAG_RESET) == 0);
}

static void test_kboot_abort_on_data(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.abort_on_data = 2;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_ABORT);
    CHECK(tx_count_type(&f, KB_DATA) == 2);
    CHECK(f.progress_calls == 1);
    CHECK(tx_count_tag(&f, TAG_RESET) == 0);
}

static void test_kboot_truncated_response(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.truncate_response = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_TIMEOUT || ret == HIPNUC_KBOOT_ERR_FRAME);
    CHECK(tx_count_tag(&f, TAG_GET_PROP) == 1);   /* stopped at the first property */
    CHECK(tx_count_tag(&f, TAG_ERASE) == 0);
}

static void test_kboot_wrong_echo(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.wrong_echo = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_RESPONSE);
    CHECK(tx_count_tag(&f, TAG_ERASE) == 1);      /* echo is first checked on erase */
    CHECK(tx_count_tag(&f, TAG_WRITE) == 0);
}

static void test_kboot_bad_crc(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.bad_crc = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_FRAME);
    CHECK(tx_count_tag(&f, TAG_GET_PROP) == 1);
    CHECK(tx_count_tag(&f, TAG_ERASE) == 0);
}

static void test_kboot_missing_final_response(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.drop_final_response = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret == HIPNUC_KBOOT_ERR_TIMEOUT);
    CHECK(tx_count_type(&f, KB_DATA) == 2);
    CHECK(tx_count_tag(&f, TAG_RESET) == 0);
}

static void test_kboot_read_failure(void)
{
    static fake_serial_t f;
    static uint8_t image[IMAGE_LEN];
    hipnuc_kboot_ctx_t ctx;
    int ret;

    fake_serial_reset(&f);
    f.read_fails = 1;
    fill_image(image, sizeof(image));
    kboot_setup(&ctx, &f);

    ret = hipnuc_kboot_update(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret < 0);
    CHECK(tx_count_tag(&f, TAG_ERASE) == 0);

    /* Individual commands must also fail cleanly. */
    fake_serial_reset(&f);
    f.read_fails = 1;
    ctx.max_packet_size = 512;
    ret = hipnuc_kboot_erase_region(&ctx, IMAGE_ADDR, IMAGE_LEN);
    CHECK(ret < 0);
    ret = hipnuc_kboot_write_memory(&ctx, IMAGE_ADDR, image, IMAGE_LEN);
    CHECK(ret < 0);
    CHECK(tx_count_type(&f, KB_DATA) == 0);
}

static void test_kboot_bad_ack_type(void)
{
    static fake_serial_t f;
    hipnuc_kboot_ctx_t ctx;
    uint8_t junk[2] = { 0x5A, 0xA7 };
    int ret;

    /* A stray byte pair instead of ACK -> ERR_FRAME. */
    fake_serial_reset(&f);
    kboot_setup(&ctx, &f);
    ctx.port.flush = NULL;   /* keep the pre-queued junk */
    fake_enqueue(&f, junk, 2);
    ret = hipnuc_kboot_reset(&ctx);
    CHECK(ret == HIPNUC_KBOOT_ERR_FRAME);
}

static void test_kboot_params(void)
{
    static fake_serial_t f;
    hipnuc_kboot_ctx_t ctx;
    hipnuc_kboot_port_t port;
    uint8_t image[4] = { 1, 2, 3, 4 };

    memset(&port, 0, sizeof(port));
    hipnuc_kboot_init(&ctx, &port);
    CHECK(ctx.command_timeout_ms == 1000);
    CHECK(ctx.erase_timeout_ms == 10000);
    CHECK(ctx.data_timeout_ms == 2000);
    CHECK(ctx.ping_timeout_ms == 50);
    CHECK(ctx.ping_retries == 10);
    CHECK(hipnuc_kboot_ping(&ctx) == HIPNUC_KBOOT_ERR_PARAM);
    CHECK(hipnuc_kboot_update(&ctx, 0, image, sizeof(image)) == HIPNUC_KBOOT_ERR_PARAM);
    CHECK(hipnuc_kboot_update(NULL, 0, image, sizeof(image)) == HIPNUC_KBOOT_ERR_PARAM);

    fake_serial_reset(&f);
    kboot_setup(&ctx, &f);
    CHECK(hipnuc_kboot_update(&ctx, 0, NULL, 4) == HIPNUC_KBOOT_ERR_PARAM);
    CHECK(hipnuc_kboot_update(&ctx, 0, image, 0) == HIPNUC_KBOOT_ERR_PARAM);
    CHECK(hipnuc_kboot_write_memory(&ctx, 0, image, 4) == HIPNUC_KBOOT_ERR_SIZE);   /* connect not called */

    ctx.max_packet_size = 512;
    ctx.flash_size = 2;
    CHECK(hipnuc_kboot_write_memory(&ctx, 0, image, 4) == HIPNUC_KBOOT_ERR_SIZE);
    CHECK(f.pkt_count == 0);

    /* max packet size 0 from the device is rejected by connect */
    fake_serial_reset(&f);
    f.max_packet_size = 0;
    kboot_setup(&ctx, &f);
    CHECK(hipnuc_kboot_connect(&ctx) == HIPNUC_KBOOT_ERR_SIZE);

    CHECK(strcmp(hipnuc_kboot_strerror(HIPNUC_KBOOT_OK), "ok") == 0);
    CHECK(hipnuc_kboot_strerror(HIPNUC_KBOOT_ERR_NAK)[0] != '\0');
    CHECK(hipnuc_kboot_strerror(1234)[0] != '\0');
}

/* ------------------------------------------------------------------------ */
/* Fake CAN bus scripted as the CAN bootloader                              */
/* ------------------------------------------------------------------------ */

#define FAKE_CAN_TX_MAX  64
#define FAKE_CAN_Q_MAX   8

typedef struct {
    hipnuc_can_frame_t tx[FAKE_CAN_TX_MAX];
    int tx_count;

    hipnuc_can_frame_t queue[FAKE_CAN_Q_MAX];
    int queue_len;

    /* scenario switches */
    int silent;                /* never reply */
    uint8_t enter_bl_cs;       /* reply cs to 0x1F51:05 */
    int abort_on_segment;      /* 1-based, 0 = never */
    int interleave_j1939;      /* queue a J1939 frame ahead of every reply */
    int seg_count;
    int j1939_seen_by_wait;    /* frames with a foreign id skipped by wait() */

    int progress_calls;
    uint8_t last_percent;
    uint8_t last_node;
    int delay_calls;
    int log_calls;
} fake_can_t;

static void fake_can_reset(fake_can_t *f)
{
    memset(f, 0, sizeof(*f));
    f->enter_bl_cs = 0x60;
}

static void fake_can_queue(fake_can_t *f, const hipnuc_can_frame_t *frame)
{
    if (f->queue_len >= FAKE_CAN_Q_MAX) {
        printf("fake CAN queue overflow\n");
        ++g_failures;
        return;
    }
    f->queue[f->queue_len++] = *frame;
}

static int fake_can_send(void *user, const hipnuc_can_frame_t *frame)
{
    fake_can_t *f = (fake_can_t *)user;
    hipnuc_can_frame_t reply;
    uint8_t cs = frame->data[0];

    if (f->tx_count >= FAKE_CAN_TX_MAX) {
        return -1;
    }
    f->tx[f->tx_count++] = *frame;
    if (f->silent) {
        return 0;
    }
    if (frame->is_extended || frame->id < 0x600U || frame->id > 0x67FU || frame->len != 8U) {
        return 0;   /* not an SDO request; the bootloader ignores it */
    }

    if (f->interleave_j1939) {
        hipnuc_can_frame_t j1939;
        memset(&j1939, 0, sizeof(j1939));
        j1939.id = 0x18FF5108U;   /* looks like a measurement PGN */
        j1939.is_extended = 1;
        j1939.len = 8;
        fake_can_queue(f, &j1939);
    }

    memset(&reply, 0, sizeof(reply));
    reply.id = 0x580U + (frame->id - 0x600U);
    reply.len = 8;
    memcpy(reply.data, frame->data, 8);   /* the bootloader echoes bytes 1..7 */

    if (cs == 0x23) {
        reply.data[0] = (frame->data[3] == 0x05) ? f->enter_bl_cs : 0x60;
        memset(reply.data + 4, 0, 4);
    } else if (cs == 0x21) {
        reply.data[0] = 0x60;
        memset(reply.data + 4, 0, 4);
    } else if ((cs & 0xE0U) == 0x00U) {
        ++f->seg_count;
        if (f->abort_on_segment == f->seg_count) {
            reply.data[0] = 0x80;
        } else {
            reply.data[0] = (cs & 0x10U) ? 0x30 : 0x20;
        }
    } else {
        reply.data[0] = 0x80;
    }
    fake_can_queue(f, &reply);
    return 0;
}

static int fake_can_wait(void *user, uint32_t id, hipnuc_can_frame_t *frame, uint32_t timeout_ms)
{
    fake_can_t *f = (fake_can_t *)user;

    (void)timeout_ms;
    while (f->queue_len > 0) {
        hipnuc_can_frame_t head = f->queue[0];
        memmove(&f->queue[0], &f->queue[1], (size_t)(f->queue_len - 1) * sizeof(f->queue[0]));
        --f->queue_len;
        if (!head.is_extended && head.id == id) {
            *frame = head;
            return 0;
        }
        ++f->j1939_seen_by_wait;   /* skipped like a real port would */
    }
    return 1;
}

static void fake_can_delay(void *user, uint32_t ms)
{
    fake_can_t *f = (fake_can_t *)user;
    (void)ms;
    ++f->delay_calls;
}

static void fake_can_progress(void *user, uint8_t node_id, uint8_t percent)
{
    fake_can_t *f = (fake_can_t *)user;
    ++f->progress_calls;
    f->last_percent = percent;
    f->last_node = node_id;
}

static void fake_can_log(void *user, uint8_t node_id, const char *message)
{
    fake_can_t *f = (fake_can_t *)user;
    (void)node_id;
    (void)message;
    ++f->log_calls;
}

static void can_setup(hipnuc_can_update_ctx_t *ctx, fake_can_t *f)
{
    hipnuc_can_update_port_t port;
    memset(&port, 0, sizeof(port));
    port.send = fake_can_send;
    port.wait = fake_can_wait;
    port.delay_ms = fake_can_delay;
    port.progress = fake_can_progress;
    port.log = fake_can_log;
    port.user = f;
    hipnuc_can_update_init(ctx, &port);
}

static int can_tx_is_expedited(const fake_can_t *f, int i, uint8_t node, uint8_t sub, uint32_t value)
{
    const hipnuc_can_frame_t *t;
    if (i < 0 || i >= f->tx_count) return 0;
    t = &f->tx[i];
    return t->id == 0x600U + node && !t->is_extended && t->len == 8 &&
           t->data[0] == 0x23 && t->data[1] == 0x51 && t->data[2] == 0x1F && t->data[3] == sub &&
           rd_u32(t->data + 4) == value;
}

static int can_tx_is_segment(const fake_can_t *f, int i, uint8_t node, uint8_t cs, const uint8_t *data, size_t n)
{
    const hipnuc_can_frame_t *t;
    size_t k;
    if (i < 0 || i >= f->tx_count) return 0;
    t = &f->tx[i];
    if (t->id != 0x600U + node || t->is_extended || t->len != 8 || t->data[0] != cs) return 0;
    if (memcmp(t->data + 1, data, n) != 0) return 0;
    for (k = n; k < 7; ++k) {
        if (t->data[1 + k] != 0) return 0;   /* padding */
    }
    return 1;
}

static void test_can_happy_path(void)
{
    static fake_can_t f;
    uint8_t image[20];
    hipnuc_can_update_ctx_t ctx;
    int ret;
    size_t i;

    for (i = 0; i < sizeof(image); ++i) {
        image[i] = (uint8_t)(0xA0 + i);
    }
    fake_can_reset(&f);
    can_setup(&ctx, &f);

    ret = hipnuc_can_update_node(&ctx, 8, image, sizeof(image));
    CHECK(ret == HIPNUC_CAN_UPDATE_OK);

    /* enter BL, confirm, initiate, 3 segments, goto app */
    CHECK(f.tx_count == 7);
    CHECK(can_tx_is_expedited(&f, 0, 8, 0x05, 0));
    CHECK(can_tx_is_expedited(&f, 1, 8, 0x06, 0));
    CHECK(f.tx[2].id == 0x608 && f.tx[2].len == 8 && f.tx[2].data[0] == 0x21);
    CHECK(f.tx[2].data[1] == 0x51 && f.tx[2].data[2] == 0x1F && f.tx[2].data[3] == 0x01);
    CHECK(rd_u32(f.tx[2].data + 4) == 20);
    CHECK(can_tx_is_segment(&f, 3, 8, 0x00, image, 7));
    CHECK(can_tx_is_segment(&f, 4, 8, 0x10, image + 7, 7));
    CHECK(can_tx_is_segment(&f, 5, 8, 0x05, image + 14, 6));   /* last: 6 bytes -> 0x00 | 0x05 */
    CHECK(can_tx_is_expedited(&f, 6, 8, 0x09, 0));

    CHECK(f.progress_calls == 3);
    CHECK(f.last_percent == 100);
    CHECK(f.last_node == 8);
    CHECK(f.delay_calls == 1);   /* boot delay only */
    CHECK(f.queue_len == 0);
}

static void test_can_last_segment_masks(void)
{
    /* Bootloader convention: n = ((15 - (cs & 0x0F)) / 2) + 1 */
    static const uint8_t expect[8] = { 0x00, 0x0F, 0x0D, 0x0B, 0x09, 0x07, 0x05, 0x03 };
    static fake_can_t f;
    uint8_t image[7 + 7];
    hipnuc_can_update_ctx_t ctx;
    uint32_t n;

    memset(image, 0x5A, sizeof(image));
    for (n = 1; n <= 7; ++n) {
        fake_can_reset(&f);
        can_setup(&ctx, &f);
        CHECK(hipnuc_can_update_download(&ctx, 3, image, 7 + n) == HIPNUC_CAN_UPDATE_OK);
        CHECK(f.tx_count == 4);   /* initiate, 2 segments, goto app */
        CHECK(f.tx[1].data[0] == 0x00);
        CHECK(f.tx[2].data[0] == (uint8_t)(0x10 | expect[n]));
        CHECK(((15 - (f.tx[2].data[0] & 0x0F)) / 2) + 1 == (int)n);
    }

    /* Exactly one 7-byte segment: toggle 0 with the 7-byte mask. */
    fake_can_reset(&f);
    can_setup(&ctx, &f);
    CHECK(hipnuc_can_update_download(&ctx, 3, image, 7) == HIPNUC_CAN_UPDATE_OK);
    CHECK(f.tx_count == 3);
    CHECK(f.tx[1].data[0] == 0x03);
}

static void test_can_enter_bootloader_reply(void)
{
    static fake_can_t f;
    hipnuc_can_update_ctx_t ctx;
    int ret;

    /* 0x60 from the application is the correct acknowledgement. */
    fake_can_reset(&f);
    f.enter_bl_cs = 0x60;
    can_setup(&ctx, &f);
    ret = hipnuc_can_update_connect(&ctx, 1);
    CHECK(ret == HIPNUC_CAN_UPDATE_OK);
    CHECK(f.tx_count == 2);

    /* Anything else is not; the handshake is retried and finally times out. */
    fake_can_reset(&f);
    f.enter_bl_cs = 0x00;
    can_setup(&ctx, &f);
    ret = hipnuc_can_update_connect(&ctx, 1);
    CHECK(ret == HIPNUC_CAN_UPDATE_ERR_TIMEOUT);
    CHECK(f.tx_count == 5);

    /* An SDO abort on enter-bootloader is also retried. */
    fake_can_reset(&f);
    f.enter_bl_cs = 0x80;
    can_setup(&ctx, &f);
    ctx.handshake_retries = 2;
    ret = hipnuc_can_update_connect(&ctx, 1);
    CHECK(ret == HIPNUC_CAN_UPDATE_ERR_TIMEOUT);
    CHECK(f.tx_count == 2);
}

static void test_can_abort_on_segment(void)
{
    static fake_can_t f;
    uint8_t image[20];
    hipnuc_can_update_ctx_t ctx;
    int ret;

    memset(image, 0x11, sizeof(image));
    fake_can_reset(&f);
    f.abort_on_segment = 2;
    can_setup(&ctx, &f);

    ret = hipnuc_can_update_node(&ctx, 8, image, sizeof(image));
    CHECK(ret == HIPNUC_CAN_UPDATE_ERR_ABORT);
    CHECK(f.tx_count == 5);   /* enter, confirm, initiate, seg1, seg2 -> no goto app */
    CHECK(f.progress_calls == 1);
}

static void test_can_no_reply(void)
{
    static fake_can_t f;
    uint8_t image[20];
    hipnuc_can_update_ctx_t ctx;
    int ret;

    memset(image, 0x22, sizeof(image));
    fake_can_reset(&f);
    f.silent = 1;
    can_setup(&ctx, &f);

    ret = hipnuc_can_update_node(&ctx, 8, image, sizeof(image));
    CHECK(ret == HIPNUC_CAN_UPDATE_ERR_TIMEOUT);
    CHECK(f.tx_count == 5);   /* five enter-bootloader attempts, nothing else */
    CHECK(can_tx_is_expedited(&f, 4, 8, 0x05, 0));
    CHECK(f.delay_calls == 4);   /* retry delays between attempts */

    /* A single expedited write also times out. */
    ret = hipnuc_can_update_sdo_write(&ctx, 8, 0x1F51, 0x06, 0, 10);
    CHECK(ret == HIPNUC_CAN_UPDATE_ERR_TIMEOUT);
}

static void test_can_params(void)
{
    static fake_can_t f;
    uint8_t image[20];
    hipnuc_can_update_ctx_t ctx;
    hipnuc_can_update_port_t port;

    memset(image, 0x33, sizeof(image));
    fake_can_reset(&f);
    can_setup(&ctx, &f);

    CHECK(ctx.expedited_timeout_ms == 100);
    CHECK(ctx.initiate_timeout_ms == 8000);
    CHECK(ctx.segment_timeout_ms == 4000);
    CHECK(ctx.boot_delay_ms == 20);
    CHECK(ctx.retry_delay_ms == 50);
    CHECK(ctx.handshake_retries == 5);

    CHECK(hipnuc_can_update_node(&ctx, 0, image, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_node(&ctx, 128, image, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_node(&ctx, 8, NULL, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_node(&ctx, 8, image, 0) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_connect(&ctx, 0) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_download(&ctx, 200, image, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_sdo_write(&ctx, 0, 0x1F51, 5, 0, 100) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(hipnuc_can_update_node(NULL, 8, image, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);
    CHECK(f.tx_count == 0);

    memset(&port, 0, sizeof(port));
    hipnuc_can_update_init(&ctx, &port);
    CHECK(hipnuc_can_update_node(&ctx, 8, image, sizeof(image)) == HIPNUC_CAN_UPDATE_ERR_PARAM);

    /* Node 127 is the highest valid id. */
    fake_can_reset(&f);
    can_setup(&ctx, &f);
    CHECK(hipnuc_can_update_node(&ctx, 127, image, sizeof(image)) == HIPNUC_CAN_UPDATE_OK);
    CHECK(f.tx[0].id == 0x67F);

    CHECK(strcmp(hipnuc_can_update_strerror(HIPNUC_CAN_UPDATE_OK), "ok") == 0);
    CHECK(hipnuc_can_update_strerror(HIPNUC_CAN_UPDATE_ERR_ABORT)[0] != '\0');
    CHECK(hipnuc_can_update_strerror(99)[0] != '\0');
}

static void test_can_interleaved_j1939(void)
{
    static fake_can_t f;
    uint8_t image[20];
    hipnuc_can_update_ctx_t ctx;
    int ret;

    memset(image, 0x44, sizeof(image));
    fake_can_reset(&f);
    f.interleave_j1939 = 1;
    can_setup(&ctx, &f);

    ret = hipnuc_can_update_node(&ctx, 8, image, sizeof(image));
    CHECK(ret == HIPNUC_CAN_UPDATE_OK);
    CHECK(f.tx_count == 7);
    CHECK(f.j1939_seen_by_wait == 7);   /* one measurement frame skipped per exchange */
    CHECK(f.queue_len == 0);
}

int main(void)
{
    test_kboot_happy_path();
    test_kboot_small_packet_size();
    test_kboot_ping_timeout();
    test_kboot_erase_status();
    test_kboot_nak_on_data();
    test_kboot_abort_on_data();
    test_kboot_truncated_response();
    test_kboot_wrong_echo();
    test_kboot_bad_crc();
    test_kboot_missing_final_response();
    test_kboot_read_failure();
    test_kboot_bad_ack_type();
    test_kboot_params();

    test_can_happy_path();
    test_can_last_segment_masks();
    test_can_enter_bootloader_reply();
    test_can_abort_on_segment();
    test_can_no_reply();
    test_can_params();
    test_can_interleaved_j1939();

    printf("test_update: %d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
