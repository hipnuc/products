/*
 * Copyright (c) 2006-2026, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial firmware update client for the HiPNUC bootloader. See hipnuc_kboot.h.
 *
 * Framing packet: 5A <type> <len lo> <len hi> <crc lo> <crc hi> <payload>
 * The CRC (CRC-16/XMODEM) covers header bytes 0..3 followed by the payload.
 * Command payload: <tag> <flags> <reserved> <param count> <u32 params...>
 */

#include "hipnuc_kboot.h"
#include "hipnuc_dec.h"

#include <string.h>

/* Framing packet types */
#define KBOOT_START_BYTE        0x5AU
#define KBOOT_TYPE_ACK          0xA1U
#define KBOOT_TYPE_NAK          0xA2U
#define KBOOT_TYPE_ACK_ABORT    0xA3U
#define KBOOT_TYPE_COMMAND      0xA4U
#define KBOOT_TYPE_DATA         0xA5U
#define KBOOT_TYPE_PING         0xA6U
#define KBOOT_TYPE_PING_RESP    0xA7U

/* Command tags */
#define KBOOT_TAG_FLASH_ERASE_REGION   0x02U
#define KBOOT_TAG_WRITE_MEMORY         0x04U
#define KBOOT_TAG_GET_PROPERTY         0x07U
#define KBOOT_TAG_RESET                0x0BU
#define KBOOT_TAG_GENERIC_RESPONSE     0xA0U
#define KBOOT_TAG_GET_PROPERTY_RESP    0xA7U

/* Property codes */
#define KBOOT_PROP_FLASH_SIZE          0x04U
#define KBOOT_PROP_FLASH_SECTOR_SIZE   0x05U
#define KBOOT_PROP_MAX_PACKET_SIZE     0x0BU
#define KBOOT_PROP_BOOTLOADER_VERSION  0x10U

#define KBOOT_NO_ECHO           0xFFU   /* response carries no command echo */

#define KBOOT_HEADER_SIZE       6U
#define KBOOT_MAX_PAYLOAD       512U    /* largest data packet we send or accept */
#define KBOOT_MAX_PARAMS        2U
#define KBOOT_PING_RESP_SIZE    10U

#define KBOOT_DEFAULT_COMMAND_TIMEOUT_MS  1000U
#define KBOOT_DEFAULT_ERASE_TIMEOUT_MS    10000U
#define KBOOT_DEFAULT_DATA_TIMEOUT_MS     2000U
#define KBOOT_DEFAULT_PING_TIMEOUT_MS     50U
#define KBOOT_DEFAULT_PING_RETRIES        10U
#define KBOOT_PING_RETRY_DELAY_MS         100U
#define KBOOT_POST_PING_DELAY_MS          300U

/* Little-endian helpers; safe for any alignment. */
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void kboot_log(const hipnuc_kboot_ctx_t *ctx, const char *message)
{
    if (ctx->port.log) {
        ctx->port.log(ctx->port.user, message);
    }
}

static void kboot_delay(const hipnuc_kboot_ctx_t *ctx, uint32_t ms)
{
    if (ctx->port.delay_ms) {
        ctx->port.delay_ms(ctx->port.user, ms);
    }
}

static void kboot_flush(const hipnuc_kboot_ctx_t *ctx)
{
    if (ctx->port.flush) {
        ctx->port.flush(ctx->port.user);
    }
}

static int kboot_check_ctx(const hipnuc_kboot_ctx_t *ctx)
{
    if (!ctx || !ctx->port.write || !ctx->port.read) {
        return HIPNUC_KBOOT_ERR_PARAM;
    }
    return HIPNUC_KBOOT_OK;
}

static int kboot_write(const hipnuc_kboot_ctx_t *ctx, const uint8_t *data, size_t len)
{
    int n = ctx->port.write(ctx->port.user, data, len);
    if (n < 0 || (size_t)n != len) {
        kboot_log(ctx, "serial write failed");
        return HIPNUC_KBOOT_ERR_WRITE;
    }
    return HIPNUC_KBOOT_OK;
}

/* Read exactly `len` bytes, looping while bytes keep arriving. A read that
 * returns 0 (timeout) or a negative value ends the attempt. */
static int kboot_read_exact(const hipnuc_kboot_ctx_t *ctx, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    size_t got = 0;

    while (got < len) {
        int n = ctx->port.read(ctx->port.user, buf + got, len - got, timeout_ms);
        if (n < 0) {
            kboot_log(ctx, "serial read failed");
            return HIPNUC_KBOOT_ERR_READ;
        }
        if (n == 0) {
            return HIPNUC_KBOOT_ERR_TIMEOUT;
        }
        if ((size_t)n > len - got) {
            /* Misbehaving port: more bytes than requested. */
            return HIPNUC_KBOOT_ERR_FRAME;
        }
        got += (size_t)n;
    }
    return HIPNUC_KBOOT_OK;
}

/* Fill `out` (at least KBOOT_HEADER_SIZE + len bytes) with a framing packet. */
static size_t kboot_build_frame(uint8_t type, const uint8_t *payload, size_t len, uint8_t *out)
{
    uint16_t crc;

    out[0] = KBOOT_START_BYTE;
    out[1] = type;
    wr_u16(out + 2, (uint16_t)len);
    if (len > 0) {
        memcpy(out + KBOOT_HEADER_SIZE, payload, len);
    }
    crc = hipnuc_crc16(0, out, 4);
    crc = hipnuc_crc16(crc, out + KBOOT_HEADER_SIZE, len);
    wr_u16(out + 4, crc);
    return KBOOT_HEADER_SIZE + len;
}

/* Read the two-byte acknowledgement that precedes every response. */
static int kboot_read_ack(const hipnuc_kboot_ctx_t *ctx, uint32_t timeout_ms)
{
    uint8_t ack[2];
    int ret = kboot_read_exact(ctx, ack, sizeof(ack), timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (ack[0] != KBOOT_START_BYTE) {
        kboot_log(ctx, "bad start byte");
        return HIPNUC_KBOOT_ERR_FRAME;
    }
    switch (ack[1]) {
    case KBOOT_TYPE_ACK:
        return HIPNUC_KBOOT_OK;
    case KBOOT_TYPE_NAK:
        kboot_log(ctx, "device answered NAK");
        return HIPNUC_KBOOT_ERR_NAK;
    case KBOOT_TYPE_ACK_ABORT:
        kboot_log(ctx, "device answered AckAbort");
        return HIPNUC_KBOOT_ERR_ABORT;
    default:
        kboot_log(ctx, "unexpected packet type");
        return HIPNUC_KBOOT_ERR_FRAME;
    }
}

/* Read a Command framing packet incrementally and CRC-check it. `payload`
 * must hold KBOOT_MAX_PAYLOAD bytes; *payload_len receives its length. */
static int kboot_read_command_frame(const hipnuc_kboot_ctx_t *ctx, uint32_t timeout_ms,
                                    uint8_t *payload, size_t *payload_len)
{
    uint8_t header[KBOOT_HEADER_SIZE];
    uint16_t len;
    uint16_t crc;
    int ret;

    ret = kboot_read_exact(ctx, header, 2, timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (header[0] != KBOOT_START_BYTE || header[1] != KBOOT_TYPE_COMMAND) {
        kboot_log(ctx, "response framing packet is invalid");
        return HIPNUC_KBOOT_ERR_FRAME;
    }

    ret = kboot_read_exact(ctx, header + 2, 4, timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    len = (uint16_t)((uint16_t)header[2] | ((uint16_t)header[3] << 8));
    if (len > KBOOT_MAX_PAYLOAD) {
        kboot_log(ctx, "response payload too long");
        return HIPNUC_KBOOT_ERR_FRAME;
    }

    ret = kboot_read_exact(ctx, payload, len, timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }

    crc = hipnuc_crc16(0, header, 4);
    crc = hipnuc_crc16(crc, payload, len);
    if (crc != (uint16_t)((uint16_t)header[4] | ((uint16_t)header[5] << 8))) {
        kboot_log(ctx, "response CRC mismatch");
        return HIPNUC_KBOOT_ERR_FRAME;
    }

    *payload_len = len;
    return HIPNUC_KBOOT_OK;
}

/* Validate a command response payload. On success params[0..1] hold the
 * status and the second parameter (property value or echoed command tag). */
static int kboot_parse_response(hipnuc_kboot_ctx_t *ctx, const uint8_t *payload, size_t len,
                                uint8_t expected_tag, uint8_t expected_echo, uint32_t *params)
{
    uint8_t count;

    if (len < 4) {
        kboot_log(ctx, "response payload is truncated");
        return HIPNUC_KBOOT_ERR_RESPONSE;
    }
    if (payload[0] != expected_tag) {
        kboot_log(ctx, "response tag mismatch");
        return HIPNUC_KBOOT_ERR_RESPONSE;
    }
    count = payload[3];
    if (count < KBOOT_MAX_PARAMS || len < 4U + (size_t)count * 4U) {
        kboot_log(ctx, "response parameter count is invalid");
        return HIPNUC_KBOOT_ERR_RESPONSE;
    }
    params[0] = rd_u32(payload + 4);
    params[1] = rd_u32(payload + 8);

    ctx->last_status = params[0];
    if (params[0] != 0) {
        kboot_log(ctx, "device reported an error status");
        return HIPNUC_KBOOT_ERR_STATUS;
    }
    if (expected_echo != KBOOT_NO_ECHO && params[1] != (uint32_t)expected_echo) {
        kboot_log(ctx, "command echo mismatch");
        return HIPNUC_KBOOT_ERR_RESPONSE;
    }
    return HIPNUC_KBOOT_OK;
}

/* ACK + Command framing packet + response validation. */
static int kboot_read_response(hipnuc_kboot_ctx_t *ctx, uint32_t timeout_ms,
                               uint8_t expected_tag, uint8_t expected_echo, uint32_t *params)
{
    uint8_t payload[KBOOT_MAX_PAYLOAD];
    size_t payload_len = 0;
    int ret;

    ret = kboot_read_ack(ctx, timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = kboot_read_command_frame(ctx, timeout_ms, payload, &payload_len);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    return kboot_parse_response(ctx, payload, payload_len, expected_tag, expected_echo, params);
}

/* Send a command with up to KBOOT_MAX_PARAMS parameters and validate the reply. */
static int kboot_command(hipnuc_kboot_ctx_t *ctx, uint8_t tag, const uint32_t *args, uint8_t arg_count,
                         uint32_t timeout_ms, uint8_t expected_tag, uint8_t expected_echo, uint32_t *params)
{
    uint8_t payload[4 + KBOOT_MAX_PARAMS * 4];
    uint8_t frame[KBOOT_HEADER_SIZE + sizeof(payload)];
    size_t frame_len;
    uint8_t i;
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (arg_count > KBOOT_MAX_PARAMS) {
        return HIPNUC_KBOOT_ERR_PARAM;
    }

    payload[0] = tag;
    payload[1] = 0;   /* flags */
    payload[2] = 0;   /* reserved */
    payload[3] = arg_count;
    for (i = 0; i < arg_count; ++i) {
        wr_u32(payload + 4 + (size_t)i * 4, args[i]);
    }
    frame_len = kboot_build_frame(KBOOT_TYPE_COMMAND, payload, 4U + (size_t)arg_count * 4U, frame);

    kboot_flush(ctx);
    ret = kboot_write(ctx, frame, frame_len);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    return kboot_read_response(ctx, timeout_ms, expected_tag, expected_echo, params);
}

void hipnuc_kboot_init(hipnuc_kboot_ctx_t *ctx, const hipnuc_kboot_port_t *port)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (port) {
        ctx->port = *port;
    }
    ctx->command_timeout_ms = KBOOT_DEFAULT_COMMAND_TIMEOUT_MS;
    ctx->erase_timeout_ms = KBOOT_DEFAULT_ERASE_TIMEOUT_MS;
    ctx->data_timeout_ms = KBOOT_DEFAULT_DATA_TIMEOUT_MS;
    ctx->ping_timeout_ms = KBOOT_DEFAULT_PING_TIMEOUT_MS;
    ctx->ping_retries = KBOOT_DEFAULT_PING_RETRIES;
}

int hipnuc_kboot_ping(hipnuc_kboot_ctx_t *ctx)
{
    static const uint8_t ping[2] = { KBOOT_START_BYTE, KBOOT_TYPE_PING };
    uint8_t resp[KBOOT_PING_RESP_SIZE];
    uint16_t crc;
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }

    kboot_flush(ctx);
    ret = kboot_write(ctx, ping, sizeof(ping));
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = kboot_read_exact(ctx, resp, sizeof(resp), ctx->ping_timeout_ms);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (resp[0] != KBOOT_START_BYTE || resp[1] != KBOOT_TYPE_PING_RESP) {
        kboot_log(ctx, "ping: invalid response packet");
        return HIPNUC_KBOOT_ERR_FRAME;
    }
    crc = hipnuc_crc16(0, resp, 8);
    if (crc != (uint16_t)((uint16_t)resp[8] | ((uint16_t)resp[9] << 8))) {
        kboot_log(ctx, "ping: response CRC mismatch");
        return HIPNUC_KBOOT_ERR_FRAME;
    }
    return HIPNUC_KBOOT_OK;
}

int hipnuc_kboot_get_property(hipnuc_kboot_ctx_t *ctx, uint32_t property, uint32_t *value)
{
    uint32_t args[1];
    uint32_t params[KBOOT_MAX_PARAMS];
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    args[0] = property;
    ret = kboot_command(ctx, KBOOT_TAG_GET_PROPERTY, args, 1, ctx->command_timeout_ms,
                        KBOOT_TAG_GET_PROPERTY_RESP, KBOOT_NO_ECHO, params);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (value) {
        *value = params[1];
    }
    return HIPNUC_KBOOT_OK;
}

int hipnuc_kboot_connect(hipnuc_kboot_ctx_t *ctx)
{
    uint8_t attempts;
    uint8_t i;
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }

    attempts = ctx->ping_retries ? ctx->ping_retries : 1U;
    ret = HIPNUC_KBOOT_ERR_TIMEOUT;
    for (i = 0; i < attempts; ++i) {
        ret = hipnuc_kboot_ping(ctx);
        if (ret == HIPNUC_KBOOT_OK) {
            break;
        }
        if (ret == HIPNUC_KBOOT_ERR_READ || ret == HIPNUC_KBOOT_ERR_WRITE ||
            ret == HIPNUC_KBOOT_ERR_PARAM) return ret;
        if (i + 1U < attempts) {
            kboot_delay(ctx, KBOOT_PING_RETRY_DELAY_MS);
        }
    }
    if (ret != HIPNUC_KBOOT_OK) {
        kboot_log(ctx, "bootloader did not answer ping");
        return ret;
    }
    kboot_log(ctx, "bootloader ping ok");

    kboot_delay(ctx, KBOOT_POST_PING_DELAY_MS);

    ret = hipnuc_kboot_get_property(ctx, KBOOT_PROP_MAX_PACKET_SIZE, &ctx->max_packet_size);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_get_property(ctx, KBOOT_PROP_FLASH_SIZE, &ctx->flash_size);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_get_property(ctx, KBOOT_PROP_FLASH_SECTOR_SIZE, &ctx->flash_sector_size);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_get_property(ctx, KBOOT_PROP_BOOTLOADER_VERSION, &ctx->bootloader_version);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (ctx->max_packet_size == 0) {
        kboot_log(ctx, "device reports max packet size 0");
        return HIPNUC_KBOOT_ERR_SIZE;
    }
    kboot_log(ctx, "bootloader properties read");
    return HIPNUC_KBOOT_OK;
}

int hipnuc_kboot_erase_region(hipnuc_kboot_ctx_t *ctx, uint32_t address, uint32_t length)
{
    uint32_t args[2];
    uint32_t params[KBOOT_MAX_PARAMS];
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    args[0] = address;
    args[1] = length;
    kboot_log(ctx, "erasing flash");
    return kboot_command(ctx, KBOOT_TAG_FLASH_ERASE_REGION, args, 2, ctx->erase_timeout_ms,
                         KBOOT_TAG_GENERIC_RESPONSE, KBOOT_TAG_FLASH_ERASE_REGION, params);
}

int hipnuc_kboot_write_memory(hipnuc_kboot_ctx_t *ctx, uint32_t address, const uint8_t *image, uint32_t length)
{
    uint8_t frame[KBOOT_HEADER_SIZE + KBOOT_MAX_PAYLOAD];
    uint32_t args[2];
    uint32_t params[KBOOT_MAX_PARAMS];
    uint32_t chunk_max;
    uint32_t offset;
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (!image || length == 0) {
        return HIPNUC_KBOOT_ERR_PARAM;
    }
    if (ctx->flash_size != 0 && length > ctx->flash_size) {
        kboot_log(ctx, "image is larger than the device flash");
        return HIPNUC_KBOOT_ERR_SIZE;
    }
    if (ctx->max_packet_size == 0) {
        kboot_log(ctx, "max packet size is not known; call connect first");
        return HIPNUC_KBOOT_ERR_SIZE;
    }
    chunk_max = ctx->max_packet_size;
    if (chunk_max > KBOOT_MAX_PAYLOAD) {
        chunk_max = KBOOT_MAX_PAYLOAD;
    }

    args[0] = address;
    args[1] = length;
    ret = kboot_command(ctx, KBOOT_TAG_WRITE_MEMORY, args, 2, ctx->command_timeout_ms,
                        KBOOT_TAG_GENERIC_RESPONSE, KBOOT_TAG_WRITE_MEMORY, params);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    kboot_log(ctx, "writing image");

    for (offset = 0; offset < length; ) {
        uint32_t chunk = length - offset;
        size_t frame_len;
        int last;

        if (chunk > chunk_max) {
            chunk = chunk_max;
        }
        last = (offset + chunk >= length);

        frame_len = kboot_build_frame(KBOOT_TYPE_DATA, image + offset, chunk, frame);
        ret = kboot_write(ctx, frame, frame_len);
        if (ret != HIPNUC_KBOOT_OK) {
            return ret;
        }
        ret = kboot_read_ack(ctx, ctx->data_timeout_ms);
        if (ret != HIPNUC_KBOOT_OK) {
            return ret;
        }
        if (last) {
            /* The final data packet is followed by the WriteMemory result.
             * `frame` has been sent, so it is reused as the receive buffer. */
            size_t payload_len = 0;

            ret = kboot_read_command_frame(ctx, ctx->data_timeout_ms, frame, &payload_len);
            if (ret != HIPNUC_KBOOT_OK) {
                return ret;
            }
            ret = kboot_parse_response(ctx, frame, payload_len, KBOOT_TAG_GENERIC_RESPONSE,
                                       KBOOT_TAG_WRITE_MEMORY, params);
            if (ret != HIPNUC_KBOOT_OK) {
                return ret;
            }
        }

        offset += chunk;
        if (ctx->port.progress) {
            ctx->port.progress(ctx->port.user, offset, length);
        }
    }

    kboot_log(ctx, "image written");
    return HIPNUC_KBOOT_OK;
}

int hipnuc_kboot_reset(hipnuc_kboot_ctx_t *ctx)
{
    uint32_t params[KBOOT_MAX_PARAMS];
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    kboot_log(ctx, "resetting device");
    return kboot_command(ctx, KBOOT_TAG_RESET, NULL, 0, ctx->command_timeout_ms,
                         KBOOT_TAG_GENERIC_RESPONSE, KBOOT_TAG_RESET, params);
}

int hipnuc_kboot_update(hipnuc_kboot_ctx_t *ctx, uint32_t address, const uint8_t *image, uint32_t length)
{
    int ret;

    ret = kboot_check_ctx(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    if (!image || length == 0) {
        return HIPNUC_KBOOT_ERR_PARAM;
    }

    ret = hipnuc_kboot_connect(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_erase_region(ctx, address, length);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_write_memory(ctx, address, image, length);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    ret = hipnuc_kboot_reset(ctx);
    if (ret != HIPNUC_KBOOT_OK) {
        return ret;
    }
    kboot_log(ctx, "update complete");
    return HIPNUC_KBOOT_OK;
}

const char *hipnuc_kboot_strerror(int status)
{
    switch (status) {
    case HIPNUC_KBOOT_OK:           return "ok";
    case HIPNUC_KBOOT_ERR_PARAM:    return "invalid argument";
    case HIPNUC_KBOOT_ERR_WRITE:    return "serial write failed";
    case HIPNUC_KBOOT_ERR_TIMEOUT:  return "no reply from bootloader";
    case HIPNUC_KBOOT_ERR_FRAME:    return "invalid framing packet";
    case HIPNUC_KBOOT_ERR_NAK:      return "bootloader NAK";
    case HIPNUC_KBOOT_ERR_ABORT:    return "bootloader abort";
    case HIPNUC_KBOOT_ERR_RESPONSE: return "unexpected response";
    case HIPNUC_KBOOT_ERR_STATUS:   return "bootloader reported an error status";
    case HIPNUC_KBOOT_ERR_SIZE:     return "image or packet too large";
    case HIPNUC_KBOOT_ERR_READ:     return "serial read failed";
    default:                        return "unknown error";
    }
}
