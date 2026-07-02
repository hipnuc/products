#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../can_interface.h"
#include "../commands.h"
#include "../config.h"
#include "../help.h"
#include "../log.h"
#include "../utils.h"
#include "hipnuc_can_update.h"

typedef struct {
    int fd;
} update_port_ctx_t;

typedef struct {
    const char *file_path;
    bool raw_binary;
    bool continue_on_error;
} update_args_t;

typedef struct {
    uint8_t *data;
    uint32_t size;
    uint32_t start_addr;
} firmware_image_t;

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

static int parse_hex_line(const char *line, uint8_t *bytes, size_t max_bytes, size_t *out_len)
{
    size_t len;
    if (!line || line[0] != ':' || !bytes || !out_len) {
        return -1;
    }
    len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        len--;
    }
    if ((len - 1U) % 2U != 0U) {
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

static int ensure_image_capacity(firmware_image_t *image, uint32_t needed)
{
    if (needed <= image->size) {
        return 0;
    }

    uint8_t *next = realloc(image->data, needed);
    if (!next) {
        return -1;
    }
    memset(next + image->size, 0, needed - image->size);
    image->data = next;
    image->size = needed;
    return 0;
}

static int load_bin_file(const char *path, firmware_image_t *image)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        log_error("Failed to open %s: %s", path, strerror(errno));
        return -1;
    }
    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }
    long len = ftell(fp);
    if (len <= 0 || len > 0x7FFFFFFFL) {
        fclose(fp);
        log_error("Invalid firmware size");
        return -1;
    }
    rewind(fp);

    image->data = malloc((size_t)len);
    if (!image->data) {
        fclose(fp);
        return -1;
    }
    image->size = (uint32_t)len;
    image->start_addr = 0;
    if (fread(image->data, 1, (size_t)len, fp) != (size_t)len) {
        fclose(fp);
        free(image->data);
        memset(image, 0, sizeof(*image));
        log_error("Failed to read firmware file");
        return -1;
    }
    fclose(fp);
    return 0;
}

static int load_hex_file(const char *path, firmware_image_t *image)
{
    FILE *fp = fopen(path, "r");
    char line[1024];
    uint32_t upper = 0;
    uint32_t origin = 0xFFFFFFFFU;
    int line_no = 0;

    if (!fp) {
        log_error("Failed to open %s: %s", path, strerror(errno));
        return -1;
    }

    while (fgets(line, sizeof(line), fp)) {
        uint8_t bytes[300];
        size_t n = 0;
        uint8_t data_len;
        uint16_t addr;
        uint8_t type;
        uint8_t sum = 0;

        line_no++;
        if (parse_hex_line(line, bytes, sizeof(bytes), &n) < 0 || n < 5) {
            log_error("Invalid HEX line %d", line_no);
            fclose(fp);
            return -1;
        }
        data_len = bytes[0];
        if (n != (size_t)data_len + 5U) {
            log_error("Invalid HEX length at line %d", line_no);
            fclose(fp);
            return -1;
        }
        for (size_t i = 0; i < n; ++i) {
            sum = (uint8_t)(sum + bytes[i]);
        }
        if (sum != 0) {
            log_error("HEX checksum failed at line %d", line_no);
            fclose(fp);
            return -1;
        }

        addr = (uint16_t)(((uint16_t)bytes[1] << 8) | bytes[2]);
        type = bytes[3];

        if (type == 0x00U) {
            uint32_t abs_addr = upper + addr;
            uint32_t offset;
            if (origin == 0xFFFFFFFFU) {
                origin = abs_addr;
                image->start_addr = origin;
            }
            if (abs_addr < origin) {
                log_error("HEX records must be in ascending address order");
                fclose(fp);
                return -1;
            }
            offset = abs_addr - origin;
            if (ensure_image_capacity(image, offset + data_len) < 0) {
                fclose(fp);
                return -1;
            }
            memcpy(image->data + offset, &bytes[4], data_len);
        } else if (type == 0x01U) {
            fclose(fp);
            return image->size > 0U ? 0 : -1;
        } else if (type == 0x02U && data_len == 2U) {
            upper = (uint32_t)(((uint16_t)bytes[4] << 8) | bytes[5]) << 4;
        } else if (type == 0x04U && data_len == 2U) {
            upper = (uint32_t)(((uint16_t)bytes[4] << 8) | bytes[5]) << 16;
        } else if (type == 0x03U || type == 0x05U) {
            continue;
        } else {
            log_error("Unsupported HEX record type 0x%02X at line %d", type, line_no);
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    return image->size > 0U ? 0 : -1;
}

static void firmware_image_free(firmware_image_t *image)
{
    if (!image) {
        return;
    }
    free(image->data);
    memset(image, 0, sizeof(*image));
}

static int update_send(void *user, const hipnuc_can_frame_t *frame)
{
    update_port_ctx_t *ctx = (update_port_ctx_t *)user;
    return can_send_frame(ctx->fd, frame);
}

static int update_wait(void *user, uint32_t can_id,
                       hipnuc_can_frame_t *frame, uint32_t timeout_ms)
{
    update_port_ctx_t *ctx = (update_port_ctx_t *)user;
    uint32_t start = utils_get_timestamp_ms();
    while ((uint32_t)(utils_get_timestamp_ms() - start) < timeout_ms) {
        int ret = can_receive_frame(ctx->fd, frame);
        if (ret < 0) {
            return -1;
        }
        if (ret == 0) {
            continue;
        }
        if ((frame->can_id & HIPNUC_CAN_SFF_MASK) == (can_id & HIPNUC_CAN_SFF_MASK)) {
            return 0;
        }
    }
    return -1;
}

static void update_delay(void *user, uint32_t delay_ms)
{
    (void)user;
    utils_delay_ms(delay_ms);
}

static void update_progress(void *user, uint8_t node_id, uint8_t progress)
{
    (void)user;
    printf("\r[Node %u] Progress: %u%%   ", (unsigned)node_id, (unsigned)progress);
    fflush(stdout);
    if (progress >= 100U) {
        printf("\n");
    }
}

static void update_log(void *user, uint8_t node_id, const char *msg)
{
    (void)user;
    log_info("[Node %u] %s", (unsigned)node_id, msg ? msg : "");
}

static int parse_args(int argc, char **argv, update_args_t *args)
{
    memset(args, 0, sizeof(*args));
    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--file") == 0) && i + 1 < argc) {
            args->file_path = argv[++i];
        } else if (strcmp(argv[i], "--bin") == 0) {
            args->raw_binary = true;
        } else if (strcmp(argv[i], "--hex") == 0) {
            args->raw_binary = false;
        } else if (strcmp(argv[i], "--continue") == 0) {
            args->continue_on_error = true;
        } else {
            return -1;
        }
    }
    return args->file_path ? 0 : -1;
}

int cmd_update(int argc, char *argv[])
{
    update_args_t args;
    firmware_image_t image;
    uint8_t nodes[32];
    int node_count;
    const char *ifname;
    int fd;
    int overall = CANHOST_EXIT_OK;

    memset(&image, 0, sizeof(image));
    if (parse_args(argc, argv, &args) < 0) {
        help_print_arg_error_json("firmware update",
                                  "usage: firmware update -f <file.hex> [--bin] [--continue]");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    node_count = config_get_target_nodes(nodes, 32);
    if (node_count <= 0) {
        help_print_arg_error_json("firmware update", "no target node configured");
        return CANHOST_EXIT_INVALID_ARGS;
    }

    if (args.raw_binary) {
        if (load_bin_file(args.file_path, &image) < 0) {
            return CANHOST_EXIT_RUNTIME_ERROR;
        }
    } else {
        if (load_hex_file(args.file_path, &image) < 0) {
            firmware_image_free(&image);
            return CANHOST_EXIT_RUNTIME_ERROR;
        }
    }

    log_info("Firmware loaded: %s size=%u start=0x%08X",
             args.file_path, (unsigned)image.size, (unsigned)image.start_addr);

    ifname = config_get_interface();
    fd = can_open_socket(ifname);
    if (fd < 0) {
        help_print_can_setup(ifname);
        firmware_image_free(&image);
        return CANHOST_EXIT_RUNTIME_ERROR;
    }

    update_port_ctx_t port_ctx = { .fd = fd };
    can_update_port_t port = {
        .send = update_send,
        .wait = update_wait,
        .delay_ms = update_delay,
        .progress = update_progress,
        .log = update_log,
        .user = &port_ctx
    };
    can_update_ctx_t updater;
    can_update_init(&updater, &port);

    for (int i = 0; i < node_count; ++i) {
        int ret = can_update_update_node(&updater, nodes[i], image.data, image.size);
        if (ret != CAN_UPDATE_OK) {
            log_error("[Node %u] firmware update failed: %d", (unsigned)nodes[i], ret);
            overall = CANHOST_EXIT_RUNTIME_ERROR;
            if (!args.continue_on_error) {
                break;
            }
        }
    }

    can_close_socket(fd);
    firmware_image_free(&image);
    return overall;
}
