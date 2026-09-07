/* update FILE: flash an Intel HEX image through the serial bootloader. */
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "global_options.h"
#include "hexfile.h"
#include "hihost_config.h"
#include "hipnuc_kboot.h"
#include "log.h"
#include "serial_port.h"

#define BAR_WIDTH 50

typedef struct {
    int fd;
    int last_percent;
} port_ctx_t;

static int port_write(void *user, const uint8_t *data, size_t len)
{
    port_ctx_t *ctx = (port_ctx_t *)user;
    return serial_port_write(ctx->fd, data, (int)len);
}

static int port_read(void *user, uint8_t *data, size_t cap, uint32_t timeout_ms)
{
    port_ctx_t *ctx = (port_ctx_t *)user;
    return serial_port_read_timeout(ctx->fd, data, (int)cap, (int)timeout_ms);
}

static void port_flush(void *user)
{
    port_ctx_t *ctx = (port_ctx_t *)user;
    tcflush(ctx->fd, TCIFLUSH);
}

static void port_delay(void *user, uint32_t ms)
{
    (void)user;
    safe_sleep((unsigned long)ms * 1000UL);
}

static void port_progress(void *user, uint32_t written, uint32_t total)
{
    port_ctx_t *ctx = (port_ctx_t *)user;
    int percent = total ? (int)((uint64_t)written * 100U / total) : 100;
    if (percent == ctx->last_percent) {
        return;
    }
    ctx->last_percent = percent;
    int filled = BAR_WIDTH * percent / 100;
    printf("\rWriting: [");
    for (int i = 0; i < BAR_WIDTH; ++i) {
        putchar(i < filled ? '#' : ' ');
    }
    printf("] %3d%%", percent);
    if (percent >= 100) {
        putchar('\n');
    }
    fflush(stdout);
}

static void port_log(void *user, const char *message)
{
    (void)user;
    log_info("%s", message);
}

int cmd_update(GlobalOptions *opts, int argc, char *argv[])
{
    if (argc != 1) {
        log_error("Usage: update FILE.hex");
        return -1;
    }

    hex_image_t img;
    if (hexfile_load(argv[0], &img) != 0) {
        return -1;
    }
    log_info("%s: %u bytes at 0x%08X", argv[0], (unsigned)img.size, (unsigned)img.start_addr);

    int fd = serial_port_open(opts->port_name);
    if (fd < 0 || serial_port_configure(fd, opts->baud_rate) < 0) {
        log_error("Cannot open %s at %d baud", opts->port_name, opts->baud_rate);
        if (fd >= 0) serial_port_close(fd);
        hex_image_free(&img);
        return -1;
    }

    /* Ask the application to enter the bootloader; a device already in the
     * bootloader ignores this text. */
    {
        char reply[256];
        serial_send_then_recv_str(fd, "REBOOT BL\r\n", NULL, reply, sizeof(reply), 10);
        safe_sleep(50 * 1000);
    }

    port_ctx_t port_ctx = { fd, -1 };
    hipnuc_kboot_port_t port = {
        .write = port_write,
        .read = port_read,
        .flush = port_flush,
        .delay_ms = port_delay,
        .progress = port_progress,
        .log = port_log,
        .user = &port_ctx
    };
    hipnuc_kboot_ctx_t ctx;
    hipnuc_kboot_init(&ctx, &port);

    int ret = hipnuc_kboot_update(&ctx, img.start_addr, img.data, img.size);
    if (ret != HIPNUC_KBOOT_OK) {
        if (port_ctx.last_percent >= 0 && port_ctx.last_percent < 100) {
            putchar('\n');
        }
        if (ret == HIPNUC_KBOOT_ERR_STATUS) {
            log_error("Update failed: %s (status %u)", hipnuc_kboot_strerror(ret), (unsigned)ctx.last_status);
        } else {
            log_error("Update failed: %s", hipnuc_kboot_strerror(ret));
        }
        if (ret != HIPNUC_KBOOT_ERR_TIMEOUT) {
            hipnuc_kboot_reset(&ctx);
        }
    } else {
        log_info("Firmware update complete; the device restarts into the new firmware.");
    }

    serial_port_close(fd);
    hex_image_free(&img);
    return ret == HIPNUC_KBOOT_OK ? 0 : -1;
}
