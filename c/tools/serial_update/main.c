/* Explicit serial firmware update. No automatic reset after failure. */
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hexfile.h"
#include "hipnuc_kboot.h"
#include "hipnuc_serial.h"
#include "serial_io.h"

static volatile sig_atomic_t stopped;
static void stop(int number) { (void)number; stopped = 1; }
typedef struct {
    hipnuc_serial_t device;
    int last_percent;
    int transport_failed;
} update_port_t;

static int send_bytes(void *user, const uint8_t *data, size_t size)
{
    update_port_t *port = user;
    return stopped || port->transport_failed ? -1 :
        hipnuc_serial_write_bytes(&port->device, data, size, 2000);
}
static int read_bytes(void *user, uint8_t *data, size_t size, uint32_t timeout)
{
    update_port_t *port = user;
    uint64_t start = hipnuc_serial_monotonic_ms();
    int polled = 0;
    for (;;) {
        uint64_t elapsed;
        uint32_t remaining;
        int result;
        if (stopped || port->transport_failed) return -1;
        elapsed = hipnuc_serial_monotonic_ms() - start;
        if (polled && elapsed >= timeout) return 0;
        remaining = elapsed < timeout ? timeout - (uint32_t)elapsed : 0;
        /* Short OS waits keep cancellation responsive during flash erase.
         * Idle wakeups do not restart the caller's total timeout. */
        result = hipnuc_serial_read_bytes(&port->device, data, size,
                                          remaining > 50 ? 50 : (int)remaining);
        polled = 1;
        if (stopped) return -1;
        if (result != 0) return result;
    }
}
static void flush_input(void *user)
{
    update_port_t *port = user;
    /* The bootloader callback has no return value. Latch a failed purge so
     * the next transfer fails without accepting stale replies. */
    if (!port->transport_failed && hipnuc_serial_flush_input(&port->device) < 0)
        port->transport_failed = 1;
}
static void delay(void *user, uint32_t milliseconds)
{
    (void)user;
    while (milliseconds && !stopped) {
        unsigned part = milliseconds > 50 ? 50 : milliseconds;
        hipnuc_serial_sleep_ms(part);
        milliseconds -= part;
    }
}
static void progress(void *user, uint32_t written, uint32_t total)
{
    update_port_t *port = user;
    int percent = total ? (int)((uint64_t)written * 100 / total) : 100;
    if (percent != port->last_percent) {
        fprintf(stderr, "\rWriting: %d%%", percent);
        fflush(stderr);
        port->last_percent = percent;
    }
}
static void message(void *user, const char *text)
{
    (void)user;
    fprintf(stderr, "%s\n", text);
}
static void help(void)
{
    puts("Usage: hipnuc-update FILE.hex -p PORT -b BAUD\n"
         "Update one explicitly selected device using its Intel HEX firmware.\n"
         "  -p, --port PORT       COM3 or /dev/ttyUSB0\n"
         "  -b, --baudrate BAUD   Current connection speed\n"
         "  -h, --help            Show this help\n"
         "A failed update stops without an automatic reset. Ctrl-C cancels.");
}
int main(int argc, char **argv)
{
    const char *path = NULL, *port_name = NULL;
    int baudrate = 0, result, i;
    hex_image_t image = {0};
    update_port_t port = {{0}, -1, 0};
    hipnuc_kboot_port_t callbacks = {send_bytes, read_bytes, flush_input,
                                    delay, progress, message, &port};
    hipnuc_kboot_ctx_t updater;
    if (argc == 1) { help(); return 0; }
    for (i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h") || !strcmp(argv[i], "help")) {
            help(); return 0;
        }
        if ((!strcmp(argv[i], "-p") || !strcmp(argv[i], "--port")) && i + 1 < argc) {
            if (port_name) goto arguments;
            port_name = argv[++i];
        } else if ((!strcmp(argv[i], "-b") || !strcmp(argv[i], "--baudrate")) && i + 1 < argc) {
            char *end;
            long value;
            if (baudrate) goto arguments;
            errno = 0;
            value = strtol(argv[++i], &end, 10);
            if (errno || !*argv[i] || *end || value <= 0 || value > INT_MAX) goto arguments;
            baudrate = (int)value;
        } else if (argv[i][0] != '-' && !path) path = argv[i];
        else goto arguments;
    }
    if (!path || !port_name || !*port_name || !baudrate) goto arguments;
    if (hexfile_load(path, &image) < 0) return 1;
    signal(SIGINT, stop);
    signal(SIGTERM, stop);
    if (hipnuc_serial_open(&port.device, port_name, baudrate) < 0) {
        fprintf(stderr, "%s: %s\n", port_name, hipnuc_serial_last_error(&port.device));
        hex_image_free(&image);
        return stopped ? 130 : 1;
    }
    fprintf(stderr, "%s at %d baud; %u firmware bytes at 0x%08X\n",
             port_name, baudrate, (unsigned)image.size, (unsigned)image.start_addr);
    /* An application enters its bootloader; an already running bootloader
     * ignores this ASCII command. Bootloader replies are checked below. */
    if (send_bytes(&port, (const uint8_t *)"REBOOT BL\r\n", 11) < 0)
        result = HIPNUC_KBOOT_ERR_WRITE;
    else {
        delay(&port, 50);
        hipnuc_kboot_init(&updater, &callbacks);
        result = hipnuc_kboot_update(&updater, image.start_addr, image.data, image.size);
    }
    if (port.last_percent >= 0) fputc('\n', stderr);
    if (result == HIPNUC_KBOOT_OK)
        fprintf(stderr, "Transfer and reset acknowledged. New application startup was not verified.\n");
    else if (stopped)
        fprintf(stderr, "Update cancelled. No automatic reset was sent after cancellation.\n");
    else {
        fprintf(stderr, "Update failed: %s\n", hipnuc_kboot_strerror(result));
        if (result == HIPNUC_KBOOT_ERR_STATUS)
            fprintf(stderr, "Device status: %u\n", (unsigned)updater.last_status);
        if (*hipnuc_serial_last_error(&port.device))
            fprintf(stderr, "%s\n", hipnuc_serial_last_error(&port.device));
    }
    hipnuc_serial_close(&port.device);
    hex_image_free(&image);
    return stopped ? 130 : result == HIPNUC_KBOOT_OK ? 0 : 1;
arguments:
    fprintf(stderr, "Error: use hipnuc-update FILE.hex -p PORT -b BAUD; see --help.\n");
    return 2;
}
