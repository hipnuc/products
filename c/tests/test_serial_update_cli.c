/* CLI integration with fake image / serial / bootloader backends. The actual
 * updater main and its callbacks are included, so error cleanup, argument
 * ordering, cancellation and reset policy are exercised without hardware. */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hexfile.h"
#include "hipnuc_kboot.h"
#include "hipnuc_serial.h"
#include "serial_io.h"

static char diagnostics[8192];
static size_t diagnostic_size;

static int capture_fprintf(FILE *file, const char *format, ...)
{
    va_list arguments;
    int size;
    (void)file;
    va_start(arguments, format);
    size = vsnprintf(diagnostics + diagnostic_size, sizeof(diagnostics) - diagnostic_size, format, arguments);
    va_end(arguments);
    if (size > 0) {
        size_t available = sizeof(diagnostics) - diagnostic_size - 1;
        diagnostic_size += (size_t)size < available ? (size_t)size : available;
    }
    return size;
}
static int capture_fputc(int character, FILE *file)
{
    (void)file;
    if (diagnostic_size < sizeof(diagnostics) - 1) {
        diagnostics[diagnostic_size++] = (char)character;
        diagnostics[diagnostic_size] = 0;
    }
    return character;
}

#define main serial_cli_main
#define fprintf capture_fprintf
#define fputc capture_fputc
#include "../tools/serial_update/main.c"
#undef fputc
#undef fprintf
#undef main

enum scenario {
    SUCCESS, OPEN_FAILURE, BOOT_WRITE_FAILURE, TRANSFER_FAILURE,
    RECEIVE_FAILURE, FLUSH_FAILURE, INTERRUPT_DURING_READ, INTERRUPT_DURING_DELAY,
    IDLE_READ, EARLY_IDLE_WAKE, LATE_SHORT_READ, INTERRUPT_IDLE_READ
};
static enum scenario scenario;
static int loads, frees, opens, closes, writes, reads, flushes, updates, resets;
static int active, allocations;
static hipnuc_serial_t *opened_device;
static uint64_t now_ms;
static int maximum_read_wait;

int hexfile_load(const char *path, hex_image_t *image)
{
    ++loads;
    memset(image, 0, sizeof(*image));
    if (strcmp(path, "image.hex") != 0) return -1;
    image->data = malloc(16);
    assert(image->data);
    memset(image->data, 0xa5, 16);
    image->size = 16;
    image->start_addr = 0x08004000;
    ++allocations;
    return 0;
}
void hex_image_free(hex_image_t *image)
{
    ++frees;
    assert(image->data && allocations == 1);
    free(image->data);
    memset(image, 0, sizeof(*image));
    --allocations;
}
int hipnuc_serial_open(hipnuc_serial_t *device, const char *name, int baudrate)
{
    ++opens;
    assert(strcmp(name, "TEST_PORT") == 0 && baudrate == 115200);
    assert(!device->is_open);
    if (scenario == OPEN_FAILURE) {
        strcpy(device->error, "open failure");
        return -1;
    }
    device->is_open = 1;
    active = 1;
    opened_device = device;
    return 0;
}
void hipnuc_serial_close(hipnuc_serial_t *device)
{
    ++closes;
    assert(device == opened_device && device->is_open && active);
    device->is_open = 0;
    active = 0;
}
const char *hipnuc_serial_last_error(const hipnuc_serial_t *device)
{
    return device->error;
}
int hipnuc_serial_write_bytes(hipnuc_serial_t *device, const uint8_t *data,
                              size_t size, int timeout_ms)
{
    ++writes;
    assert(active && device == opened_device && timeout_ms == 2000);
    if (writes == 1) {
        assert(size == 11 && memcmp(data, "REBOOT BL\r\n", 11) == 0);
        if (scenario == BOOT_WRITE_FAILURE) {
            strcpy(device->error, "boot write failure");
            return -1;
        }
    }
    return (int)size;
}
int hipnuc_serial_read_bytes(hipnuc_serial_t *device, uint8_t *data, size_t size,
                             int timeout_ms)
{
    ++reads;
    assert(active && device == opened_device && size > 0 && timeout_ms >= 0);
    if (timeout_ms > maximum_read_wait) maximum_read_wait = timeout_ms;
    if (scenario == IDLE_READ || scenario == EARLY_IDLE_WAKE ||
        scenario == LATE_SHORT_READ || scenario == INTERRUPT_IDLE_READ) {
        unsigned wait = (unsigned)timeout_ms;
        if (scenario == EARLY_IDLE_WAKE && wait > 7) wait = 7;
        now_ms += wait;
        if (scenario == INTERRUPT_IDLE_READ) raise(SIGINT);
        if (scenario != LATE_SHORT_READ || reads < 3) return 0;
    }
    if (scenario == INTERRUPT_DURING_READ) raise(SIGINT);
    if (scenario == RECEIVE_FAILURE || scenario == INTERRUPT_DURING_READ) {
        strcpy(device->error, "receive failure");
        return -1;
    }
    data[0] = 0x5a;
    return 1;
}
int hipnuc_serial_flush_input(hipnuc_serial_t *device)
{
    ++flushes;
    assert(active && device == opened_device);
    if (scenario == FLUSH_FAILURE) {
        strcpy(device->error, "flush failure");
        return -1;
    }
    return 0;
}
void hipnuc_serial_sleep_ms(unsigned milliseconds)
{
    assert(milliseconds <= 50);
    now_ms += milliseconds;
    if (scenario == INTERRUPT_DURING_DELAY) raise(SIGINT);
}
uint64_t hipnuc_serial_monotonic_ms(void) { return now_ms; }
void hipnuc_kboot_init(hipnuc_kboot_ctx_t *context, const hipnuc_kboot_port_t *callbacks)
{
    memset(context, 0, sizeof(*context));
    context->port = *callbacks;
}
int hipnuc_kboot_reset(hipnuc_kboot_ctx_t *context)
{
    const uint8_t request = 0xb;
    ++resets;
    return context->port.write(context->port.user, &request, 1) < 0 ?
           HIPNUC_KBOOT_ERR_WRITE : HIPNUC_KBOOT_OK;
}
int hipnuc_kboot_update(hipnuc_kboot_ctx_t *context, uint32_t address,
                         const uint8_t *image, uint32_t size)
{
    uint8_t response = 0;
    const uint8_t request = 0xa6;
    ++updates;
    assert(active && address == 0x08004000 && size == 16 && image[0] == 0xa5);
    context->port.flush(context->port.user);
    // A failed flush must be latched by the CLI callback and block this write.
    if (context->port.write(context->port.user, &request, 1) < 0) return HIPNUC_KBOOT_ERR_WRITE;
    {
        int result = context->port.read(context->port.user, &response, 1, 100);
        if (result < 0) return HIPNUC_KBOOT_ERR_READ;
        if (!result) return HIPNUC_KBOOT_ERR_TIMEOUT;
    }
    if (scenario == TRANSFER_FAILURE) {
        context->last_status = 123;
        return HIPNUC_KBOOT_ERR_STATUS;
    }
    context->port.progress(context->port.user, size, size);
    return hipnuc_kboot_reset(context);
}
const char *hipnuc_kboot_strerror(int status)
{
    switch (status) {
    case HIPNUC_KBOOT_ERR_STATUS: return "device status failure";
    case HIPNUC_KBOOT_ERR_READ: return "serial read failure";
    case HIPNUC_KBOOT_ERR_WRITE: return "serial write failure";
    default: return "test status";
    }
}

static void reset_test(enum scenario selected)
{
    assert(!active && !allocations);
    scenario = selected;
    loads = frees = opens = closes = writes = reads = flushes = updates = resets = 0;
    opened_device = NULL;
    stopped = 0;  // repeated entry calls model independent CLI processes
    now_ms = 0;
    maximum_read_wait = 0;
    diagnostic_size = 0;
    diagnostics[0] = 0;
}
static int run_valid(enum scenario selected)
{
    char *arguments[] = {"hipnuc-update", "image.hex", "-p", "TEST_PORT", "-b", "115200"};
    reset_test(selected);
    int result = serial_cli_main(6, arguments);
    assert(!active && !allocations && frees == 1);
    return result;
}
static void arguments_before_io(void)
{
    char *missing_port[] = {"hipnuc-update", "image.hex", "-b", "115200"};
    char *missing_baud[] = {"hipnuc-update", "image.hex", "-p", "TEST_PORT"};
    char *bad_baud[] = {"hipnuc-update", "image.hex", "-p", "TEST_PORT", "-b", "999999999999999"};
    char *duplicate[] = {"hipnuc-update", "image.hex", "-p", "TEST_PORT", "-p", "TEST_PORT", "-b", "115200"};
    char *no_image[] = {"hipnuc-update", "-p", "TEST_PORT", "-b", "115200"};
    char *missing_file[] = {"hipnuc-update", "missing.hex", "-p", "TEST_PORT", "-b", "115200"};
    reset_test(SUCCESS);
    assert(serial_cli_main(4, missing_port) == 2);
    assert(serial_cli_main(4, missing_baud) == 2);
    assert(serial_cli_main(6, bad_baud) == 2);
    assert(serial_cli_main(8, duplicate) == 2);
    assert(serial_cli_main(5, no_image) == 2);
    assert(loads == 0 && opens == 0);
    assert(serial_cli_main(6, missing_file) == 1);
    assert(loads == 1 && opens == 0 && frees == 0 && allocations == 0);
}

static void bounded_reads(void)
{
    const enum scenario cases[] = {IDLE_READ, EARLY_IDLE_WAKE, LATE_SHORT_READ, INTERRUPT_IDLE_READ};
    size_t index;
    for (index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index) {
        update_port_t port = {0};
        uint8_t bytes[8];
        int result;
        reset_test(cases[index]);
        assert(hipnuc_serial_open(&port.device, "TEST_PORT", 115200) == 0);
        result = read_bytes(&port, bytes, sizeof(bytes), 125);
        assert(maximum_read_wait <= 50);
        if (scenario == INTERRUPT_IDLE_READ) {
            assert(result == -1 && now_ms == 50 && reads == 1);
        } else {
            assert(now_ms == 125);
            assert(result == (scenario == LATE_SHORT_READ ? 1 : 0));
            if (scenario == LATE_SHORT_READ) assert(reads == 3);
        }
        hipnuc_serial_close(&port.device);
    }
    {
        update_port_t port = {0};
        uint8_t byte;
        reset_test(IDLE_READ);
        assert(hipnuc_serial_open(&port.device, "TEST_PORT", 115200) == 0);
        assert(read_bytes(&port, &byte, 1, 0) == 0);
        assert(reads == 1 && now_ms == 0 && maximum_read_wait == 0);
        scenario = SUCCESS;
        assert(read_bytes(&port, &byte, 1, 0) == 1);
        assert(reads == 2 && now_ms == 0);
        hipnuc_serial_close(&port.device);
    }
}

int main(void)
{
    arguments_before_io();
    assert(run_valid(SUCCESS) == 0);
    assert(opens == 1 && closes == 1 && updates == 1 && resets == 1);
    assert(strstr(diagnostics, "startup was not verified"));
    assert(run_valid(OPEN_FAILURE) == 1);
    assert(opens == 1 && closes == 0 && writes == 0 && updates == 0 && resets == 0);
    assert(run_valid(BOOT_WRITE_FAILURE) == 1);
    assert(writes == 1 && closes == 1 && updates == 0 && resets == 0);
    assert(run_valid(TRANSFER_FAILURE) == 1);
    assert(closes == 1 && resets == 0 && strstr(diagnostics, "Device status: 123"));
    assert(run_valid(RECEIVE_FAILURE) == 1);
    assert(closes == 1 && resets == 0 && strstr(diagnostics, "receive failure"));
    assert(run_valid(INTERRUPT_DURING_READ) == 130);
    assert(closes == 1 && resets == 0 && strstr(diagnostics, "cancelled"));
    assert(run_valid(INTERRUPT_DURING_DELAY) == 130);
    assert(closes == 1 && writes == 1 && resets == 0);
    assert(run_valid(FLUSH_FAILURE) == 1);
    assert(flushes == 1 && writes == 1 && reads == 0 && resets == 0 && closes == 1);
    assert(strstr(diagnostics, "flush failure"));
    bounded_reads();
    puts("Serial updater CLI integration passed.");
    return 0;
}
