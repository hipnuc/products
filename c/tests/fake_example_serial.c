/* Only the transport is replaced; both public example programs run unchanged. */
#include "hipnuc_serial.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int calls;
static int scene(const char *name)
{
    const char *value = getenv("HIPNUC_EXAMPLE_SCENE");
    return value && strcmp(value, name) == 0;
}

int hipnuc_serial_open(hipnuc_serial_t *device, const char *port, int baudrate)
{
    (void)port;
    (void)baudrate;
    if (scene("open_error")) return -1;
    device->is_open = 1;
    return 0;
}

int hipnuc_serial_read_sample(hipnuc_serial_t *device, hipnuc_sample_t *sample, int timeout_ms)
{
    (void)device;
    (void)timeout_ms;
    ++calls;
    if (calls == 3) {
        memset(sample, 0, sizeof(*sample));
        sample->source = HIPNUC_SOURCE_HI83;
        sample->valid = HIPNUC_VALID_ROLL_PITCH;
        sample->roll = 0.25f;
        sample->pitch = -0.5f;
        if (scene("interrupt")) raise(SIGINT);
        return 1;
    }
    return calls < 6 ? 0 : -1;
}

const char *hipnuc_serial_last_error(const hipnuc_serial_t *device)
{
    (void)device;
    return "Test transport unavailable";
}

void hipnuc_serial_close(hipnuc_serial_t *device)
{
    if (device->is_open) fputs("TEST_CLOSED\n", stderr);
    device->is_open = 0;
}
