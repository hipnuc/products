/* Deterministic transport: exercise call deadlines and retained receive bytes. */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "hipnuc_serial.h"
#include "serial_io.h"

static uint8_t input[4096];
static size_t input_size, input_offset, chunk;
static uint64_t now_ms;
static int noise, disconnected, interrupted;

uint64_t hipnuc_serial_monotonic_ms(void) { return now_ms; }
int hipnuc_serial_read_bytes(hipnuc_serial_t *device, uint8_t *data, size_t cap,
                             int timeout_ms)
{
    size_t count = input_size - input_offset;
    if (disconnected) return -1;
    if (interrupted) { --interrupted; ++now_ms; return 0; }
    if (noise) {
        ++now_ms;
        memset(data, 0xFF, cap);
        device->bytes_received += cap;
        return (int)cap;
    }
    if (!count) { now_ms += (unsigned)timeout_ms; return 0; }
    if (count > cap) count = cap;
    if (chunk && count > chunk) count = chunk;
    memcpy(data, input + input_offset, count);
    input_offset += count;
    device->bytes_received += count;
    return (int)count;
}

static void append_frame(const uint8_t *payload, size_t size)
{
    uint8_t *p = input + input_size;
    uint16_t crc;
    p[0] = 0x5a; p[1] = 0xa5; p[2] = (uint8_t)size; p[3] = (uint8_t)(size >> 8);
    memcpy(p + 6, payload, size);
    crc = hipnuc_crc16(hipnuc_crc16(0, p, 4), payload, size);
    p[4] = (uint8_t)crc; p[5] = (uint8_t)(crc >> 8);
    input_size += size + 6;
}

static void reset_input(void)
{
    input_size = input_offset = chunk = 0;
    noise = disconnected = interrupted = 0;
    now_ms = 0;
}

int main(void)
{
    const uint8_t hi83[] = {0x83, 0, 0, 0, 0, 0, 0, 0};
    const char gga[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    hipnuc_serial_t device = {0};
    hipnuc_sample_t sample;
    unsigned i;
    device.is_open = 1;
    for (i = 0; i < 5; ++i) append_frame(hi83, sizeof(hi83));
    for (i = 0; i < 5; ++i) {
        assert(hipnuc_serial_read_sample(&device, &sample, 0) == 1);
        assert(sample.source == HIPNUC_SOURCE_HI83);
    }
    assert(device.samples_received == 5);
    assert(device.bytes_received == input_size);
    assert(hipnuc_serial_read_sample(&device, &sample, 0) == 0);

    /* A checksum-valid NMEA sentence inside a binary envelope is not input
     * to the NMEA parser, even when that binary packet is unsupported. */
    reset_input();
    append_frame((const uint8_t *)gga, strlen(gga));
    memcpy(input + input_size, gga, strlen(gga)); input_size += strlen(gga);
    chunk = 3;
    assert(hipnuc_serial_read_sample(&device, &sample, 100) == 1);
    assert(sample.source == HIPNUC_SOURCE_NMEA_GGA);
    assert(device.nmea.sentence_count == 1);
    assert(hipnuc_serial_read_sample(&device, &sample, 20) == 0);
    assert(now_ms == 20);

    reset_input();
    interrupted = 3;
    assert(hipnuc_serial_read_sample(&device, &sample, 25) == 0);
    assert(now_ms == 25); /* Early OS wakeups are not the call's timeout. */

    reset_input();
    noise = 1;
    assert(hipnuc_serial_read_sample(&device, &sample, 25) == 0);
    assert(now_ms == 25); /* Continuous bytes never extend the deadline. */
    disconnected = 1;
    assert(hipnuc_serial_read_sample(&device, &sample, 25) == -1);
    assert(hipnuc_serial_read_sample(&device, &sample, -1) == -1);
    device.is_open = 0;
    assert(hipnuc_serial_read_sample(&device, &sample, 0) == -1);
    return 0;
}
