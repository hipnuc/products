#include "hipnuc_serial.h"
#include "serial_io.h"
#include <stdio.h>

const char *hipnuc_serial_last_error(const hipnuc_serial_t *device)
{
    return device ? device->error : "Missing serial connection";
}

int hipnuc_serial_read_sample(hipnuc_serial_t *device, hipnuc_sample_t *sample,
                              int timeout_ms)
{
    uint64_t deadline;
    int polled = 0;
    if (!device) return -1;
    if (!sample || !device->is_open || timeout_ms < 0) {
        snprintf(device->error, sizeof(device->error),
                 "Read requires an open connection, a sample and a nonnegative timeout");
        return -1;
    }
    device->error[0] = '\0';
    deadline = hipnuc_serial_monotonic_ms() + (unsigned)timeout_ms;
    for (;;) {
        while (device->received_offset < device->received_size) {
            uint8_t byte = device->received[device->received_offset++];
            int in_binary = device->binary.nbyte != 0;
            int decoded = hipnuc_input(&device->binary, byte);
            if (in_binary || device->binary.nbyte != 0) {
                /* A binary header also cancels a truncated NMEA sentence. */
                device->nmea.nbyte = 0;
                if (decoded > 0 && hipnuc_sample_from_raw(&device->binary, sample)) {
                    ++device->samples_received;
                    return 1;
                }
            } else if (nmea_input(&device->nmea, byte) > 0 &&
                       hipnuc_sample_from_nmea(&device->nmea, sample)) {
                ++device->samples_received;
                return 1;
            }
        }
        {
            uint64_t now = hipnuc_serial_monotonic_ms();
            int remaining = now < deadline ? (int)(deadline - now) : 0;
            int count;
            /* One nonblocking OS poll is permitted, even with timeout zero. */
            if (polled && remaining == 0) return 0;
            count = hipnuc_serial_read_bytes(device, device->received,
                                             sizeof(device->received), remaining);
            if (count < 0) return count;
            /* A signal or an early OS wakeup can yield no bytes before the
             * deadline. Only the total call deadline means timeout. */
            polled = 1;
            device->received_offset = 0;
            device->received_size = (size_t)count;
        }
    }
}
