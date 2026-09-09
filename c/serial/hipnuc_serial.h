/* Synchronous desktop serial input (Windows/Linux), 8N1, no flow control. */
#ifndef HIPNUC_SERIAL_H
#define HIPNUC_SERIAL_H

#include <stddef.h>
#include <stdint.h>
#include "hipnuc_dec.h"
#include "nmea_dec.h"
#include "hipnuc_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Zero-initialize once. One owner per connection; do not copy an open object
 * or use it concurrently. is_open and counters are read-only diagnostics and
 * start again at each open; other fields are implementation state. A growing
 * receive_errors count means the host lost bytes, not that the port failed.
 * No heap allocation is needed. */
typedef struct {
    intptr_t handle;
    int is_open;
    char error[256];
    uint64_t bytes_received;
    uint64_t samples_received;
    uint64_t receive_errors;   /* UART overrun/framing/parity events: bytes were lost */
    uint32_t error_snapshot;
    hipnuc_raw_t binary;
    nmea_raw_t nmea;
    uint8_t received[1024];
    size_t received_size;
    size_t received_offset;
} hipnuc_serial_t;

/* Explicit port (COM3 or /dev/ttyUSB0) and baud rate. Returns 0 or -1.
 * Opening an already open object fails without closing its connection.
 * Drivers may round the baud rate; opening does not verify device communication. */
int hipnuc_serial_open(hipnuc_serial_t *device, const char *port, int baudrate);

/* Returns 1 for one NEW sample, 0 on timeout, -1 on transport/argument error.
 * timeout_ms is the total call deadline, not an inter-byte timeout; zero is
 * nonblocking. Remaining bytes stay in the object for the next call. Binary
 * frame payloads are never also interpreted as NMEA sentences. */
int hipnuc_serial_read_sample(hipnuc_serial_t *device, hipnuc_sample_t *sample,
                              int timeout_ms);

/* Object-owned error text; valid until the next operation on this object. */
const char *hipnuc_serial_last_error(const hipnuc_serial_t *device);

/* Releases the connection. Safe on a zero-initialized or closed object.
 * Retains the last error and counters for inspection. */
void hipnuc_serial_close(hipnuc_serial_t *device);

#ifdef __cplusplus
}
#endif
#endif
