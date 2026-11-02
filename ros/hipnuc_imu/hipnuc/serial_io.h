/* Internal Windows/Linux byte transport for the sample reader.
 * Never mix byte reads with read_sample() on the same open connection. */
#ifndef HIPNUC_SERIAL_IO_H
#define HIPNUC_SERIAL_IO_H
#include "hipnuc_serial.h"

/* Returns the number of bytes read, 0 when none arrived yet (deadline, signal
 * or early wakeup; not necessarily a timeout), or -1 on failure. */
int hipnuc_serial_read_bytes(hipnuc_serial_t *device, uint8_t *data, size_t size,
                             int timeout_ms);
uint64_t hipnuc_serial_monotonic_ms(void);
#endif
