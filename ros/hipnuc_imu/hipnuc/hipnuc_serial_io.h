/* Internal Windows/Linux port access for hipnuc_serial.c. */
#ifndef HIPNUC_SERIAL_IO_H
#define HIPNUC_SERIAL_IO_H

#include "hipnuc_serial.h"

/* Open and configure the port (8N1, no flow control); sets handle and is_open.
 * Returns 0, or -1 with the reason in dev->error. */
int hipnuc_serial_io_open(hipnuc_serial_t *dev, const char *port, int baud);
void hipnuc_serial_io_close(hipnuc_serial_t *dev);

/* Returns the number of bytes read, 0 when none arrived yet (deadline, signal
 * or early wakeup), or -1 on failure. Lost-byte events the driver reports are
 * added to dev->rx_errors. */
int hipnuc_serial_io_read(hipnuc_serial_t *dev, uint8_t *data, size_t size, int timeout_ms);

/* Returns how many bytes the port accepted (0..len); Windows waits up to
 * 100 ms for room. */
size_t hipnuc_serial_io_write(hipnuc_serial_t *dev, const uint8_t *data, size_t len);

uint64_t hipnuc_serial_io_ms(void);   /* monotonic milliseconds */

#endif
