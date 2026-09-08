/* Internal byte transport shared by the sample reader and firmware updater.
 * Never mix byte reads with read_sample() on the same open connection. */
#ifndef HIPNUC_SERIAL_IO_H
#define HIPNUC_SERIAL_IO_H
#include "hipnuc_serial.h"

int hipnuc_serial_read_bytes(hipnuc_serial_t *device, uint8_t *data, size_t size,
                             int timeout_ms);
int hipnuc_serial_write_bytes(hipnuc_serial_t *device, const uint8_t *data,
                              size_t size, int timeout_ms);
int hipnuc_serial_flush_input(hipnuc_serial_t *device);
uint64_t hipnuc_serial_monotonic_ms(void);
void hipnuc_serial_sleep_ms(unsigned milliseconds);
#endif
