#ifndef _WIN32
#define _XOPEN_SOURCE 600
#endif
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <string.h>
#include "hipnuc_serial.h"
#include "serial_io.h"
#ifndef _WIN32
#include <fcntl.h>
#include <pty.h>
#include <sys/wait.h>
#include <unistd.h>

static void independent_connections(void)
{
    const char gga[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    hipnuc_serial_t first = {0}, second = {0};
    hipnuc_sample_t sample;
    int master_a, master_b, slave;
    char name_a[128], name_b[128];
    assert(openpty(&master_a, &slave, name_a, NULL, NULL) == 0);
    close(slave);
    assert(openpty(&master_b, &slave, name_b, NULL, NULL) == 0);
    close(slave);
    assert(hipnuc_serial_open(&first, name_a, 9600) == 0);
    assert(hipnuc_serial_open(&second, name_b, 115200) == 0);
    /* Retain a partial sentence on the first port while a complete sample
     * arrives independently on the second. Closing one never closes both. */
    assert(write(master_a, gga, 10) == 10);
    assert(hipnuc_serial_read_sample(&first, &sample, 10) == 0);
    assert(write(master_b, gga, sizeof(gga) - 1) == (ssize_t)sizeof(gga) - 1);
    assert(hipnuc_serial_read_sample(&second, &sample, 100) == 1);
    assert(sample.source == HIPNUC_SOURCE_NMEA_GGA);
    hipnuc_serial_close(&second);
    assert(first.samples_received == 0 && first.bytes_received == 10);
    assert(write(master_a, gga + 10, sizeof(gga) - 11) == (ssize_t)sizeof(gga) - 11);
    assert(hipnuc_serial_read_sample(&first, &sample, 100) == 1);
    assert(sample.source == HIPNUC_SOURCE_NMEA_GGA);
    assert(first.samples_received == 1 && second.samples_received == 1);
    assert(first.bytes_received == sizeof(gga) - 1);
    hipnuc_serial_close(&first);
    close(master_a);
    close(master_b);
}
#endif

int main(void)
{
    hipnuc_serial_t device = {0};
    hipnuc_sample_t sample;
    hipnuc_serial_close(&device);
    assert(hipnuc_serial_open(&device, "hipnuc-no-such-port", 115200) == -1);
    assert(!device.is_open && *hipnuc_serial_last_error(&device));
    hipnuc_serial_close(&device);
    assert(*hipnuc_serial_last_error(&device));
    assert(hipnuc_serial_read_sample(&device, &sample, 0) == -1);
#ifndef _WIN32
    independent_connections();
    {
        int master, slave, saved_stdin;
        char name[128];
        uint8_t bytes[8];
        uint64_t start;
        intptr_t handle;
        assert(openpty(&master, &slave, name, NULL, NULL) == 0);
        close(slave);
        assert(hipnuc_serial_open(&device, name, 256000) == 0);
        handle = device.handle;
        assert(hipnuc_serial_open(&device, name, 9600) == -1);
        assert(device.is_open && device.handle == handle);
        assert(write(master, "abc", 3) == 3);
        assert(hipnuc_serial_read_bytes(&device, bytes, sizeof(bytes), 100) == 3);
        assert(memcmp(bytes, "abc", 3) == 0);
        start = hipnuc_serial_monotonic_ms();
        assert(hipnuc_serial_read_sample(&device, &sample, 30) == 0);
        assert(hipnuc_serial_monotonic_ms() - start >= 25);
        assert(hipnuc_serial_monotonic_ms() - start < 500);
        hipnuc_serial_close(&device);
        /* A valid POSIX descriptor can be zero; it is not an open flag. */
        saved_stdin = dup(STDIN_FILENO);
        close(STDIN_FILENO);
        assert(hipnuc_serial_open(&device, name, 115200) == 0);
        assert(device.handle == STDIN_FILENO);
        close(master);
        assert(hipnuc_serial_read_sample(&device, &sample, 30) == -1);
        hipnuc_serial_close(&device);
        assert(fcntl(STDIN_FILENO, F_GETFD) == -1);
        if (saved_stdin >= 0) { assert(dup2(saved_stdin, STDIN_FILENO) == 0); close(saved_stdin); }
    }
#endif
    return 0;
}
