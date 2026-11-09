/* OS calls live here; protocol decoding and customer code stay portable. */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#else
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#endif
#include "hipnuc_serial_io.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <asm/termbits.h>
#include <linux/serial.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#endif

#ifndef _WIN32
/* Cumulative driver counters, baselined at open; not every driver has them. */
static int receive_error_total(int fd, uint32_t *total)
{
    struct serial_icounter_struct counters;
    if (ioctl(fd, TIOCGICOUNT, &counters) != 0) return 0;
    *total = (uint32_t)counters.overrun + (uint32_t)counters.buf_overrun +
             (uint32_t)counters.frame + (uint32_t)counters.parity;
    return 1;
}

/* Lost bytes are not a transport failure: the decoder resynchronises. */
static void poll_receive_errors(hipnuc_serial_t *device)
{
    uint32_t total = device->error_snapshot;
    if (!receive_error_total((int)device->handle, &total)) return;
    if (total > device->error_snapshot) device->rx_errors += total - device->error_snapshot;
    device->error_snapshot = total;
}
#endif

static int io_error(hipnuc_serial_t *device, const char *operation)
{
#ifdef _WIN32
    DWORD code = GetLastError();
    char detail[160] = {0};
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                   NULL, code, 0, detail, sizeof(detail), NULL);
    detail[strcspn(detail, "\r\n")] = '\0';
    snprintf(device->error, sizeof(device->error), "%s: %s (Windows error %lu)",
             operation, detail, (unsigned long)code);
#else
    snprintf(device->error, sizeof(device->error), "%s: %s", operation, strerror(errno));
#endif
    return -1;
}

uint64_t hipnuc_serial_io_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (uint64_t)time.tv_sec * 1000 + (uint64_t)time.tv_nsec / 1000000;
#endif
}

void hipnuc_serial_io_close(hipnuc_serial_t *device)
{
    if (!device->is_open) return;
#ifdef _WIN32
    CloseHandle((HANDLE)device->handle);
#else
    ioctl((int)device->handle, TIOCNXCL);
    close((int)device->handle);
#endif
    device->is_open = 0;
}

#ifdef _WIN32
/* MAXDWORD/0/0 is nonblocking. With a positive constant, MAXDWORD for BOTH
 * interval and multiplier waits only for the first byte. Writes wait at most
 * 100 ms for the driver to take a command. */
static int set_timeouts(HANDLE handle, int read_timeout_ms)
{
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = read_timeout_ms > 0 ? MAXDWORD : 0;
    timeouts.ReadTotalTimeoutConstant = (DWORD)read_timeout_ms;
    timeouts.WriteTotalTimeoutConstant = 100;
    return SetCommTimeouts(handle, &timeouts);
}
#else
/* Line speed in bits per second, any value the driver can make. The kernel reads
 * c_ispeed and c_ospeed only when the baud bits of c_cflag are BOTHER. */
static int set_rate(int fd, struct termios2 *config, unsigned rate)
{
    config->c_cflag = (config->c_cflag & ~(CBAUD | CIBAUD)) | BOTHER;
    config->c_ispeed = config->c_ospeed = rate;
    return ioctl(fd, TCSETS2, config);
}
#endif

int hipnuc_serial_io_open(hipnuc_serial_t *device, const char *port, int baudrate)
{
#ifdef _WIN32
    {
        char path[256];
        wchar_t wide_path[256];
        HANDLE handle;
        DCB config = {0};
        int count = snprintf(path, sizeof(path), "%s%s",
                             strncmp(port, "\\\\.\\", 4) == 0 ? "" : "\\\\.\\", port);
        if (count < 0 || (size_t)count >= sizeof(path) ||
            !MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1,
                                wide_path, sizeof(wide_path) / sizeof(wide_path[0]))) {
            snprintf(device->error, sizeof(device->error), "Invalid or too long port name");
            return -1;
        }
        handle = CreateFileW(wide_path, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                             OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (handle == INVALID_HANDLE_VALUE) return io_error(device, "Cannot open port");
        device->handle = (intptr_t)handle;
        device->is_open = 1;
        config.DCBlength = sizeof(config);
        if (!GetCommState(handle, &config)) goto configure_error;
        config.BaudRate = (DWORD)baudrate;
        config.ByteSize = 8;
        config.Parity = NOPARITY;
        config.StopBits = ONESTOPBIT;
        config.fBinary = TRUE;
        config.fParity = FALSE;
        config.fOutxCtsFlow = config.fOutxDsrFlow = FALSE;
        config.fDtrControl = DTR_CONTROL_DISABLE;
        config.fDsrSensitivity = FALSE;
        config.fTXContinueOnXoff = TRUE;
        config.fOutX = config.fInX = FALSE;
        config.fErrorChar = config.fNull = FALSE;
        config.fRtsControl = RTS_CONTROL_DISABLE;
        config.fAbortOnError = FALSE;
        if (!SetCommState(handle, &config) || !set_timeouts(handle, 0)) goto configure_error;
    }
#else
    {
        struct termios2 config;
        struct serial_struct serial;
        unsigned rate = (unsigned)baudrate;
        int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) return io_error(device, "Cannot open port");
        device->handle = fd;
        device->is_open = 1;
        /* TIOCEXCL stops later opens; flock also detects an earlier reader. */
        if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
            /* Close without TIOCNXCL: the other owner's exclusivity stays. */
            snprintf(device->error, sizeof(device->error), "Port is in use by another program");
            close(fd);
            device->is_open = 0;
            return -1;
        }
        if (ioctl(fd, TIOCEXCL) < 0 || ioctl(fd, TCGETS2, &config) < 0)
            goto configure_error;
        config.c_iflag = 0;
        config.c_oflag = 0;
        config.c_lflag = 0;
        config.c_cflag = BOTHER | CS8 | CREAD | CLOCAL;
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 0;
        /* Linux cdc_acm sends the line settings to the adapter only when they
         * change. A CH347 reopened at the same rate then delivers its input
         * seconds late for the whole session. Passing through another rate
         * sends them; input that arrived before is stale. */
        if (set_rate(fd, &config, rate == 9600 ? 19200 : 9600) < 0 ||
            set_rate(fd, &config, rate) < 0)
            goto configure_error;
        ioctl(fd, TCFLSH, TCIFLUSH);
        /* FTDI adapters otherwise hold input for up to 16 ms; other drivers
         * ignore the flag or refuse the request. */
        if (ioctl(fd, TIOCGSERIAL, &serial) == 0) {
            serial.flags |= ASYNC_LOW_LATENCY;
            ioctl(fd, TIOCSSERIAL, &serial);
        }
        /* Drivers may round the requested rate to a hardware divider. */
        receive_error_total(fd, &device->error_snapshot);
    }
#endif
    return 0;

configure_error:
    io_error(device, "Cannot configure serial port");
    hipnuc_serial_io_close(device);
    return -1;
}

size_t hipnuc_serial_io_write(hipnuc_serial_t *device, const uint8_t *data, size_t len)
{
    if (!device->is_open || !len || len > INT_MAX) return 0;
#ifdef _WIN32
    {
        DWORD done = 0;
        if (!WriteFile((HANDLE)device->handle, data, (DWORD)len, &done, NULL)) io_error(device, "Serial write failed");
        return (size_t)done;
    }
#else
    {
        ssize_t done = write((int)device->handle, data, len);
        if (done < 0 && errno != EAGAIN && errno != EINTR) io_error(device, "Serial write failed");
        return done > 0 ? (size_t)done : 0;
    }
#endif
}

#ifndef _WIN32
static int wait_port(hipnuc_serial_t *device, short events, int timeout_ms)
{
    struct pollfd descriptor;
    int result;
    descriptor.fd = (int)device->handle;
    descriptor.events = events;
    descriptor.revents = 0;
    result = poll(&descriptor, 1, timeout_ms);
    /* The caller decides whether to retry an interrupted wait. */
    if (result < 0 && errno == EINTR) return 0;
    if (result < 0) return io_error(device, "Serial poll failed");
    if (result == 0) return 0;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        snprintf(device->error, sizeof(device->error), "Serial port disconnected");
        return -1;
    }
    return (descriptor.revents & events) != 0;
}
#endif

int hipnuc_serial_io_read(hipnuc_serial_t *device, uint8_t *data, size_t size, int timeout_ms)
{
    if (!device) return -1;
    if (!device->is_open || !data || !size || size > INT_MAX || timeout_ms < 0) {
        snprintf(device->error, sizeof(device->error), "Invalid serial read arguments");
        return -1;
    }
#ifdef _WIN32
    {
        DWORD got = 0, errors = 0;
        COMSTAT status;
        HANDLE handle = (HANDLE)device->handle;
        if (!ClearCommError(handle, &errors, &status)) return io_error(device, "Cannot inspect serial port");
        /* The condition is already cleared. */
        if (errors & (CE_OVERRUN | CE_RXOVER | CE_FRAME | CE_RXPARITY)) device->rx_errors++;
        if (!set_timeouts(handle, timeout_ms) || !ReadFile(handle, data, (DWORD)size, &got, NULL))
            return io_error(device, "Serial read failed");
        return (int)got;
    }
#else
    {
        ssize_t got;
        int ready = wait_port(device, POLLIN, timeout_ms);
        poll_receive_errors(device);
        if (ready <= 0) return ready;
        got = read((int)device->handle, data, size);
        if (got < 0 && (errno == EAGAIN || errno == EINTR)) return 0;
        if (got < 0) return io_error(device, "Serial read failed");
        if (got == 0) {
            snprintf(device->error, sizeof(device->error), "Serial port closed while reading");
            return -1;
        }
        return (int)got;
    }
#endif
}
