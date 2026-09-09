/* OS calls live here; protocol decoding and customer code stay portable. */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#else
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#endif
#include "serial_io.h"
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
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#endif

/* One lost-byte event: the partial frame is gone, the connection is not. */
static void note_receive_error(hipnuc_serial_t *device)
{
    ++device->receive_errors;
    device->binary.nbyte = 0;
    device->nmea.nbyte = 0;
}

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

static void poll_receive_errors(hipnuc_serial_t *device)
{
    uint32_t total = device->error_snapshot;
    if (!receive_error_total((int)device->handle, &total)) return;
    if (total > device->error_snapshot) note_receive_error(device);
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

uint64_t hipnuc_serial_monotonic_ms(void)
{
#ifdef _WIN32
    return (uint64_t)GetTickCount64();
#else
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (uint64_t)time.tv_sec * 1000 + (uint64_t)time.tv_nsec / 1000000;
#endif
}

void hipnuc_serial_close(hipnuc_serial_t *device)
{
    if (!device || !device->is_open) return;
#ifdef _WIN32
    CloseHandle((HANDLE)device->handle);
#else
    ioctl((int)device->handle, TIOCNXCL);
    close((int)device->handle);
#endif
    device->is_open = 0;
    device->received_size = device->received_offset = 0;
}

int hipnuc_serial_open(hipnuc_serial_t *device, const char *port, int baudrate)
{
    if (!device) return -1;
    if (device->is_open) {
        snprintf(device->error, sizeof(device->error), "Connection is already open");
        return -1;
    }
    if (!port || !*port || baudrate <= 0) {
        snprintf(device->error, sizeof(device->error), "Port and positive baud rate are required");
        return -1;
    }
    memset(device, 0, sizeof(*device));
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
        if (!SetCommState(handle, &config)) goto configure_error;
    }
#else
    {
        struct termios2 config;
        int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) return io_error(device, "Cannot open port");
        device->handle = fd;
        device->is_open = 1;
        if (ioctl(fd, TIOCEXCL) < 0 || ioctl(fd, TCGETS2, &config) < 0)
            goto configure_error;
        config.c_iflag = 0;
        config.c_oflag = 0;
        config.c_lflag = 0;
        config.c_cflag = BOTHER | CS8 | CREAD | CLOCAL;
        config.c_ispeed = config.c_ospeed = (unsigned)baudrate;
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 0;
        if (ioctl(fd, TCSETS2, &config) < 0) goto configure_error;
        /* termios2 supports product rates such as 256000 without rounding. */
        if (ioctl(fd, TCGETS2, &config) < 0) goto configure_error;
        if (config.c_ispeed != (unsigned)baudrate || config.c_ospeed != (unsigned)baudrate) {
            snprintf(device->error, sizeof(device->error), "Port did not accept %d baud", baudrate);
            hipnuc_serial_close(device);
            return -1;
        }
        receive_error_total(fd, &device->error_snapshot);
    }
#endif
    return 0;

configure_error:
    io_error(device, "Cannot configure serial port");
    hipnuc_serial_close(device);
    return -1;
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

int hipnuc_serial_read_bytes(hipnuc_serial_t *device, uint8_t *data, size_t size,
                             int timeout_ms)
{
    int count;
    if (!device) return -1;
    if (!device->is_open || !data || !size || size > INT_MAX || timeout_ms < 0) {
        snprintf(device->error, sizeof(device->error), "Invalid serial read arguments");
        return -1;
    }
#ifdef _WIN32
    {
        COMMTIMEOUTS timeouts = {0};
        DWORD got = 0, errors = 0;
        COMSTAT status;
        HANDLE handle = (HANDLE)device->handle;
        if (!ClearCommError(handle, &errors, &status)) return io_error(device, "Cannot inspect serial port");
        /* The condition is already cleared. Lost bytes are not a transport
         * failure: count them and let the decoder resynchronize. */
        if (errors & (CE_OVERRUN | CE_RXOVER | CE_FRAME | CE_RXPARITY)) note_receive_error(device);
        timeouts.ReadIntervalTimeout = MAXDWORD;
        /* MAXDWORD/0/0 is nonblocking. With a positive constant, MAXDWORD
         * for BOTH interval and multiplier waits only for the first byte. */
        timeouts.ReadTotalTimeoutMultiplier = timeout_ms > 0 ? MAXDWORD : 0;
        timeouts.ReadTotalTimeoutConstant = (DWORD)timeout_ms;
        timeouts.WriteTotalTimeoutConstant = 1000;
        if (!SetCommTimeouts(handle, &timeouts) ||
            !ReadFile(handle, data, (DWORD)size, &got, NULL))
            return io_error(device, "Serial read failed");
        count = (int)got;
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
        count = (int)got;
    }
#endif
    device->bytes_received += (unsigned)count;
    return count;
}
