// Minimal POSIX serial port for the ROS nodes: open with any baudrate
// (termios2 BOTHER for nonstandard rates), read with a timeout, write.

#ifndef HIPNUC_ROS_POSIX_SERIAL_HPP
#define HIPNUC_ROS_POSIX_SERIAL_HPP

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <asm/termbits.h>   // termios2, BOTHER (Linux)

namespace hipnuc_ros {

class PosixSerial {
public:
    PosixSerial() : fd_(-1) {}
    ~PosixSerial() { close(); }
    PosixSerial(const PosixSerial &) = delete;
    PosixSerial &operator=(const PosixSerial &) = delete;

    bool is_open() const { return fd_ >= 0; }

    // Returns an empty string on success, otherwise an error description.
    std::string open(const std::string &device, int baudrate)
    {
        close();
        fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) return device + ": " + std::strerror(errno);

        struct termios2 tio;
        if (ioctl(fd_, TCGETS2, &tio) != 0) { std::string e = std::strerror(errno); close(); return e; }
        tio.c_cflag &= ~(CBAUD | CSIZE | PARENB | CSTOPB | CRTSCTS);
        tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (ioctl(fd_, TCSETS2, &tio) != 0) { std::string e = std::strerror(errno); close(); return e; }
        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return "";
    }

    void close()
    {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    // Bytes read (0 on timeout), or -1 on a transport error.
    int read(uint8_t *buf, size_t cap, int timeout_ms)
    {
        if (fd_ < 0) return -1;
        struct pollfd p;
        p.fd = fd_;
        p.events = POLLIN;
        int r = ::poll(&p, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
        ssize_t n = ::read(fd_, buf, cap);
        if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
        return static_cast<int>(n);
    }

    bool write(const std::string &text)
    {
        if (fd_ < 0) return false;
        size_t done = 0;
        while (done < text.size()) {
            ssize_t n = ::write(fd_, text.data() + done, text.size() - done);
            if (n < 0) { if (errno == EAGAIN || errno == EINTR) continue; return false; }
            done += static_cast<size_t>(n);
        }
        return true;
    }

private:
    int fd_;
};

}  // namespace hipnuc_ros

#endif  // HIPNUC_ROS_POSIX_SERIAL_HPP
