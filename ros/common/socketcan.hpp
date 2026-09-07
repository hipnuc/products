// Minimal SocketCAN reader for the ROS nodes (classic CAN and CAN FD).

#ifndef HIPNUC_ROS_SOCKETCAN_HPP
#define HIPNUC_ROS_SOCKETCAN_HPP

#include <cerrno>
#include <cstring>
#include <string>

#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hipnuc_can_frame.h"

namespace hipnuc_ros {

class SocketCan {
public:
    SocketCan() : fd_(-1) {}
    ~SocketCan() { close(); }
    SocketCan(const SocketCan &) = delete;
    SocketCan &operator=(const SocketCan &) = delete;

    bool is_open() const { return fd_ >= 0; }

    std::string open(const std::string &interface)
    {
        close();
        fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
        if (fd_ < 0) return std::string("socket: ") + std::strerror(errno);
        int fd_frames = 1;
        ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_frames, sizeof(fd_frames));

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(fd_, SIOCGIFINDEX, &ifr) != 0) { std::string e = interface + ": " + std::strerror(errno); close(); return e; }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
            std::string e = std::string("bind: ") + std::strerror(errno);
            close();
            return e;
        }
        return "";
    }

    void close()
    {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    // 1 when a frame was read, 0 on timeout, -1 on error.
    int read(hipnuc_can_frame_t &out, int timeout_ms)
    {
        if (fd_ < 0) return -1;
        struct pollfd p;
        p.fd = fd_;
        p.events = POLLIN;
        int r = ::poll(&p, 1, timeout_ms);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;

        struct canfd_frame frame;
        ssize_t n = ::read(fd_, &frame, sizeof(frame));
        if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
        if (n != CAN_MTU && n != CANFD_MTU) return 0;

        std::memset(&out, 0, sizeof(out));
        out.is_extended = (frame.can_id & CAN_EFF_FLAG) ? 1 : 0;
        out.is_remote = (frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
        out.is_error = (frame.can_id & CAN_ERR_FLAG) ? 1 : 0;
        out.id = frame.can_id & (out.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        out.len = frame.len > 64 ? 64 : frame.len;
        std::memcpy(out.data, frame.data, out.len);
        return 1;
    }

private:
    int fd_;
};

}  // namespace hipnuc_ros

#endif  // HIPNUC_ROS_SOCKETCAN_HPP
