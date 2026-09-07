#include "can_interface.h"

#include <dirent.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>            /* before the linux/ headers: shared ifreq definitions */
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if_arp.h>
#include <linux/net_tstamp.h>

#include "config.h"
#include "log.h"

static void trim_newline(char *str)
{
    if (!str) {
        return;
    }
    str[strcspn(str, "\r\n")] = '\0';
}

static int read_sysfs_string(const char *path, char *buf, size_t len)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    if (!fgets(buf, (int)len, f)) {
        fclose(f);
        return -1;
    }
    fclose(f);
    trim_newline(buf);
    return 0;
}

static bool is_virtual_can(const char *ifname)
{
    return strncmp(ifname, "vcan", 4) == 0 || strncmp(ifname, "vxcan", 5) == 0;
}

static bool is_real_can_interface(const char *ifname)
{
    if (!ifname || !*ifname || is_virtual_can(ifname)) {
        return false;
    }
    if (strncmp(ifname, "slcan", 5) == 0) {
        return true;
    }

    char type_path[256];
    int ret = snprintf(type_path, sizeof(type_path), "/sys/class/net/%s/type", ifname);
    if (ret < 0 || (size_t)ret >= sizeof(type_path)) {
        return false;
    }
    char type_buf[16];
    if (read_sysfs_string(type_path, type_buf, sizeof(type_buf)) < 0) {
        return false;
    }
    return atoi(type_buf) == ARPHRD_CAN;
}

int can_list_interfaces(can_interface_info_t *interfaces, int max_count)
{
    if (!interfaces || max_count <= 0) {
        return -1;
    }

    DIR *dir = opendir("/sys/class/net");
    if (!dir) {
        log_error("Cannot access /sys/class/net");
        return -1;
    }

    int count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && count < max_count) {
        if (entry->d_name[0] == '.' || !is_real_can_interface(entry->d_name)) {
            continue;
        }

        can_interface_info_t *info = &interfaces[count];
        memset(info, 0, sizeof(*info));
        if (strlen(entry->d_name) >= sizeof(info->name)) {
            continue;
        }
        strcpy(info->name, entry->d_name);
        strcpy(info->state, "unknown");

        char state_path[256];
        int ret = snprintf(state_path, sizeof(state_path), "/sys/class/net/%s/operstate", entry->d_name);
        if (ret >= 0 && (size_t)ret < sizeof(state_path)) {
            char state_buf[16];
            if (read_sysfs_string(state_path, state_buf, sizeof(state_buf)) == 0) {
                snprintf(info->state, sizeof(info->state), "%s", state_buf);
            }
        }
        ++count;
    }
    closedir(dir);
    return count;
}

int can_check_interface_status(const char *ifname)
{
    if (!ifname) {
        return -1;
    }
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/operstate", ifname);

    char state[16] = {0};
    if (read_sysfs_string(path, state, sizeof(state)) < 0) {
        log_error("Interface '%s' not found", ifname);
        return -1;
    }
    /* Some drivers never report "up"; treat "unknown" as usable. */
    if (strcmp(state, "up") == 0 || strcmp(state, "unknown") == 0) {
        return 1;
    }
    log_warn("Interface '%s' state is %s (not ready)", ifname, state);
    return 0;
}

int can_open_socket(const char *ifname)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    if (!ifname || !*ifname) {
        log_error("Interface name cannot be empty");
        return -1;
    }

    int sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        log_error("Failed to create CAN socket: %s", strerror(errno));
        return -1;
    }

    /* Bounded blocking reads so Ctrl+C is noticed promptly. */
    struct timeval timeout = { 0, 500000 };
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        log_warn("Failed to set socket timeout: %s", strerror(errno));
    }

    int fd_enable = config_get_canfd_enable() ? 1 : 0;
    if (setsockopt(sockfd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &fd_enable, sizeof(fd_enable)) < 0) {
        log_warn("Failed to set CAN FD option: %s", strerror(errno));
    }

    int tstamp_flags = SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE |
                       SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
    if (setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING, &tstamp_flags, sizeof(tstamp_flags)) < 0) {
        log_warn("Failed to enable timestamping: %s", strerror(errno));
    }

    int rcvbuf = 4 * 1024 * 1024;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        log_warn("Failed to set SO_RCVBUF: %s", strerror(errno));
    }

    memset(&ifr, 0, sizeof(ifr));
    int n = snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", ifname);
    if (n < 0 || (size_t)n >= sizeof(ifr.ifr_name)) {
        log_error("Interface name too long: '%s'", ifname);
        close(sockfd);
        return -1;
    }
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        log_error("Failed to query index for '%s': %s", ifname, strerror(errno));
        close(sockfd);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("Failed to bind to '%s': %s", ifname, strerror(errno));
        close(sockfd);
        return -1;
    }

    log_info("Connected to CAN interface %s", ifname);
    return sockfd;
}

void can_close_socket(int sockfd)
{
    if (sockfd >= 0) {
        close(sockfd);
    }
}

static canid_t to_linux_id(const hipnuc_can_frame_t *frame)
{
    canid_t id = frame->is_extended ? ((frame->id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                                    : (frame->id & CAN_SFF_MASK);
    if (frame->is_remote) {
        id |= CAN_RTR_FLAG;
    }
    return id;
}

int can_send_frame(int sockfd, const hipnuc_can_frame_t *frame)
{
    if (sockfd < 0 || !frame || frame->len > 64) {
        return -1;
    }

    if (frame->len <= CAN_MAX_DLEN) {
        struct can_frame raw;
        memset(&raw, 0, sizeof(raw));
        raw.can_id = to_linux_id(frame);
        raw.can_dlc = frame->len;
        memcpy(raw.data, frame->data, frame->len);
        if (write(sockfd, &raw, sizeof(raw)) != (ssize_t)sizeof(raw)) {
            log_error("CAN send failed: %s", strerror(errno));
            return -1;
        }
        return 0;
    }

    struct canfd_frame rawfd;
    memset(&rawfd, 0, sizeof(rawfd));
    rawfd.can_id = to_linux_id(frame);
    rawfd.len = frame->len;
    if (config_get_canfd_brs()) {
        rawfd.flags |= CANFD_BRS;
    }
    memcpy(rawfd.data, frame->data, frame->len);
    if (write(sockfd, &rawfd, sizeof(rawfd)) != (ssize_t)sizeof(rawfd)) {
        log_error("CAN FD send failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

/* Fill `out` from a received buffer of nbytes (CAN_MTU or CANFD_MTU). */
static int from_linux(const struct canfd_frame *raw, size_t nbytes, uint64_t ts_us, can_rx_frame_t *out)
{
    hipnuc_can_frame_t *f = &out->frame;
    canid_t id = raw->can_id;
    uint8_t len;

    if (nbytes == CAN_MTU) {
        const struct can_frame *cf = (const struct can_frame *)raw;
        len = cf->can_dlc > CAN_MAX_DLEN ? CAN_MAX_DLEN : cf->can_dlc;
    } else if (nbytes == CANFD_MTU) {
        len = raw->len > CANFD_MAX_DLEN ? CANFD_MAX_DLEN : raw->len;
    } else {
        return -1;
    }

    memset(f, 0, sizeof(*f));
    f->is_extended = (id & CAN_EFF_FLAG) ? 1 : 0;
    f->is_remote = (id & CAN_RTR_FLAG) ? 1 : 0;
    f->is_error = (id & CAN_ERR_FLAG) ? 1 : 0;
    f->id = f->is_extended ? (id & CAN_EFF_MASK) : (id & CAN_SFF_MASK);
    f->len = len;
    memcpy(f->data, raw->data, len);
    out->timestamp_us = ts_us;
    return 0;
}

/* Hardware receive time when present, otherwise the software time. */
static uint64_t timestamp_from_cmsg(struct msghdr *msg)
{
    for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msg); cmsg; cmsg = CMSG_NXTHDR(msg, cmsg)) {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPING) {
            struct timespec ts[3];
            memcpy(ts, CMSG_DATA(cmsg), sizeof(ts));
            const struct timespec *t = (ts[2].tv_sec || ts[2].tv_nsec) ? &ts[2] : &ts[0];
            return (uint64_t)t->tv_sec * 1000000ULL + (uint64_t)t->tv_nsec / 1000ULL;
        }
    }
    return 0;
}

static int recv_one(int sockfd, int flags, can_rx_frame_t *out)
{
    struct canfd_frame raw;
    struct iovec iov = { .iov_base = &raw, .iov_len = sizeof(raw) };
    char ctrlmsg[256];
    struct msghdr msg;

    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = ctrlmsg;
    msg.msg_controllen = sizeof(ctrlmsg);

    ssize_t nbytes = recvmsg(sockfd, &msg, flags);
    if (nbytes < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        log_error("recvmsg failed: %s", strerror(errno));
        return -1;
    }
    if (from_linux(&raw, (size_t)nbytes, timestamp_from_cmsg(&msg), out) < 0) {
        log_warn("Partial CAN frame received");
        return -1;
    }
    return 1;
}

int can_receive_frame(int sockfd, can_rx_frame_t *out)
{
    if (sockfd < 0 || !out) {
        return -1;
    }
    return recv_one(sockfd, 0, out);
}

int can_receive_frames(int sockfd, can_rx_frame_t *out, size_t max_frames, int timeout_ms)
{
    if (sockfd < 0 || !out || max_frames == 0) {
        return -1;
    }

    struct pollfd pfd = { .fd = sockfd, .events = POLLIN, .revents = 0 };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr == 0) {
        return 0;
    }
    if (pr < 0) {
        if (errno == EINTR) {
            return 0;
        }
        log_error("poll failed: %s", strerror(errno));
        return -1;
    }

    size_t count = 0;
    while (count < max_frames) {
        int r = recv_one(sockfd, MSG_DONTWAIT, &out[count]);
        if (r == 0) {
            break;
        }
        if (r < 0) {
            return count > 0 ? (int)count : -1;
        }
        count++;
    }
    return (int)count;
}
