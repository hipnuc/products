// can_interface.c
#include "can_interface.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_arp.h>
#include <linux/can/raw.h>
#include <linux/net_tstamp.h>
#include <poll.h>

static void trim_newline(char *str)
{
    if (!str) {
        return;
    }
    for (char *p = str; *p; ++p) {
        if (*p == '\n' || *p == '\r') {
            *p = '\0';
            break;
        }
    }
}

static int read_sysfs_string(const char *path, char *buf, size_t len)
{
    if (!path || !buf || len == 0) {
        return -1;
    }

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
    return (strncmp(ifname, "vcan", 4) == 0) || (strncmp(ifname, "vxcan", 5) == 0);
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


// Enumerate physical CAN interfaces exposed by SocketCAN
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
        if (entry->d_name[0] == '.') {
            continue;
        }
        if (!is_real_can_interface(entry->d_name)) {
            continue;
        }

        can_interface_info_t *info = &interfaces[count];
        memset(info, 0, sizeof(*info));
        strncpy(info->name, entry->d_name, sizeof(info->name) - 1);
        strncpy(info->state, "unknown", sizeof(info->state) - 1);
        info->is_can_device = true;

        char state_path[256];
        int ret = snprintf(state_path, sizeof(state_path), "/sys/class/net/%s/operstate", entry->d_name);
        if (ret >= 0 && (size_t)ret < sizeof(state_path)) {
            char state_buf[16];
            if (read_sysfs_string(state_path, state_buf, sizeof(state_buf)) == 0) {
                strncpy(info->state, state_buf, sizeof(info->state) - 1);
            }
        }
        ++count;
    }

    closedir(dir);
    return count;
}

// Check if a given CAN interface is reported as up
int can_check_interface_status(const char *ifname)
{
    if (!ifname) {
        return -1;
    }
    
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/operstate", ifname);
    
    FILE *f = fopen(path, "r");
    if (!f) {
        log_error("Interface '%s' not found", ifname);
        return -1;
    }
    
    char state[16] = {0};
    fgets(state, sizeof(state), f);
    fclose(f);
    
    // Trim trailing newline
    char *newline = strchr(state, '\n');
    if (newline) *newline = '\0';
    
    // Treat 'unknown' as usable because some drivers never flip to 'up'
    if (strncmp(state, "up", 2) == 0 || strncmp(state, "unknown", 7) == 0) {
        return 1; // Interface ready
    }
    
    log_warn("Interface '%s' state is %s (not ready)", ifname, state);
    return 0; // Interface unavailable
}

// Open a CAN RAW socket bound to ifname
int can_open_socket(const char *ifname)
{
    int sockfd;
    struct ifreq ifr;
    struct sockaddr_can addr;
    
    if (!ifname) {
        log_error("Interface name cannot be empty");
        return -1;
    }
    
    // Create CAN RAW socket
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd < 0) {
        log_error("Failed to create CAN socket: %s", strerror(errno));
        return -1;
    }
    
    // Set a read timeout so signal handlers can break blocking calls
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000; // 500 ms
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        log_warn("Failed to set socket timeout: %s", strerror(errno));
        // continue, not fatal
    }

    int tstamp_flags = SOF_TIMESTAMPING_RAW_HARDWARE;
    if (setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING, &tstamp_flags, sizeof(tstamp_flags)) < 0) {
        log_warn("Failed to enable timestamping: %s", strerror(errno));
    }

    int rcvbuf = 4 * 1024 * 1024;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
        log_warn("Failed to set SO_RCVBUF: %s", strerror(errno));
    }
    
    // Resolve interface index
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) {
        log_error("Failed to query index for '%s': %s", ifname, strerror(errno));
        close(sockfd);
        return -1;
    }
    
    // Bind socket to the requested interface
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

// Close CAN socket helper
void can_close_socket(int sockfd)
{
    if (sockfd >= 0) {
        close(sockfd);
    }
}


int can_receive_frame_ts(int sockfd, struct can_frame *frame, uint64_t *hw_ts_us)
{
    if (sockfd < 0 || !frame || !hw_ts_us) {
        return -1;
    }

    struct iovec iov = {
        .iov_base = frame,
        .iov_len = sizeof(struct can_frame)
    };

    char ctrlmsg[256];
    struct msghdr msg = {
        .msg_iov = &iov,
        .msg_iovlen = 1,
        .msg_control = ctrlmsg,
        .msg_controllen = sizeof(ctrlmsg)
    };

    ssize_t nbytes = recvmsg(sockfd, &msg, 0);
    if (nbytes < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // Timeout or interrupted
        }
        log_error("recvmsg failed: %s", strerror(errno));
        return -1;
    }
    if ((size_t)nbytes < sizeof(struct can_frame)) {
        log_warn("Partial CAN frame received");
        return -1;
    }

    // Extract hardware timestamp (if available)
    *hw_ts_us = 0;
    for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPING) {
            struct timespec *ts = (struct timespec *)CMSG_DATA(cmsg);
            // Only use raw hardware timestamp (ts[2])
            if (ts[2].tv_sec || ts[2].tv_nsec) {
                *hw_ts_us = (uint64_t)ts[2].tv_sec * 1000000ULL 
                          + (uint64_t)ts[2].tv_nsec / 1000ULL;
            }
            break;
        }
    }

    return 1; // Success (hw_ts_us == 0 means no hardware timestamp available)
}

int can_receive_frames_ts(int sockfd,
                          struct can_frame *frames,
                          uint64_t *ts_us,
                          size_t max_frames,
                          int timeout_ms)
{
    if (sockfd < 0 || !frames || !ts_us || max_frames == 0) {
        return -1;
    }

    struct pollfd pfd = { .fd = sockfd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr == 0) {
        return 0; // timeout
    }
    if (pr < 0) {
        if (errno == EINTR) {
            return 0; // interrupted
        }
        log_error("poll failed: %s", strerror(errno));
        return -1;
    }

    size_t count = 0;
    while (count < max_frames) {
        struct iovec iov = {
            .iov_base = &frames[count],
            .iov_len = sizeof(struct can_frame)
        };

        char ctrlmsg[256];
        struct msghdr msg;
        memset(&msg, 0, sizeof(msg));
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrlmsg;
        msg.msg_controllen = sizeof(ctrlmsg);

        ssize_t nbytes = recvmsg(sockfd, &msg, MSG_DONTWAIT);
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break; // drained
            }
            if (errno == EINTR) {
                continue; // retry
            }
            log_error("recvmsg failed: %s", strerror(errno));
            return (int)count > 0 ? (int)count : -1;
        }
        if ((size_t)nbytes < sizeof(struct can_frame)) {
            log_warn("Partial CAN frame received");
            continue;
        }

        uint64_t hw_ts_us = 0;
        for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg); cmsg; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPING) {
                struct timespec *ts = (struct timespec *)CMSG_DATA(cmsg);
                if (ts[2].tv_sec || ts[2].tv_nsec) {
                    hw_ts_us = (uint64_t)ts[2].tv_sec * 1000000ULL + (uint64_t)ts[2].tv_nsec / 1000ULL;
                }
                break;
            }
        }
        ts_us[count] = hw_ts_us; // 0 if not available
        count++;
    }

    return (int)count;
}


// Validates the requested interface is available before executing commands
int can_ensure_interface_ready(const char *ifname)
{
    if (!ifname) {
        log_error("Missing CAN interface (use -i)");
        return -1;
    }
    int status = can_check_interface_status(ifname);
    if (status <= 0) {
        log_error("CAN interface '%s' is not ready", ifname);
        return -1;
    }
    return 0;
}
