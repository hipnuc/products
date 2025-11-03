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

static bool read_uint32_from_file(const char *path, uint32_t *value)
{
    if (!value) {
        return false;
    }
    char buf[32];
    if (read_sysfs_string(path, buf, sizeof(buf)) < 0) {
        return false;
    }
    char *end = NULL;
    errno = 0;
    unsigned long val = strtoul(buf, &end, 0);
    if (errno != 0 || end == buf) {
        return false;
    }
    *value = (uint32_t)val;
    return true;
}

static uint32_t read_bitrate_for_interface(const char *ifname)
{
    static const char *paths[] = {
        "/sys/class/net/%s/can_bittiming/bitrate",
        "/sys/class/net/%s/can_bittiming/nominal_bitrate",
        "/sys/class/net/%s/can_ext_bittiming/bitrate"
    };
    char path[256];
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        int ret = snprintf(path, sizeof(path), paths[i], ifname);
        if (ret < 0 || (size_t)ret >= sizeof(path)) {
            continue;
        }
        uint32_t bitrate = 0;
        if (read_uint32_from_file(path, &bitrate)) {
            return bitrate;
        }
    }
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

static bool state_is_down(const char *state)
{
    return state && strncmp(state, "down", 4) == 0;
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

        if (!state_is_down(info->state)) {
            info->bitrate = read_bitrate_for_interface(entry->d_name);
        } else {
            info->bitrate = 0;
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

// Receive a single CAN frame with timeout awareness
int can_receive_frame(int sockfd, struct can_frame *frame)
{
    if (sockfd < 0 || !frame) {
        return -1;
    }
    
    ssize_t nbytes = read(sockfd, frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
        if (errno == EINTR) {
            return 0; // interrupted by signal
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0; // timeout
        }
        log_error("Failed to read CAN frame: %s", strerror(errno));
        return -1;
    }
    
    if ((size_t)nbytes < sizeof(struct can_frame)) {
        log_warn("Received partial CAN frame");
        return -1;
    }
    
    return 1; // Success
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
