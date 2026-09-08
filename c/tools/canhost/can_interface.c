#include "can_interface.h"
#include <dirent.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if_arp.h>

static int sysfs_line(const char *name, const char *field, char *text, size_t size)
{
    char path[256];
    snprintf(path, sizeof(path), "/sys/class/net/%s/%s", name, field);
    FILE *file = fopen(path, "r");
    if (!file) return -1;
    int result = fgets(text, (int)size, file) ? 0 : -1;
    fclose(file);
    text[strcspn(text, "\r\n")] = 0;
    return result;
}

int can_list_interfaces(can_interface_info_t *items, int maximum)
{
    DIR *directory = opendir("/sys/class/net");
    struct dirent *entry;
    int count = 0;
    if (!directory || !items || maximum <= 0) {
        if (directory) closedir(directory);
        return -1;
    }
    while ((entry = readdir(directory)) != NULL && count < maximum) {
        char type[16] = {0};
        if (entry->d_name[0] == '.' || strlen(entry->d_name) >= sizeof(items[count].name)) continue;
        if (sysfs_line(entry->d_name, "type", type, sizeof(type)) || atoi(type) != ARPHRD_CAN) continue;
        strcpy(items[count].name, entry->d_name);
        strcpy(items[count].state, "unknown");
        (void)sysfs_line(entry->d_name, "operstate", items[count].state, sizeof(items[count].state));
        ++count;
    }
    closedir(directory);
    return count;
}

int can_open_socket(const char *name)
{
    struct ifreq interface;
    struct sockaddr_can address;
    int enabled = 1;
    if (!name || !*name || strlen(name) >= IFNAMSIZ) { errno = EINVAL; return -1; }
    int fd = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);
    if (fd < 0) return -1;
    // Enabling CAN FD reception also accepts Classic CAN frames.
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)) < 0) goto failed;
    // A software timestamp uses the host realtime clock. Raw hardware clocks
    // are not interchangeable with Unix time without synchronization.
    (void)setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled));
    memset(&interface, 0, sizeof(interface));
    strcpy(interface.ifr_name, name);
    if (ioctl(fd, SIOCGIFINDEX, &interface) < 0) goto failed;
    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = interface.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) goto failed;
    return fd;
failed:
    {
        int error = errno;
        close(fd);
        errno = error;
        return -1;
    }
}

void can_close_socket(int fd)
{
    if (fd >= 0) close(fd);
}

int can_send_frame(int fd, const hipnuc_can_frame_t *frame)
{
    struct canfd_frame packet;
    if (fd < 0 || !frame || frame->len > 64 || frame->is_error ||
        frame->id > (frame->is_extended ? CAN_EFF_MASK : CAN_SFF_MASK)) { errno = EINVAL; return -1; }
    memset(&packet, 0, sizeof(packet));
    packet.can_id = frame->id | (frame->is_extended ? CAN_EFF_FLAG : 0) | (frame->is_remote ? CAN_RTR_FLAG : 0);
    packet.len = frame->len;
    memcpy(packet.data, frame->data, frame->len);
    size_t size = frame->len <= 8 ? CAN_MTU : CANFD_MTU;
    return write(fd, &packet, size) == (ssize_t)size ? 0 : -1;
}

static int receive_one(int fd, can_rx_frame_t *out)
{
    struct canfd_frame packet;
    struct iovec vector = {&packet, sizeof(packet)};
    union { struct cmsghdr alignment; char bytes[CMSG_SPACE(sizeof(struct timespec))]; } control;
    struct msghdr message;
    memset(&message, 0, sizeof(message));
    message.msg_iov = &vector;
    message.msg_iovlen = 1;
    message.msg_control = control.bytes;
    message.msg_controllen = sizeof(control.bytes);
    ssize_t size = recvmsg(fd, &message, MSG_DONTWAIT);
    if (size < 0) return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ? 0 : -1;
    if ((size != CAN_MTU && size != CANFD_MTU) || packet.len > (size == CAN_MTU ? 8 : 64)) {
        errno = EPROTO;
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->frame.is_extended = (packet.can_id & CAN_EFF_FLAG) != 0;
    out->frame.is_remote = (packet.can_id & CAN_RTR_FLAG) != 0;
    out->frame.is_error = (packet.can_id & CAN_ERR_FLAG) != 0;
    out->frame.id = packet.can_id & (out->frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out->frame.len = packet.len;
    memcpy(out->frame.data, packet.data, packet.len);
    struct timespec timestamp;
    clock_gettime(CLOCK_REALTIME, &timestamp);
    if (!(message.msg_flags & MSG_CTRUNC)) {
        for (struct cmsghdr *item = CMSG_FIRSTHDR(&message); item; item = CMSG_NXTHDR(&message, item)) {
            if (item->cmsg_level == SOL_SOCKET && item->cmsg_type == SCM_TIMESTAMPNS &&
                item->cmsg_len >= CMSG_LEN(sizeof(timestamp))) {
                memcpy(&timestamp, CMSG_DATA(item), sizeof(timestamp));
                break;
            }
        }
    }
    out->timestamp_us = (uint64_t)timestamp.tv_sec * 1000000 + (uint64_t)timestamp.tv_nsec / 1000;
    return 1;
}

int can_receive_frames(int fd, can_rx_frame_t *out, size_t maximum, int timeout_ms)
{
    struct pollfd descriptor = {fd, POLLIN, 0};
    if (fd < 0 || !out || !maximum || timeout_ms < 0) { errno = EINVAL; return -1; }
    int ready = poll(&descriptor, 1, timeout_ms);
    if (ready < 0) return errno == EINTR ? 0 : -1;
    if (!ready) return 0;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) { errno = EIO; return -1; }
    if (!(descriptor.revents & POLLIN)) return 0;
    size_t count = 0;
    while (count < maximum) {
        int result = receive_one(fd, &out[count]);
        if (result < 0) return -1;
        if (!result) break;
        ++count;
    }
    return (int)count;
}
