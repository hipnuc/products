// can_interface.h
#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include <linux/can.h>

// Metadata collected for each physical CAN interface
typedef struct {
    char name[16];           // Interface name (e.g. can0)
    char state[16];          // Up/down/unknown
    bool is_can_device;      // True when backed by a CAN controller
    uint32_t bitrate;        // Configured bitrate in bit/s (0 when unavailable)
} can_interface_info_t;

// API
int can_list_interfaces(can_interface_info_t *interfaces, int max_count);
int can_check_interface_status(const char *ifname);
int can_open_socket(const char *ifname);
void can_close_socket(int sockfd);
int can_receive_frame(int sockfd, struct can_frame *frame);

// Ensures the provided interface exists and is ready
int can_ensure_interface_ready(const char *ifname);

#endif // CAN_INTERFACE_H
