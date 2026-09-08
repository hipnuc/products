/* Linux SocketCAN I/O. All functions leave configuration/bitrate to the user. */
#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H
#include <stddef.h>
#include <stdint.h>
#include "hipnuc_can_frame.h"
typedef struct { char name[16]; char state[16]; } can_interface_info_t;
typedef struct {
    hipnuc_can_frame_t frame;
    uint64_t timestamp_us;  /* host Unix time, kernel software receive time when available */
} can_rx_frame_t;
int can_list_interfaces(can_interface_info_t *items, int maximum);
int can_open_socket(const char *name);
void can_close_socket(int fd);
int can_send_frame(int fd, const hipnuc_can_frame_t *frame);
/* At most maximum frames, with one initial poll; 0 idle, -1 I/O failure. */
int can_receive_frames(int fd, can_rx_frame_t *out, size_t maximum, int timeout_ms);
#endif
