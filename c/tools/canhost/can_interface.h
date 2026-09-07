/* SocketCAN access for canhost: interface discovery, RAW socket, frame
 * conversion between Linux can_frame/canfd_frame and hipnuc_can_frame_t. */
#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hipnuc_can_frame.h"

typedef struct {
    char name[16];           /* e.g. can0 */
    char state[16];          /* up / down / unknown */
} can_interface_info_t;

/* A received frame with the kernel or hardware receive time (us since the
 * epoch, 0 when unavailable). The SDK frame carries no timestamp. */
typedef struct {
    hipnuc_can_frame_t frame;
    uint64_t timestamp_us;
} can_rx_frame_t;

/* Physical CAN interfaces under /sys/class/net (vcan excluded). */
int can_list_interfaces(can_interface_info_t *interfaces, int max_count);

/* 1 when the interface is up (or reports "unknown"), 0 when down, -1 when
 * it does not exist. */
int can_check_interface_status(const char *ifname);

/* Open a RAW socket bound to ifname (CAN FD enabled per config). */
int can_open_socket(const char *ifname);
void can_close_socket(int sockfd);

/* Send one frame. Returns 0 or -1. */
int can_send_frame(int sockfd, const hipnuc_can_frame_t *frame);

/* Receive one frame, waiting up to the socket timeout (500 ms). Returns 1
 * with the frame, 0 on timeout or interrupt, -1 on error. */
int can_receive_frame(int sockfd, can_rx_frame_t *out);

/* Receive up to max_frames without blocking after the first poll of
 * timeout_ms. Returns the number of frames, 0 on timeout, -1 on error. */
int can_receive_frames(int sockfd, can_rx_frame_t *out, size_t max_frames, int timeout_ms);

#endif /* CAN_INTERFACE_H */
