// utils.c
#include "utils.h"
#include <time.h>
#include <string.h>
#include <stdio.h>

// Millisecond timestamp based on CLOCK_MONOTONIC
uint32_t utils_get_timestamp_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// Convert a Linux CAN frame into the HiPNUC CAN helper struct
void utils_linux_can_to_hipnuc_can(const struct can_frame *linux_frame, hipnuc_can_frame_t *hipnuc_frame)
{
    hipnuc_frame->can_id = linux_frame->can_id;
    hipnuc_frame->can_dlc = linux_frame->can_dlc;
    memcpy(hipnuc_frame->data, linux_frame->data, 8);
}

// Print quick setup commands when a CAN interface cannot be opened
void utils_print_can_setup_hint(const char *ifname)
{
    const char *iface = (ifname && *ifname) ? ifname : "can0";
    fprintf(stderr,
            "Error: cannot open CAN interface '%s'\n",
            ifname ? ifname : "(null)");
    fprintf(stderr, "Try: sudo ip link set %s down\n", iface);
    fprintf(stderr, "     sudo ip link set %s type can bitrate 500000\n", iface);
    fprintf(stderr, "     sudo ip link set %s up\n", iface);
    fprintf(stderr, "     ip -details link show %s\n", iface);
    fprintf(stderr, "Tip: run 'canhost list' to see available interfaces\n");
}
