/* Linux SocketCAN: configure the interface with ip link, then run. */
#define _POSIX_C_SOURCE 200809L
#include "hipnuc_j1939.h"
#include <errno.h>
#include <inttypes.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static const char *INTERFACE = "can0";
static const int NODE_ID = -1; /* -1 accepts all sources; otherwise 0..255. */
static const int TIMEOUT_MS = 2000;
static volatile sig_atomic_t stopped;
static void stop(int number) { (void)number; stopped = 1; }

static uint64_t now_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
}

int main(void)
{
    struct sockaddr_can address = {0};
    int enabled = 1, result = 0, waiting = 0;
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) { perror("CAN socket"); return 1; }
    address.can_family = AF_CAN;
    address.can_ifindex = (int)if_nametoindex(INTERFACE);
    /* CAN FD reception also accepts Classic CAN. */
    if (!address.can_ifindex ||
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enabled, sizeof(enabled)) < 0 ||
        bind(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror(INTERFACE);
        close(fd);
        return 1;
    }
    signal(SIGINT, stop);
    fprintf(stderr, "Listening on %s. Ctrl-C stops.\n", INTERFACE);
    uint64_t last_sample = now_ms();
    while (!stopped) {
        struct pollfd input = {fd, POLLIN, 0};
        struct canfd_frame packet;
        hipnuc_can_frame_t frame = {0};
        hipnuc_sample_t sample;
        int ready = poll(&input, 1, 100);
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0 || (input.revents & (POLLERR | POLLHUP | POLLNVAL))) {
            fputs("CAN receive failed; check the interface.\n", stderr);
            result = 1;
            break;
        }
        if (!waiting && now_ms() - last_sample >= (uint64_t)TIMEOUT_MS) {
            fputs("No valid sample; check bitrate, source address and device output. Waiting...\n", stderr);
            waiting = 1;
        }
        if (!ready) continue;
        ssize_t size = read(fd, &packet, sizeof(packet));
        if (size < 0 && errno == EINTR) continue;
        if (size < 0) { perror("CAN receive"); result = 1; break; }
        if ((size != CAN_MTU && size != CANFD_MTU) || packet.len > (size == CAN_MTU ? 8 : 64))
            continue;
        frame.is_extended = (packet.can_id & CAN_EFF_FLAG) != 0;
        frame.is_remote = (packet.can_id & CAN_RTR_FLAG) != 0;
        frame.is_error = (packet.can_id & CAN_ERR_FLAG) != 0;
        frame.id = packet.can_id & (frame.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        frame.len = packet.len;
        memcpy(frame.data, packet.data, packet.len);
        if (hipnuc_j1939_parse(&frame, &sample, NULL) <= 0 ||
            (NODE_ID >= 0 && sample.node_id != NODE_ID)) continue;
        if (waiting) fputs("Data resumed.\n", stderr);
        waiting = 0;
        last_sample = now_ms();
        printf("node=%u pgn=0x%05X fields=0x%" PRIx64, (unsigned)sample.node_id,
               (unsigned)hipnuc_j1939_pgn(frame.id), sample.valid);
        if (sample.valid & HIPNUC_VALID_ACC)
            printf(" acc(m/s^2)=%.3f %.3f %.3f", sample.acc[0], sample.acc[1], sample.acc[2]);
        if (sample.valid & HIPNUC_VALID_GYR)
            printf(" gyr(rad/s)=%.3f %.3f %.3f", sample.gyr[0], sample.gyr[1], sample.gyr[2]);
        if (sample.valid & HIPNUC_VALID_ROLL_PITCH)
            printf(" roll/pitch(rad)=%.3f %.3f", sample.roll, sample.pitch);
        if (sample.valid & HIPNUC_VALID_YAW) printf(" yaw(rad)=%.3f", sample.yaw);
        putchar('\n');
        /* Process the other valid fields here; never reuse an older PGN. */
        if (fflush(stdout) != 0) { perror("Output failed"); result = 1; break; }
    }
    close(fd);
    return result ? result : stopped ? 130 : 0;
}
