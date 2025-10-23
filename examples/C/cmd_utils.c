// cmd_utils.c
#include <time.h>
#include <errno.h>

int safe_sleep(unsigned long usec) {
    struct timespec ts;
    ts.tv_sec = usec / 1000000;
    ts.tv_nsec = (usec % 1000000) * 1000;

    while (nanosleep(&ts, &ts) == -1) {
        if (errno != EINTR) {
            return -1;
        }
    }
    return 0;
}