// cmd_utils.h
#ifndef CMD_UTILS_H
#define CMD_UTILS_H

// Sleep for specified microseconds, handling EINTR
int safe_sleep(unsigned long usec);

#endif // CMD_UTILS_H