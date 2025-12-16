// cmd_utils.h
#ifndef CMD_UTILS_H
#define CMD_UTILS_H

// Sleep for specified microseconds, handling EINTR
int safe_sleep(unsigned long usec);

// Load serial config from INI file
// Returns 0 on success; -1 on error or missing keys
int ini_load_serial(const char *path, char *port_out, size_t port_out_sz, int *baud_out);

// Update/Create INI file with serial config (overwrite minimal keys)
// Returns 0 on success; -1 on failure
int ini_update_serial(const char *path, const char *port, int baud);


#endif // CMD_UTILS_H