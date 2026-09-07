/* POSIX serial port helpers for hihost (termios). */
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stddef.h>
#include <stdint.h>

#define MAX_PORT_NAME 64
#define MAX_PORTS 32

typedef struct {
    char name[MAX_PORT_NAME];
} PortInfo;

/* Supported baud rates, terminated by 0. Shared with `probe`. */
extern const int serial_port_baud_rates[];

/* Open /dev/<name> (or an absolute path). Returns the fd or -1. */
int serial_port_open(const char *portname);

/* 8N1, raw, no flow control. Returns 0 or -1. */
int serial_port_configure(int fd, int baud_rate);

/* Write all bytes and drain. Returns bytes written or -1. */
int serial_port_write(int fd, const void *buffer, int size);

/* Read up to `size` bytes, waiting at most `timeout_ms` overall. Returns
 * bytes read (0 on timeout) or -1. */
int serial_port_read_timeout(int fd, void *buffer, int size, int timeout_ms);

/* Short (1 ms) poll used by the read loop. */
int serial_port_read(int fd, void *buffer, int size);

/* Flush input, send `send_str`, collect the reply for `timeout_ms`, NUL
 * terminate it. When `expected` is given, returns -1 unless the reply
 * contains it. Returns the reply length or -1. */
int serial_send_then_recv_str(int fd, const char *send_str, const char *expected,
                              char *recv_buf, size_t recv_buf_size, int timeout_ms);

void serial_port_close(int fd);

int list_serial_ports(PortInfo **ports);
void free_port_list(PortInfo *ports);
void print_port_list(const PortInfo *ports, int count);

#endif /* SERIAL_PORT_H */
