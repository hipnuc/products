#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>

int serial_port_open(const char *portname);
int serial_port_configure(int fd, int baud_rate);
int serial_send_then_recv(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms);
void serial_port_close(int fd);
int serial_port_write(int fd, char *buffer, int size);
int serial_port_read(int fd, const char *buffer, int size);

#endif // SERIAL_PORT_H