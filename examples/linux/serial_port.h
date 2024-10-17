#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/time.h>

#define MAX_PORT_NAME 64
#define MAX_PORTS 32

// Structure to hold information about a serial port
typedef struct {
    char name[MAX_PORT_NAME];
} PortInfo;


// Open the serial port
int serial_port_open(const char *portname);

// Configure the serial port
int serial_port_configure(int fd, int baud_rate);

// Send data and then receive response
int serial_send_then_recv(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms);

// Write data to the serial port
int serial_port_write(int fd, const void *buffer, int size);

// Read data from the serial port
int serial_port_read(int fd, void *buffer, int size);

// Close the serial port
void serial_port_close(int fd);

int list_serial_ports(PortInfo **ports);
void free_port_list(PortInfo *ports);
void print_port_list(const PortInfo *ports, int count);


#endif // SERIAL_PORT_H
