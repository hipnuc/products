#include "serial_port.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>


static const struct {
    int rate;
    speed_t constant;
} baud_map[] = {
    {4800, B4800}, {9600, B9600}, {19200, B19200}, {38400, B38400},
    {57600, B57600}, {115200, B115200}, {230400, B230400},
    {0, B0}  // Sentinel
};

int serial_port_open(const char *portname)
{
    if (portname == NULL || strlen(portname) == 0) {
        fprintf(stderr, "Error: Invalid port name\n");
        return -1;
    }

    char full_name[MAX_PORT_NAME];
    if (strncmp(portname, "/dev/", 5) == 0) {
        // If it already contains "/dev/", use it directly
        strncpy(full_name, portname, sizeof(full_name) - 1);
    } else {
        // Otherwise, add "/dev/" prefix
        snprintf(full_name, sizeof(full_name), "/dev/%s", portname);
    }
    full_name[sizeof(full_name) - 1] = '\0';  // Ensure null-termination

    // Try to open the port
    int fd = open(full_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        // If opening fails, try again without "/dev/" prefix
        if (strncmp(full_name, "/dev/", 5) == 0) {
            fd = open(full_name + 5, O_RDWR | O_NOCTTY | O_NDELAY);
        }
    }

    if (fd == -1) {
        fprintf(stderr, "Error opening %s: %s\n", full_name, strerror(errno));
        return -1;
    }

    // Check if the port is a terminal device
    if (!isatty(fd)) {
        fprintf(stderr, "Error: %s is not a terminal device\n", full_name);
        close(fd);
        return -1;
    }

    return fd;
}

int serial_port_configure(int fd, int baud_rate)
{
    if (fd < 0) return -1;

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("Error getting port attributes");
        return -1;
    }

    // Set baud rate
    speed_t baud_constant = B0;
    for (int i = 0; baud_map[i].rate != 0; i++) {
        if (baud_map[i].rate == baud_rate) {
            baud_constant = baud_map[i].constant;
            break;
        }
    }

    if (baud_constant == B0) {
        fprintf(stderr, "Unsupported baud rate: %d\n", baud_rate);
        return -1;
    }

    if (cfsetispeed(&options, baud_constant) < 0 || 
        cfsetospeed(&options, baud_constant) < 0) {
        perror("Error setting baud rate");
        return -1;
    }

    // Configure other port settings
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 data bits
    options.c_cflag |= CLOCAL | CREAD;  // Enable receiver, ignore modem control lines

    // Disable hardware flow control
    options.c_cflag &= ~CRTSCTS;

    // Set input mode (non-canonical, no echo,...)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    options.c_iflag &= ~(INLCR | ICRNL);  // Disable newline & carriage return translation

    // Set output mode (raw output)
    options.c_oflag &= ~OPOST;

    // Set read timeout and minimum character count
    options.c_cc[VMIN] = 0;  // Minimum number of characters
    options.c_cc[VTIME] = 10;  // Timeout in deciseconds (1 second)

    // Apply the new settings
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("Error setting port attributes");
        return -1;
    }

    // Flush the buffer
    tcflush(fd, TCIOFLUSH);

    return 0;
}



// Send data and then receive response
int serial_send_then_recv(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms)
{
    if (fd < 0 || !send_str || !recv_buf || recv_buf_size == 0)
    {
        fprintf(stderr, "Invalid parameters\n");
        return -1;
    }

    // Set the port to non-blocking mode
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
    {
        fprintf(stderr, "Error getting file status flags: %s\n", strerror(errno));
        return -1;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        fprintf(stderr, "Error setting non-blocking mode: %s\n", strerror(errno));
        return -1;
    }

    // Send the string
    ssize_t bytes_written = write(fd, send_str, strlen(send_str));
    if (bytes_written < 0)
    {
        fprintf(stderr, "Error writing to serial port: %s\n", strerror(errno));
        fcntl(fd, F_SETFL, flags); // Restore original flags
        return -1;
    }

    // Clear input buffer
    tcflush(fd, TCIFLUSH);

    int total_bytes_read = 0;
    recv_buf[0] = '\0';
    int elapsed_time = 0;

    while (elapsed_time < timeout_ms)
    {
        ssize_t bytes_read = read(fd, recv_buf + total_bytes_read, recv_buf_size - total_bytes_read - 1);
        if (bytes_read > 0)
        {
            total_bytes_read += bytes_read;
            recv_buf[total_bytes_read] = '\0'; // Ensure null-termination
            if (expected && *expected && strstr(recv_buf, expected) != NULL)
            {
                fcntl(fd, F_SETFL, flags); // Restore original flags
                return total_bytes_read;
            }
        }
        else if (bytes_read < 0 && errno != EAGAIN)
        {
            fprintf(stderr, "Error reading from serial port: %s\n", strerror(errno));
            fcntl(fd, F_SETFL, flags); // Restore original flags
            return -1;
        }

        usleep(10000); // 10ms
        elapsed_time += 10;
    }

    // Timeout occurred
    fcntl(fd, F_SETFL, flags); // Restore original flags
    return total_bytes_read;
}

// Write data to the serial port
int serial_port_write(int fd, const void *buffer, int size)
{
    if (fd < 0 || !buffer || size <= 0)
    {
        fprintf(stderr, "Invalid parameters for write\n");
        return -1;
    }
    return write(fd, buffer, size);
}

// Read data from the serial port
int serial_port_read(int fd, void *buffer, int size)
{
    if (fd < 0 || !buffer || size <= 0)
    {
        fprintf(stderr, "Invalid parameters for read\n");
        return -1;
    }
    return read(fd, buffer, size);
}

// Close the serial port
void serial_port_close(int fd)
{
    if (fd >= 0)
    {
        close(fd);
    }
}

// Function to check if a device is likely a physical serial port
static bool is_physical_serial_port(const char *device_name) {
    return (strstr(device_name, "ttyUSB") != NULL) || 
           (strstr(device_name, "ttyACM") != NULL) ||
           (strstr(device_name, "ttyAMA") != NULL) ||
           (strstr(device_name, "ttyS") != NULL);  // Include ttyS for some physical ports
}

int list_serial_ports(PortInfo **ports) {
    DIR *dir;
    struct dirent *ent;
    char path[1024];
    struct stat st;
    int count = 0;

    // Allocate memory for the ports array
    *ports = malloc(sizeof(PortInfo) * MAX_PORTS);
    if (*ports == NULL) {
        return -1;  // Memory allocation failed
    }

    // Open the /dev directory
    dir = opendir("/dev");
    if (dir != NULL) {
        // Iterate through all entries in the /dev directory
        while ((ent = readdir(dir)) != NULL && count < MAX_PORTS) {
            // Check if the entry is likely a physical serial port
            if (is_physical_serial_port(ent->d_name)) {
                snprintf(path, sizeof(path), "/dev/%s", ent->d_name);
                // Check if the entry is a character device
                if (stat(path, &st) == 0 && S_ISCHR(st.st_mode)) {
                    // Add the port to our list
                    strncpy((*ports)[count].name, path, MAX_PORT_NAME - 1);
                    (*ports)[count].name[MAX_PORT_NAME - 1] = '\0';
                    count++;
                }
            }
        }
        closedir(dir);
    }

    return count;
}

void free_port_list(PortInfo *ports) {
    // Free the allocated memory for the ports array
    if (ports != NULL) {
        free(ports);
    }
}

void print_port_list(const PortInfo *ports, int count) {
    printf("Available physical serial ports:\n");
    for (int i = 0; i < count; i++) {
        printf("  %s\n", ports[i].name);
    }
}

