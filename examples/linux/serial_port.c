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
    {57600, B57600}, {115200, B115200}, {230400, B230400}, {460800, B460800}, {921600, B921600},
    {0, B0}  // Sentinel
};


/**
 * Opens a serial port device and returns a file descriptor for it.
 *
 * @param portname The name of the serial port device to open, e.g. "ttyUSB0".
 * @return The file descriptor for the opened serial port, or -1 on error.
 */
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


/**
 * Configures the serial port with the specified baud rate.
 *
 * @param fd The file descriptor of the open serial port.
 * @param baud_rate The baud rate to set for the serial port.
 * @return 0 on success, -1 on error.
 */
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

// Write data to the serial port
int serial_port_write(int fd, const void *buffer, int size)
{
    if (fd < 0 || !buffer || size <= 0)
    {
        return -1;
    }

    int bytes_written = write(fd, buffer, size);
    if (bytes_written < 0)
    {
        return -1;
    }

    if (tcdrain(fd) < 0)
    {
        return -1;
    }

    return bytes_written;
}

/**
 * Read data from serial port with idle timeout mechanism
 *
 * @param fd          File descriptor of the serial port
 * @param buffer      Buffer to store received data
 * @param size        Maximum number of bytes to read
 * @param timeout_ms  Idle timeout in milliseconds (timeout between bytes)
 *
 * @return Number of bytes read, or -1 on error
 * @note The timeout is reset each time new data arrives
 */
static int serial_port_read_timeout(int fd, void *buffer, int size, int timeout_ms)
{
    if (fd < 0 || !buffer || size <= 0 || timeout_ms < 0)
    {
        fprintf(stderr, "Invalid parameters for read with timeout\n");
        return -1;
    }

    uint8_t *buf = (uint8_t *)buffer;
    int total_read = 0;
    int ret;

    // Setup for select() timeout
    struct timeval tv;
    fd_set readfds;

    while (total_read < size)
    {
        // Reset select() parameters for each iteration
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        
        // Configure timeout for this iteration
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        // Wait for data or timeout
        ret = select(fd + 1, &readfds, NULL, NULL, &tv);

        if (ret < 0)
        {
            // Handle interruption by signal
            if (errno == EINTR)
                continue;
            perror("select");
            return -1;
        }
        else if (ret == 0)
        {
            // No data received within timeout period
            // This means the line has been idle for timeout_ms
            break;
        }
        
        // Data is available, read it
        ret = read(fd, buf + total_read, size - total_read);
        if (ret < 0)
        {
            // Handle non-blocking operations
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            perror("read");
            return -1;
        }
        else if (ret == 0)
        {
            // Port closed or disconnected
            break;
        }

        // Update total bytes read
        total_read += ret;
    }

    return total_read;
}

// Read data from the serial port
int serial_port_read(int fd, void *buffer, int size)
{
    return serial_port_read_timeout(fd, buffer, size, 1);
}

/**
 * Send data and wait for response with idle timeout
 *
 * @param fd              File descriptor of the serial port
 * @param tx_data        Data to transmit
 * @param tx_len         Length of data to transmit
 * @param rx_data        Buffer to store received data
 * @param expected_rx_len Expected number of bytes to receive
 * @param timeout_ms     Idle timeout in milliseconds
 *
 * @return Number of bytes received, or -1 on error
 * @note The timeout is reset whenever new data arrives
 */
int serial_send_then_recv(int fd, const uint8_t *tx_data, int tx_len, 
                         uint8_t *rx_data, int expected_rx_len, int timeout_ms) 
{
    if (fd < 0 || !tx_data || !rx_data || tx_len <= 0 || expected_rx_len <= 0)
    {
        fprintf(stderr, "Invalid parameters for send_then_recv\n");
        return -1;
    }
    
    // Clear any pending input data
    tcflush(fd, TCIFLUSH);

    // Send the data
    if (serial_port_write(fd, tx_data, tx_len) != tx_len) {
        fprintf(stderr, "Failed to send data\n");
        return -1;
    }

    // Read response with idle timeout mechanism
    return serial_port_read_timeout(fd, rx_data, expected_rx_len, timeout_ms);
}

int serial_send_then_recv_str(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms)
{
    if (fd < 0 || !send_str || !recv_buf || recv_buf_size == 0)
    {
        fprintf(stderr, "Invalid parameters\n");
        return -1;
    }

    int send_len = strlen(send_str);
    int total_bytes_read = serial_send_then_recv(fd, (const uint8_t *)send_str, send_len, (uint8_t *)recv_buf, recv_buf_size - 1, timeout_ms);

    if (total_bytes_read < 0)
    {
        return -1;
    }

    // Ensure null-termination
    recv_buf[total_bytes_read] = '\0';

    // Check for expected response if provided
    if (expected && *expected)
    {
        if (strstr(recv_buf, expected) != NULL)
        {
            return total_bytes_read;
        }
        else
        {
            // Expected response not found
            return -1;
        }
    }

    return total_bytes_read;
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

