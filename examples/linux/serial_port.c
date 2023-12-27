#include "serial_port.h"

static int is_valid_baud(int baud_rate)
{
    int valid_rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 921600};
    int num_rates = sizeof(valid_rates) / sizeof(valid_rates[0]);

    for (int i = 0; i < num_rates; i++)
    {
        if (baud_rate == valid_rates[i])
        {
            return 1;
        }
    }
    return 0;
}

int serial_port_open(const char *portname)
{
    char fullPortname[64];

    // 检查是否包含"/dev/"前缀，如果没有，则添加
    if (strncmp(portname, "/dev/", 5) == 0)
    {
        strncpy(fullPortname, portname, sizeof(fullPortname));
    }
    else
    {
        snprintf(fullPortname, sizeof(fullPortname), "/dev/%s", portname);
    }

    int fd = open(fullPortname, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", fullPortname, strerror(errno));
    }
    return fd;
}

int serial_port_configure(int fd, int baud_rate)
{
    struct termios options;

    tcgetattr(fd, &options);

    if (fd == -1)
    {
        perror("open_port: Unable to open SerialPort");
        puts("Please check the usb port name!!!");
        puts("such as \" sudo ./main ttyUSB0 \"");
        exit(0);
    }

    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed\n");
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("isatty success!\n");
    }

    bzero(&options, sizeof(options));

    options.c_cflag = CS8 | CLOCAL | CREAD;

    switch (baud_rate)
    {
    case 9600:
        options.c_cflag |= B9600;
        break;
    case 115200:
        options.c_cflag |= B115200;
        break;
    case 230400:
        options.c_cflag |= B230400;
        break;
    case 460800:
        options.c_cflag |= B460800;
        break;
    case 921600:
        options.c_cflag |= B921600;
        break;
    default:
        printf("port baud input error!\r\n");
        exit(0);
    }

    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return (fd);
}

int serial_send_then_recv(int fd, const char *send_str, const char *expected, char *recv_buf, size_t recv_buf_size, int timeout_ms)
{
    // 设置串口为非阻塞模式
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // 发送字符串
    ssize_t bytes_written = write(fd, send_str, strlen(send_str));
    if (bytes_written < 0)
    {
        fprintf(stderr, "Error writing to serial port: %s\n", strerror(errno));
        return -1;
    }

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
            if (strlen(expected) > 0 && strstr(recv_buf, expected) != NULL)
            {
                // 恢复串口为阻塞模式
                fcntl(fd, F_SETFL, flags);
                return total_bytes_read;
            }
        }

        usleep(10000); // 10ms
        elapsed_time += 10;
    }

    // 超时，恢复串口为阻塞模式
    fcntl(fd, F_SETFL, flags);
    return total_bytes_read;
}

int serial_port_write(int fd, char *buffer, int size)
{
    return read(fd, buffer, size);
}

int serial_port_read(int fd, const char *buffer, int size)
{
    return write(fd, buffer, size);
}

void serial_port_close(int fd)
{
    close(fd);
}
