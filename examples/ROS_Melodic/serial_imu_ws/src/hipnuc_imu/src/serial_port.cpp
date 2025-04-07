#include "serial_port.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <sys/time.h>
#include <signal.h>
#include <stdio.h>
#include <ros/ros.h>


class SerialPortImpl {
    public:

    SerialPortImpl(const std::string& port_name, int baud_rate)
        : port_name_(port_name), baud_rate_(baud_rate),
         fd_(-1)
         { }

    ~SerialPortImpl()
    {
        if (fd_ >= 0)
            close();
    }

    bool open()
    {
         fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
            throw SerialPortException("Failed to open port: " + port_name_);
        
        configurePort();

        fds.fd = fd_;
        fds.events = POLLIN;

         return true;
    }

    void configurePort()
    {
        struct termios options;
        tcgetattr(fd_, &options);
        cfmakeraw(&options);
        speed_t baud = getBaudRate(baud_rate_);
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;

        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(INLCR | ICRNL);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        if (tcsetattr(fd_, TCSANOW, &options) != 0)
            throw SerialPortException("Failed to configure port");
    }

    bool isOpen() const
    {
        return fd_ >= 0;
    }


    bool read(std::vector<uint8_t>& buffer)
    {
        int ret = poll(&fds, 1, 10);
        if (ret > 0)
        {
            buffer.resize(82);
            ssize_t n = ::read(fd_, buffer.data(), buffer.size());
            if (n > 0)
            {
                buffer.resize(n);
                return true;
            }
        }
        return false;
    }

    bool write(const std::vector<uint8_t>& data)
    {
        if (fd_ < 0)
            return false;
        
        ssize_t written = ::write(fd_, data.data(), data.size());

        return written == static_cast<ssize_t>(data.size());
    }

    bool close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_= -1;
            return true;
        }

        return false;
    }

    private:
    speed_t getBaudRate(int baud)
    {
        switch(baud)
        {
            case 115200: return B115200;
            case 460800: return B460800;
            case 921600: return B921600;
            default: throw SerialPortException("Unsupported baud rate");
        }
    }

    std::string port_name_;
    int baud_rate_;
    int fd_;
    struct pollfd fds;
};


SerialPort::SerialPort(const std::string& port_name, int baud_rate)
    : impl_(std::make_unique<SerialPortImpl>(port_name, baud_rate)) {}

SerialPort::~SerialPort() = default;
bool SerialPort::open() 
{
    return impl_->open();
}

bool SerialPort::isOpen() 
{
    return impl_->isOpen();
}

bool SerialPort::read(std::vector<uint8_t>& buffer) 
{
    return impl_->read(buffer);
}

bool SerialPort::close() 
{
    return impl_->close();
}