#include "serial_port.h"

bool SerialPortConfig::isValid() const
{
    if (port_name.empty() || baud_rate <= 0)
        return false;
    
    if (data_bits < 5 || data_bits >8)
        return false;
    
    if (stop_bits != 1 && stop_bits != 2)
        return false;
    
    return true;
}

class SerialPort::Impl 
{
public:
    Impl(const SerialPortConfig& config)
        : config_(config)
        , fd_(-1)
        , status_()
        , error_callback_()
        {
            if (!config.isValid())
                throw SerialPortException("Invalid serial port configuration");
        }
    
    ~Impl()
    {
        if (isOpen())
            close();
    }

    bool open()
    {
        if (isOpen())
            return true;

        fd_ = ::open(config_.port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd_ < 0)
        {
            reportError(std::error_code(errno, std::system_category()));
            return false;
        }

        if (!configurePort())
        {
            close();
            return false;
        }
        
        status_.is_open = true;
        return true;
    }

    bool close()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
            status_.is_open = false;
            return true;
        }
        return false;
    }

    bool isOpen() const
    {
        return fd_ >= 0 && status_.is_open;
    }

    size_t read(std::vector<uint8_t>& buffer)
    {
        if (!isOpen())
            throw SerialPortException("Port not open");

        struct pollfd fds;
        fds.fd = fd_;
        fds.events = POLLIN;

        int poll_result = poll(&fds, 1, config_.read_timeout.count());

        if (poll_result <= 0)
        {
            reportError(std::error_code(errno, std::system_category()));
            return 0;
        }
 
        ssize_t bytes_read = ::read(fd_, buffer.data(), buffer.size());

        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return 0;
            }
            reportError(std::error_code(errno, std::system_category()));
            return 0;
        }

        status_.bytes_received += bytes_read;
        status_.last_read_time = std::chrono::steady_clock::now();
        return static_cast<size_t>(bytes_read);
    }

    size_t write(const std::vector<uint8_t>& data)
    {
        if (!isOpen())
            throw SerialPortException("Port not open");
        
        size_t total_written = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (total_written < data.size())
        {
            ssize_t written = ::write(fd_,
            data.data() + total_written,
            data.size() - total_written);

            if (written < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    auto elapsed = std::chrono::steady_clock::now() - start_time;
                    if (elapsed > config_.write_timeout)
                        break;
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                reportError(std::error_code(errno, std::system_category()));
                break;
            }

            total_written += written;
        }

        if (total_written > 0)
        {
            status_.bytes_sent += total_written;
            status_.last_write_time = std::chrono::steady_clock::now();
        }

        return total_written;
    }

    void flush()
    {
        if (!isOpen())
            return ;
        
        ::tcflush(fd_, TCIOFLUSH);
    }

    SerialPortStatus getStatus() const
    {
        return status_;
    }

    void setErrorCallback(ErrorCallback callback)
    {
        error_callback_ = std::move(callback);
    }

    bool updateConfig(const SerialPortConfig& new_config)
    {
        if (!new_config.isValid())
            return false;
        
        bool was_open = isOpen();
        if (was_open)
            close();

        config_ = new_config;

        if (was_open)
            return open();
        
        return true;
    }

    const SerialPortConfig& getConfig() const
    {
        return config_;
    }

private:
    bool configurePort()
    {
        struct termios options;
        if (tcgetattr(fd_, &options) != 0)
        {
            reportError(std::error_code(errno, std::system_category()));
            return false;
        }

        speed_t baud = converBaudRate(config_.baud_rate);
   
        if (cfsetispeed(&options, baud) != 0 || cfsetospeed(&options, baud) != 0)
        {
            reportError(std::error_code(errno, std::system_category()));
            return false;
        }

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;

        switch (config_.data_bits)
        {
            case 5: options.c_cflag |= CS5; break;
            case 6: options.c_cflag |= CS6; break;
            case 7: options.c_cflag |= CS7; break;
            case 8: options.c_cflag |= CS8; break;
            default: throw SerialPortException("config data bits error");
        }

        if (config_.stop_bits == 2) {
            options.c_cflag |= CSTOPB;
        } else {
            options.c_cflag &= ~CSTOPB;
        }

        switch (config_.Parity) {
            case SerialPortConfig::Parity::None:
                options.c_cflag &= ~PARENB;
                break;
            case SerialPortConfig::Parity::Even:
                options.c_cflag |= PARENB;
                options.c_cflag &= ~PARODD;
                break;
            case SerialPortConfig::Parity::Odd:
                options.c_cflag |= PARENB;
                options.c_cflag |= PARODD;
                break;
        }

        if (config_.hardware_flow_control) {
            options.c_cflag |= CRTSCTS;
        } else {
            options.c_cflag &= ~CRTSCTS;
        }

        if (config_.software_flow_control) {
            options.c_iflag |= (IXON | IXOFF | IXANY);
        } else {
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
        }

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            reportError(std::error_code(errno, std::system_category()));
            return false;
        }

        return true;
    }

    static speed_t converBaudRate(int baud_rate)
    {
        switch (baud_rate) 
        {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default: throw SerialPortException("Unsupported baud rate");
        }
    }

    void reportError(const std::error_code& ec) 
    {
        if (error_callback_) 
        {
            error_callback_(ec);
        }
    }

    SerialPortConfig config_;
    int fd_;
    SerialPortStatus status_;
    ErrorCallback error_callback_;
};

SerialPort::SerialPort(const SerialPortConfig& config)
    : pimpl_(std::make_unique<Impl>(config))
{}

SerialPort::SerialPort(SerialPort&&) noexcept = default;
SerialPort& SerialPort::operator=(SerialPort&&) noexcept = default;
SerialPort::~SerialPort() = default;

bool SerialPort::open() {return pimpl_->open(); }
bool SerialPort::close() { return pimpl_->close(); }
bool SerialPort::isOpen() const { return pimpl_->isOpen(); }
size_t SerialPort::read(std::vector<uint8_t>& buffer) { return pimpl_->read(buffer); }
size_t SerialPort::write(const std::vector<uint8_t>& data) { return pimpl_->write(data); }
void SerialPort::flush() { pimpl_->flush(); }
SerialPortStatus SerialPort::getStatus() const { return pimpl_->getStatus(); }
void SerialPort::setErrorCallback(ErrorCallback callback) { pimpl_->setErrorCallback(std::move(callback)); }
bool SerialPort::updateConfig(const SerialPortConfig& new_config) { return pimpl_->updateConfig(new_config); }
const SerialPortConfig& SerialPort::getConfig() const { return pimpl_->getConfig(); }

