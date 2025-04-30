#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <functional>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <system_error>
#include <thread>


class SerialPortException : public std::runtime_error {
    public :
    explicit SerialPortException(const std::string& msg)
    : std::runtime_error(msg){}
};

struct SerialPortConfig 
{
    std::string port_name;
    int baud_rate;
    int data_bits{8};
    int stop_bits{1};
    enum class Parity {None, Odd, Even } Parity{Parity::None};
    bool hardware_flow_control{false};
    bool software_flow_control{false};
    std::chrono::milliseconds read_timeout{100};
    std::chrono::milliseconds write_timeout{100};

    bool isValid() const;
};

struct SerialPortStatus
{
    bool is_open{false};
    size_t bytes_received{0};
    size_t bytes_sent{0};
    std::chrono::steady_clock::time_point last_read_time;
    std::chrono::steady_clock::time_point last_write_time;
};

class SerialPort
{
    public:
    explicit SerialPort(const SerialPortConfig& config);

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator = (const SerialPort&) = delete;

    SerialPort(SerialPort&&) noexcept;
    SerialPort& operator = (SerialPort&&) noexcept;

    ~SerialPort() ;

    bool open(); 
    bool close();
    bool isOpen() const;
    size_t read(std::vector<uint8_t>& buffer);
    size_t write(const std::vector<uint8_t>& buffer); 
    void flush(); 
    SerialPortStatus getStatus() const ;

    using ErrorCallback = std::function<void(const std::error_code&)>;
    void setErrorCallback(ErrorCallback callback);

    bool updateConfig(const SerialPortConfig& new_config);
    const SerialPortConfig& getConfig() const;

    private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

