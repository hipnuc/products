#pragma once

#include <string>
#include <vector>
#include <memory>

#include <sys/mman.h>
#include <sys/resource.h>


class SerialPortImpl;

class SerialPortException : public std::runtime_error {
    public :
    explicit SerialPortException(const std::string& msg)
    : std::runtime_error(msg){}
};

class SerialPort 
{
    public:
    SerialPort(const std::string& port_name, int baud_rate);
    ~SerialPort();

    bool open();
    bool read(std::vector<uint8_t>& buffer);
    bool isOpen();
    bool write(const std::vector<uint8_t>& data);
    bool close();

    private:
    std::unique_ptr<SerialPortImpl> impl_;
};



