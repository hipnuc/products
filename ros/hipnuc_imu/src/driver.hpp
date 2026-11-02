// Device reading shared by the ROS 1 and ROS 2 nodes, independent of ROS.
// The driver never changes device settings: after finding the baud rate it
// sends two read-only queries (LOG VERSION, LOG USRCONFIG) and then listens.

#ifndef HIPNUC_IMU_DRIVER_HPP
#define HIPNUC_IMU_DRIVER_HPP

#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "hipnuc_sample.h"

namespace hipnuc_imu {

using Clock = std::chrono::steady_clock;

struct Settings {
    std::string port;
    int baudrate = 115200;  // tried first; the others follow
};

// A Status at info level is OK.
enum class Level { info, warn, error };

struct Status {
    Level level = Level::error;
    std::string message;
    std::vector<std::pair<std::string, std::string>> values;
};

// A measurement's time is when the device began sending its frame.
class Output {
public:
    virtual ~Output() = default;
    virtual void log(Level level, const std::string &text) = 0;
    // Every decoded frame. imu is true when it carries acceleration and
    // angular velocity; the times of those frames always increase.
    virtual void measurement(const hipnuc_sample_t &sample, Clock::time_point time, bool imu) = 0;
    // Once per second.
    virtual void status(const Status &status) = 0;
};

// Opens the port, reopens it after failures and returns when running()
// becomes false.
void run(const Settings &settings, Output &output, const std::function<bool()> &running);

}  // namespace hipnuc_imu

#endif  // HIPNUC_IMU_DRIVER_HPP
