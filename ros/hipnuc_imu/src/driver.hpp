// Device reading shared by the ROS 1 and ROS 2 nodes, independent of ROS.
// The driver never changes device settings: after finding the baud rate it
// sends two read-only queries (LOG VERSION, LOG USRCONFIG) and then listens.

#ifndef HIPNUC_IMU_DRIVER_HPP
#define HIPNUC_IMU_DRIVER_HPP

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <time.h>

#include "hipnuc_data.h"

namespace hipnuc_imu {

// CLOCK_MONOTONIC_RAW: NTP slews the steady clock, up to 500 ppm through the
// kernel and far more under chrony, and would bend DeviceClock's line. A node
// carries a time onto its own clock by its age, a few milliseconds.
struct Clock {
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<Clock>;
    static constexpr bool is_steady = true;
    static time_point now() noexcept
    {
        timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        return time_point(duration(static_cast<rep>(ts.tv_sec) * 1000000000 + ts.tv_nsec));
    }
};

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

// A measurement's time is the device's own time of the frame (HI83
// device_time_us, else HI91 system_time_ms) carried onto Clock by DeviceClock;
// a frame without one gets the moment the device began sending it.
class Output {
public:
    virtual ~Output() = default;
    virtual void log(Level level, const std::string &text) = 0;
    // Every decoded frame. imu is true when it carries acceleration and
    // angular velocity; the times of those frames always increase.
    virtual void measurement(const hipnuc_data_t &sample, Clock::time_point time, bool imu) = 0;
    // Once per second.
    virtual void status(const Status &status) = 0;
};

// Two reads of Clock around one of the ROS clock take less than this unless
// the thread paused between them.
constexpr auto kClockRead = std::chrono::microseconds(50);

// A node stamps a measurement on its own clock from two clock reads, and a
// pause between them moves the stamp back. An Imu stamp (ns) that does not
// follow the last moves 1 us after it, unless it is 100 ms or more back: then
// the clock was set.
class ImuStampOrder {
public:
    int64_t operator()(int64_t ns)
    {
        if (ns <= last_ && last_ - ns < 100000000) ns = last_ + 1000;
        return last_ = ns;
    }

private:
    int64_t last_ = 0;
};

// Carries one device clock onto Clock. A frame arrives some time after the
// device stamped it, never before, and the quickest arrivals recur, so the
// lowest arrival-minus-device offsets lie on a line whose slope is the rate
// difference of the two clocks. The lowest offset of each second of device time
// is kept for kWindows seconds, the slope is fitted through them and the line
// is laid on the lowest. A frame's time is its device time on that line: it
// keeps the device's spacing, not the jitter of the adapter and the host, and
// it is never later than the frame's arrival. A second whose lowest offset lies
// far above the line had no quick arrival (the host held its frames back) and
// stays out of the fit. Device time that steps back (restart, wrap, a change
// of time base) or runs ahead of the line starts the lows over; the slope
// stays, the device's crystal being the same.
class DeviceClock {
public:
    Clock::time_point operator()(int64_t device_ns, Clock::time_point arrival);
    void reset() { *this = DeviceClock(); }
    // Three lows span two seconds at least: two may lie side by side.
    bool fitted() const { return count_ >= 3; }
    // The device clock's rate against Clock, ppm, once fitted.
    double ppm() const { return -slope_ * 1e6; }

private:
    struct Low {
        int64_t device = 0, offset = 0;  // ns
        bool held = false;               // far above the line: not fitted
    };
    static constexpr int kWindows = 16;
    int64_t line(int64_t device) const;
    void fit();

    Low lows_[kWindows], current_;
    int count_ = 0, next_ = 0;
    int64_t window_start_ = 0, last_device_ = 0;
    bool started_ = false;
    double slope_ = 0;  // offset change per ns of device time
};

// Opens the port, reopens it after failures and returns when running()
// becomes false.
void run(const Settings &settings, Output &output, const std::function<bool()> &running);

}  // namespace hipnuc_imu

#endif  // HIPNUC_IMU_DRIVER_HPP
