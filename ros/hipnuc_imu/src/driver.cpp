#include "driver.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <thread>
#include <vector>

#include <unistd.h>

#include "convert.hpp"
#include "hipnuc_serial.h"

namespace hipnuc_imu {
namespace {

constexpr auto kRetry = std::chrono::seconds(1);
constexpr auto kRetryLog = std::chrono::seconds(5);
constexpr auto kReport = std::chrono::seconds(1);
constexpr auto kSilent = std::chrono::seconds(2);
constexpr auto kHint = std::chrono::seconds(3);
constexpr int kPollMs = 50;
constexpr int kStaleMs = 100;  // input right after opening
constexpr int kWaitMs = 500;   // for frames at a candidate baud rate, or for a reply
// Device baud rates in the order hihost scans them.
constexpr int kBaudrates[] = {115200, 921600, 460800, 230400, 256000, 57600, 38400, 19200, 9600, 4800};
// SLAM front ends commonly want 200 Hz; 2D navigation runs fine on less.
constexpr double kSlamRate = 200.0;

double seconds(Clock::duration duration) { return std::chrono::duration<double>(duration).count(); }

std::string text(double value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.1f", value);
    return buffer;
}

void add_device_status(Status &status, const hipnuc_data_t &sample)
{
    if (!(sample.valid & HIPNUC_VALID_STATUS)) return;
    auto flag = [&](const char *key, bool value) { status.values.emplace_back(key, value ? "true" : "false"); };
    char word[8];
    std::snprintf(word, sizeof(word), "0x%04X", sample.main_status);
    status.values.emplace_back("main_status", word);
    flag("attitude_converged", sample.attitude_converged);
    flag("gyro_bias_converged", sample.gyro_bias_converged);
    flag("static", sample.device_static);
    flag("magnetic_disturbance", sample.magnetic_disturbance);
    flag("accelerometer_saturated", sample.main_status & HIPNUC_STATUS_ACC_SAT);
    flag("gyroscope_saturated", sample.main_status & HIPNUC_STATUS_GYR_SAT);
}

// Frame counting, the once-per-second status and its log line on each change.
class Report {
public:
    explicit Report(Output &output) : output_(output) {}

    void frame(const hipnuc_data_t &sample, Clock::time_point now)
    {
        ++frames_;
        last_frame_ = now;
        if (sample.valid & HIPNUC_VALID_STATUS) status_sample_ = sample;
    }

    // A new connection gets a full report period before it counts as silent.
    void opened()
    {
        last_frame_ = {};
        last_ = Clock::now();
        frames_at_last_ = frames_;
        status_sample_ = hipnuc_data_t{};
    }

    bool silent(Clock::time_point now) const
    {
        return last_frame_ == Clock::time_point{} || now - last_frame_ > kSilent;
    }

    bool due(Clock::time_point now) const { return now - last_ >= kReport; }

    void publish(Status status, Clock::time_point now)
    {
        const double elapsed = seconds(now - last_);
        status.values.insert(status.values.begin(),
                             {{"frames", std::to_string(frames_)},
                              {"frame_rate_hz", text((frames_ - frames_at_last_) / elapsed)}});
        add_device_status(status, status_sample_);
        // Open failures log themselves; report how receiving changes.
        if (status.level != Level::error && status.message != last_message_) output_.log(status.level, status.message);
        last_message_ = status.level == Level::error ? std::string() : status.message;
        output_.status(status);
        frames_at_last_ = frames_;
        last_ = now;
    }

private:
    Output &output_;
    uint64_t frames_ = 0, frames_at_last_ = 0;
    Clock::time_point last_ = Clock::now(), last_frame_{};
    hipnuc_data_t status_sample_{};
    std::string last_message_;
};

// SLAM front ends integrate stamp differences: LIO-SAM fails on dt <= 0 and
// FAST-LIO drops its buffer on a step back. Each Imu time follows the last.
class Increasing {
public:
    Clock::time_point operator()(Clock::time_point time)
    {
        if (time <= last_) {
            time = last_ + std::chrono::microseconds(1);
            ++adjusted_;
        }
        return last_ = time;
    }
    uint64_t adjusted() const { return adjusted_; }

private:
    Clock::time_point last_{};
    uint64_t adjusted_ = 0;
};

// Line time of 8N1 bytes at the configured rate.
Clock::duration line_time(size_t bytes, int baudrate)
{
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(bytes * 10.0 / baudrate));
}

std::string open_hint(const std::string &port)
{
    if (access(port.c_str(), F_OK) != 0) return " (check the cable and the port name: ls /dev/serial/by-id)";
    if (access(port.c_str(), R_OK | W_OK) != 0)
        return " (run: sudo usermod -aG dialout $USER, then log out and back in)";
    return "";
}

// At least kWaitMs, and three of the longest frames (150 bytes) at low baud rates.
int wait_ms(int baudrate) { return std::max(kWaitMs, 3 * 1500 * 1000 / baudrate); }

// True once two valid frames or NMEA sentences arrive: the baud rate is right.
// A CH347 can still hold a frame it received at the previous baud rate, so the
// first bytes after opening prove nothing.
bool frames_arrive(hipnuc_serial_t &serial, int baudrate)
{
    hipnuc_data_t sample;
    int valid = 0;
    const auto start = Clock::now();
    const auto stale = start + std::chrono::milliseconds(kStaleMs);
    const auto deadline = stale + std::chrono::milliseconds(wait_ms(baudrate));
    for (auto now = start; now < deadline; now = Clock::now()) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
        const int result = hipnuc_serial_read(&serial, &sample, static_cast<int>(left));
        if (result < 0) return false;
        if (result > 0 && Clock::now() >= stale && ++valid == 2) return true;
    }
    return false;
}

// Values of the given keys in the reply to a read-only query, from the lines
// that arrived even when OK or ERR did not. Frames keep streaming meanwhile
// and are dropped. complete turns false when the reply does not end with OK
// or ERR.
std::map<std::string, std::string> query(hipnuc_serial_t &serial, int baudrate, const char *command,
                                         std::initializer_list<const char *> keys, bool &complete)
{
    std::map<std::string, std::string> values;
    const int status = hipnuc_serial_command(&serial, command, wait_ms(baudrate));
    complete = complete && status >= 0;
    // Copied now: the reply buffer is reused by the next command.
    for (const char *key : keys)
        if (const char *value = hipnuc_serial_reply_value(&serial, key)) values[key] = value;
    return values;
}

struct Device {
    std::string model, firmware, axes;
    bool complete = false;  // both replies ended with OK or ERR
};

Device identify(hipnuc_serial_t &serial, int baudrate)
{
    Device device;
    device.complete = true;
    auto version = query(serial, baudrate, "LOG VERSION", {"PNAME", "BUILD", "APP_VER"}, device.complete);
    device.model = version["PNAME"];
    const std::string app = version["APP_VER"];  // 171 is 1.7.1
    if (app.size() == 3) device.firmware = std::string{app[0], '.', app[1], '.', app[2]};
    if (!version["BUILD"].empty()) device.firmware += " (" + version["BUILD"] + ")";
    // Not every device lists the saved axes setting, and COORD 4 does not turn
    // the vectors on every device: point the user at the check in the README.
    const std::string coord = query(serial, baudrate, "LOG USRCONFIG", {"COORD"}, device.complete)["COORD"];
    const std::string readme = "see \"Axes and TF\" in the README";
    if (coord == "0")
        device.axes = "device axes: x right, y forward, z up (COORD 0); " + readme;
    else if (coord == "4")
        device.axes = "device axes setting COORD 4 (x forward, y left, z up); "
                      "check them as described in \"Axes and TF\" in the README";
    else if (!coord.empty())
        device.axes = "device axes setting COORD " + coord + "; " + readme;
    else
        device.axes = "the device does not report its axes setting; " + readme;
    return device;
}

// Baud rate and output period that carry kSlamRate with 20% headroom.
std::string rate_hint(const std::string &port, int baudrate, double rate, const hipnuc_data_t &sample, size_t bytes)
{
    const char *frame = sample.source == HIPNUC_SOURCE_HI83 ? "HI83" : sample.source == HIPNUC_SOURCE_HI81 ? "HI81" : "HI91";
    const double needed = bytes * 10.0 * kSlamRate / 0.8;
    int target = baudrate;
    for (int candidate : {230400, 460800, 921600})
        if (baudrate < needed && candidate >= needed) {
            target = candidate;
            break;
        }
    std::string hint = "imu/data runs at " + text(rate) + " Hz; SLAM usually needs 200 Hz. To raise it, stop this node and run: ";
    if (target != baudrate) hint += "hihost baudrate " + std::to_string(target) + " -p " + port + " --save, then ";
    return hint + "hihost command \"LOG " + frame + " ONTIME 0.005\" -p " + port + " -b " + std::to_string(target) +
           " --save";
}

constexpr int64_t kWindowNs = 1000000000;  // one low per second of device time
constexpr int64_t kJumpNs = 10000000;      // device time this far ahead of the line has jumped
// Crystals differ by tens of ppm: a steeper fit is the burst of frames a host
// stall leaves in the buffer, not a clock.
constexpr double kSlopeMax = 1e-3;

int64_t nanoseconds(Clock::time_point time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

constexpr int64_t kMsNs = 1000000;
constexpr int64_t kDayNs = 86400000 * kMsNs;
constexpr int64_t kUtcEnterNs = 50 * kMsNs;   // the host clock keeps UTC this closely
constexpr int64_t kUtcLeaveNs = 100 * kMsNs;  // no longer
constexpr double kUtcSettleS = 10.0;

// The UTC of an HI83 or HI91 frame's own sample since the epoch in ns, or 0:
// HI83 carries the date, HI91 the time of day only, which takes the host's
// nearest day. An RMC's UTC is its fix, not its frame's time.
int64_t frame_utc(const hipnuc_data_t &s, int64_t host_utc)
{
    if (s.source == HIPNUC_SOURCE_HI83 && (s.valid & HIPNUC_VALID_UTC)) {
        std::tm t{};
        t.tm_year = s.utc.year - 1900;
        t.tm_mon = s.utc.month - 1;
        t.tm_mday = s.utc.day;
        t.tm_hour = s.utc.hour;
        t.tm_min = s.utc.minute;
        t.tm_sec = s.utc.second;
        return static_cast<int64_t>(timegm(&t)) * 1000000000 + s.utc.millisecond * kMsNs;
    }
    if (s.source != HIPNUC_SOURCE_HI91 || !(s.valid & HIPNUC_VALID_STATUS) || (s.main_status & HIPNUC_STATUS_UTC_UNSYNC))
        return 0;
    int64_t utc = host_utc / kDayNs * kDayNs + s.system_time_ms * kMsNs;
    if (utc - host_utc > kDayNs / 2) utc -= kDayNs;
    if (host_utc - utc > kDayNs / 2) utc += kDayNs;
    return utc;
}

// A device synchronized to UTC (RMC and PPS) shares its time with GNSS-timed
// sensors such as a lidar, a host clock only as closely as its own sync. Once
// the host clock agrees with the device's UTC within kUtcEnterNs, frame times
// move onto that UTC; when the device loses it, or the two clocks part by
// kUtcLeaveNs, they move back. Either move approaches the other clock with time
// constant kUtcSettleS instead of stepping, and the UTC fields, counting whole
// milliseconds, are averaged.
class Utc {
public:
    Clock::time_point operator()(const hipnuc_data_t &sample, Clock::time_point time)
    {
        const int64_t host_utc = nanoseconds_utc(), steady = nanoseconds(Clock::now());
        const int64_t utc = frame_utc(sample, host_utc);
        if ((sample.valid & HIPNUC_VALID_STATUS) && (sample.main_status & HIPNUC_STATUS_UTC_UNSYNC)) on_ = false;
        if (utc) {
            // The frame's UTC on Clock, half a millisecond after the whole one.
            gap_ = utc + kMsNs / 2 - (host_utc - steady) - nanoseconds(time);
            if (!on_ && std::llabs(gap_) < kUtcEnterNs) on_ = true;
            else if (on_ && std::llabs(gap_) > kUtcLeaveNs) on_ = false;
        }
        const double share =
            last_ == Clock::time_point{} ? 0 : std::max(0.0, std::min(1.0, seconds(time - last_) / kUtcSettleS));
        last_ = time;
        offset_ += (static_cast<double>(on_ ? gap_ : 0) - offset_) * share;
        return time + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double, std::nano>(offset_));
    }
    bool on() const { return on_; }

private:
    static int64_t nanoseconds_utc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
            .count();
    }
    bool on_ = false;
    int64_t gap_ = 0;    // ns
    double offset_ = 0;  // ns
    Clock::time_point last_{};
};

}  // namespace

int64_t DeviceClock::line(int64_t device) const
{
    auto through = [&](const Low &low) {
        return low.offset + static_cast<int64_t>(slope_ * static_cast<double>(device - low.device));
    };
    int64_t lowest = through(current_);
    for (int i = 0; i < count_; ++i) lowest = std::min(lowest, through(lows_[i]));
    return lowest;
}

void DeviceClock::fit()
{
    const Low *origin = nullptr;
    int used = 0;
    double mean_x = 0, mean_y = 0;
    for (int i = 0; i < count_; ++i) {
        if (lows_[i].held) continue;
        if (!origin) origin = &lows_[i];
        mean_x += static_cast<double>(lows_[i].device - origin->device);
        mean_y += static_cast<double>(lows_[i].offset - origin->offset);
        ++used;
    }
    if (used < 3) return;
    mean_x /= used;
    mean_y /= used;
    double sxy = 0, sxx = 0;
    for (int i = 0; i < count_; ++i) {
        if (lows_[i].held) continue;
        const double dx = static_cast<double>(lows_[i].device - origin->device) - mean_x;
        sxy += dx * (static_cast<double>(lows_[i].offset - origin->offset) - mean_y);
        sxx += dx * dx;
    }
    slope_ = sxx > 0 ? std::max(-kSlopeMax, std::min(kSlopeMax, sxy / sxx)) : 0;
}

Clock::time_point DeviceClock::operator()(int64_t device, Clock::time_point arrival)
{
    const int64_t offset = nanoseconds(arrival) - device;
    if (started_ && (device < last_device_ || offset < line(device) - kJumpNs)) {
        started_ = false;
        count_ = next_ = 0;
    }
    if (!started_ || device - window_start_ >= kWindowNs) {
        if (started_) {
            current_.held = fitted() && current_.offset - line(current_.device) >= kJumpNs;
            lows_[next_] = current_;
            next_ = (next_ + 1) % kWindows;
            if (count_ < kWindows) ++count_;
            if (fitted()) fit();
        }
        current_ = {device, offset};
        window_start_ = device;
        started_ = true;
    } else if (offset < current_.offset + static_cast<int64_t>(slope_ * static_cast<double>(device - current_.device))) {
        // Lowest against the drift already known, not where the drift alone puts it.
        current_ = {device, offset};
    }
    last_device_ = device;
    const int64_t time = device + std::min(line(device), offset);
    return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(time)));
}

void run(const Settings &settings, Output &output, const std::function<bool()> &running)
{
    hipnuc_serial_t serial{};
    Report report(output);
    Increasing increasing;
    // HI83 device_time_us and HI91 system_time_ms count from different origins.
    DeviceClock device_us, device_ms;
    const DeviceClock *stamped = nullptr;  // the clock of the last Imu, if any
    Utc utc;
    std::vector<int> candidates{settings.baudrate};
    for (int baudrate : kBaudrates)
        if (baudrate != settings.baudrate) candidates.push_back(baudrate);
    size_t candidate = 0;
    int baudrate = 0;  // of the open connection
    Device device;
    auto next_retry = Clock::now(), next_log = next_retry, opened_at = next_retry;
    Clock::time_point read_at{}, first_imu{};
    // Frames decoded from bytes that an earlier frame's read already brought:
    // their arrivals are spaced by line time, not by the output period.
    uint64_t bytes_at_report = 0, batched = 0, imu_count = 0;
    bool scan_bytes = false, scan_unsupported = false, rate_noted = false, imu_hint = false;
    const std::string looking = "looking for the device's baud rate";
    std::string error, searching = looking, found, axes;
    while (running()) {
        auto now = Clock::now();
        if (!serial.is_open && now >= next_retry) {
            const int trying = candidates[candidate];
            // This driver scans the baud rate itself, in its own order.
            if (hipnuc_serial_open(&serial, settings.port.c_str(), trying) != 0) {
                error = hipnuc_serial_last_error(&serial);
                next_retry = now + kRetry;
                if (now >= next_log) {
                    output.log(Level::warn, "cannot open " + settings.port + ": " + error + open_hint(settings.port));
                    next_log = now + kRetryLog;
                }
            } else if (!frames_arrive(serial, trying)) {
                error.clear();
                scan_bytes = scan_bytes || serial.bytes_read > 0;
                scan_unsupported = scan_unsupported || serial.decoder.unsupported > 0;
                hipnuc_serial_close(&serial);
                if (++candidate == candidates.size()) {
                    candidate = 0;
                    searching = scan_unsupported ? "frames this driver does not decode: check the HI83 fields "
                                                   "(LOG HI83 MAP) or the output frame"
                                : scan_bytes ? "bytes but no valid frames at any baud rate: check that the device outputs "
                                               "HI91, HI83 or HI81"
                                             : "no data at any baud rate: check the wiring and that the device output is on";
                    scan_bytes = scan_unsupported = false;
                }
            } else {
                error.clear();
                scan_bytes = scan_unsupported = false;
                searching = looking;
                baudrate = trying;
                // Replies cut short, say by another program probing the port,
                // keep what complete ones said before.
                const Device fresh = identify(serial, baudrate);
                if (fresh.complete || !device.complete) device = fresh;
                // Reconnecting to the same device logs nothing new.
                const std::string line = (device.model.empty() ? "device" : device.model) +
                                         (device.firmware.empty() ? "" : " firmware " + device.firmware) + " on " +
                                         settings.port + " at " + std::to_string(baudrate) + " baud";
                if (line != found) output.log(Level::info, line);
                if (device.axes != axes) output.log(Level::info, device.axes);
                found = line;
                axes = device.axes;
                report.opened();
                bytes_at_report = serial.bytes_read;
                imu_count = 0;
                opened_at = now;
                // Frames left from the scan and the queries arrived before now.
                read_at = Clock::now();
                device_us.reset();
                device_ms.reset();
                stamped = nullptr;
                utc = Utc();
            }
        }
        if (serial.is_open) {
            hipnuc_data_t sample;
            const uint64_t bytes_before = serial.bytes_read;
            const int result = hipnuc_serial_read(&serial, &sample, kPollMs);
            now = Clock::now();
            const bool received = serial.bytes_read != bytes_before;
            if (received) read_at = now;
            if (result > 0) {
                if (!received) ++batched;
                // The device began sending the message at least the line time
                // of its bytes and of the bytes read after it before that read.
                const bool nmea = sample.source == HIPNUC_SOURCE_NMEA_GGA || sample.source == HIPNUC_SOURCE_NMEA_RMC;
                const size_t bytes = nmea ? std::strlen(hipnuc_decoder_line(&serial.decoder)) + 2  // CR LF
                                          : serial.decoder.frame_size;
                const auto sent = read_at - line_time(bytes + hipnuc_serial_pending(&serial), baudrate);
                DeviceClock *clock = nullptr;
                int64_t device_ns = 0;
                if (sample.valid & HIPNUC_VALID_DEVICE_TIME) {
                    clock = &device_us;
                    device_ns = static_cast<int64_t>(sample.device_time_us) * 1000;
                } else if (sample.valid & HIPNUC_VALID_SYSTEM_TIME) {
                    clock = &device_ms;
                    device_ns = static_cast<int64_t>(sample.system_time_ms) * 1000000;
                }
                const auto time = utc(sample, clock ? (*clock)(device_ns, sent) : sent);
                report.frame(sample, now);
                const bool imu = has_imu(sample);
                output.measurement(sample, imu ? increasing(time) : time, imu);
                if (imu) {
                    stamped = clock;
                    if (imu_count++ == 0) first_imu = now;
                    if (!rate_noted && now - first_imu >= kHint) {
                        rate_noted = true;
                        const double rate = (imu_count - 1) / seconds(now - first_imu);
                        if (rate < 0.95 * kSlamRate)
                            output.log(Level::info, rate_hint(settings.port, baudrate, rate, sample, bytes));
                    }
                }
            } else if (result < 0) {
                error = hipnuc_serial_last_error(&serial);
                output.log(Level::error, settings.port + " read failed: " + error + "; reopening");
                hipnuc_serial_close(&serial);
                next_retry = now + kRetry;
                next_log = now;
            }
        } else if (!error.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
        }
        now = Clock::now();
        if (!report.due(now)) continue;
        if (serial.is_open && imu_count == 0 && !imu_hint && !report.silent(now) && now - opened_at >= kHint) {
            imu_hint = true;
            output.log(Level::warn, "imu/data stays silent because the frames lack acceleration or angular velocity; "
                                    "HI83 needs MAP bits 0 and 1");
        }
        Status status;
        bool reconnect = false;
        if (!serial.is_open && !error.empty()) {
            status.level = Level::error;
            status.message = "cannot open " + settings.port + ": " + error;
        } else if (!serial.is_open) {
            status.level = Level::warn;
            status.message = searching;
        } else if (report.silent(now) && serial.bytes_read != bytes_at_report) {
            // The connection opened on valid frames, so the line settings or the
            // device output changed: ModemManager, for one, probes a new ttyACM
            // port and leaves it at 57600 baud. Opening again finds them anew.
            status.level = Level::warn;
            status.message = "bytes received but no valid frames; reconnecting";
            reconnect = true;
        } else if (report.silent(now)) {
            status.level = Level::warn;
            status.message = "no data: check the wiring and that the device output is on";
        } else {
            status.level = Level::info;
            status.message = "receiving";
        }
        // The counters start again at each open and survive close.
        bytes_at_report = serial.bytes_read;
        status.values = {{"baudrate", std::to_string(serial.is_open ? baudrate : 0)},
                         {"device", device.model},
                         {"firmware", device.firmware},
                         {"bytes", std::to_string(serial.bytes_read)},
                         {"crc_errors", std::to_string(serial.decoder.crc_errors)},
                         {"invalid_frames", std::to_string(serial.decoder.unsupported)},
                         {"receive_errors", std::to_string(serial.rx_errors)},
                         {"nmea_checksum_errors", std::to_string(serial.nmea_errors)},
                         {"batched_frames", std::to_string(batched)},
                         {"stamp_adjustments", std::to_string(increasing.adjusted())},
                         {"stamp_clock", utc.on() ? "utc" : stamped ? "device" : "host"},
                         {"device_clock_ppm", stamped && stamped->fitted() ? text(stamped->ppm()) : ""}};
        report.publish(status, now);
        if (reconnect) hipnuc_serial_close(&serial);
    }
    hipnuc_serial_close(&serial);
}

}  // namespace hipnuc_imu
