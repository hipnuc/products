#include "driver.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <thread>
#include <vector>

#include <unistd.h>

#include "convert.hpp"
#include "hipnuc_serial.h"
extern "C" {
#include "serial_io.h"
}

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

void add_device_status(Status &status, const hipnuc_sample_t &sample)
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
    flag("accelerometer_saturated", sample.main_status & (1u << 5));
    flag("gyroscope_saturated", sample.main_status & (1u << 6));
}

// Frame counting, the once-per-second status and its log line on each change.
class Report {
public:
    explicit Report(Output &output) : output_(output) {}

    void frame(const hipnuc_sample_t &sample, Clock::time_point now)
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
        hipnuc_sample_clear(&status_sample_);
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
    hipnuc_sample_t status_sample_{};
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

size_t frame_bytes(const hipnuc_serial_t &serial, const hipnuc_sample_t &sample)
{
    if (sample.source == HIPNUC_SOURCE_NMEA_GGA || sample.source == HIPNUC_SOURCE_NMEA_RMC)
        return std::strlen(reinterpret_cast<const char *>(serial.nmea.buf)) + 2;  // and CR LF
    return HIPNUC_HEADER_SIZE + static_cast<size_t>(serial.binary.len);
}

std::string open_hint(const std::string &port)
{
    if (access(port.c_str(), F_OK) != 0) return " (check the cable and the port name: ls /dev/serial/by-id)";
    if (access(port.c_str(), R_OK | W_OK) != 0)
        return " (run: sudo usermod -aG dialout $USER, then log out and back in)";
    return "";
}

// The byte reads below happen before the first read_sample() on a connection,
// so no buffered input is skipped.

// At least kWaitMs, and three of the longest frames (150 bytes) at low baud rates.
int wait_ms(int baudrate) { return std::max(kWaitMs, 3 * 1500 * 1000 / baudrate); }

// True once two valid frames or NMEA sentences arrive: the baud rate is right.
// A CH347 can still hold a frame it received at the previous baud rate, so the
// first bytes after opening prove nothing.
bool frames_arrive(hipnuc_serial_t &serial, int baudrate)
{
    hipnuc_raw_t binary{};
    nmea_raw_t nmea{};
    uint8_t data[256];
    int valid = 0;
    const uint64_t start = hipnuc_serial_monotonic_ms(), deadline = start + kStaleMs + wait_ms(baudrate);
    for (uint64_t now = start; now < deadline; now = hipnuc_serial_monotonic_ms()) {
        const int count = hipnuc_serial_read_bytes(&serial, data, sizeof(data), static_cast<int>(deadline - now));
        if (count < 0) return false;
        if (hipnuc_serial_monotonic_ms() < start + kStaleMs) continue;
        for (int i = 0; i < count; ++i)
            if ((hipnuc_input(&binary, data[i]) > 0 || nmea_input(&nmea, data[i]) > 0) && ++valid == 2) return true;
    }
    return false;
}

// Values of the given keys in the reply to a read-only query; none when the
// device does not answer. Frames keep streaming meanwhile: valid ones are cut
// out, and one the device cut short when the command arrived may leave bytes
// in front of a reply line, so keys are searched for within lines. complete
// turns false when the reply does not end with OK or ERR.
std::map<std::string, std::string> query(hipnuc_serial_t &serial, int baudrate, const char *command,
                                         std::initializer_list<const char *> keys, bool &complete)
{
    std::map<std::string, std::string> values;
    bool ended = false;
    // The leading line end ends any partial line the device holds.
    const std::string line = std::string("\r\n") + command + "\r\n";
    std::string text;
    if (::write(static_cast<int>(serial.handle), line.data(), line.size()) == static_cast<ssize_t>(line.size())) {
        hipnuc_raw_t binary{};
        uint8_t data[256];
        const uint64_t deadline = hipnuc_serial_monotonic_ms() + wait_ms(baudrate);
        for (uint64_t now = hipnuc_serial_monotonic_ms(); !ended && now < deadline; now = hipnuc_serial_monotonic_ms()) {
            const int count = hipnuc_serial_read_bytes(&serial, data, sizeof(data), static_cast<int>(deadline - now));
            if (count < 0) break;
            for (int i = 0; i < count; ++i) {
                text += static_cast<char>(data[i]);
                if (hipnuc_input(&binary, data[i]) > 0) text.resize(text.size() - (HIPNUC_HEADER_SIZE + binary.len));
            }
            ended = text.find("\nOK\r") != std::string::npos || text.find("\nERR") != std::string::npos;
        }
    }
    complete = complete && ended;
    for (size_t start = 0, end; start < text.size(); start = end + 1) {
        end = std::min(text.find('\n', start), text.size());
        const std::string reply = text.substr(start, end - start);
        for (const char *key : keys) {
            const auto at = reply.find(std::string(key) + "=");
            if (at == std::string::npos) continue;
            std::string value = reply.substr(at + std::strlen(key) + 1);
            while (!value.empty() && (value.back() < 0x20 || value.back() > 0x7E)) value.pop_back();
            values[key] = value;
        }
    }
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
    // Firmware built from late 2025 lists the saved axes setting. COORD 4 turns
    // the vectors only on some recent firmware, and the version does not tell.
    const std::string coord = query(serial, baudrate, "LOG USRCONFIG", {"COORD"}, device.complete)["COORD"];
    const std::string readme = "see \"Axes and TF\" in the README";
    if (coord == "0")
        device.axes = "device axes: x right, y forward, z up (COORD 0); " + readme;
    else if (coord == "4")
        device.axes = "device axes setting COORD 4 gives x forward, y left, z up only on some recent firmware; "
                      "check them as described in \"Axes and TF\" in the README";
    else if (!coord.empty())
        device.axes = "device axes setting COORD " + coord + "; " + readme;
    else
        device.axes = "this firmware does not report its axes; " + readme;
    return device;
}

// Baud rate and output period that carry kSlamRate with 20% headroom.
std::string rate_hint(const std::string &port, int baudrate, double rate, const hipnuc_sample_t &sample, size_t bytes)
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

}  // namespace

void run(const Settings &settings, Output &output, const std::function<bool()> &running)
{
    hipnuc_serial_t serial{};
    Report report(output);
    Increasing increasing;
    std::vector<int> candidates{settings.baudrate};
    for (int baudrate : kBaudrates)
        if (baudrate != settings.baudrate) candidates.push_back(baudrate);
    size_t candidate = 0;
    int baudrate = 0;  // of the open connection
    Device device;
    auto next_retry = Clock::now(), next_log = next_retry, opened_at = next_retry;
    Clock::time_point read_at{}, first_imu{};
    // Frames decoded from bytes that an earlier frame's read already brought:
    // their stamps are spaced by line time, not by the output period.
    uint64_t bytes_at_report = 0, batched = 0, imu_count = 0;
    bool scan_bytes = false, rate_noted = false, imu_hint = false;
    const std::string looking = "looking for the device's baud rate";
    std::string error, searching = looking, found, axes;
    while (running()) {
        auto now = Clock::now();
        if (!serial.is_open && now >= next_retry) {
            const int trying = candidates[candidate];
            if (hipnuc_serial_open(&serial, settings.port.c_str(), trying) != 0) {
                error = hipnuc_serial_last_error(&serial);
                next_retry = now + kRetry;
                if (now >= next_log) {
                    output.log(Level::warn, "cannot open " + settings.port + ": " + error + open_hint(settings.port));
                    next_log = now + kRetryLog;
                }
            } else if (!frames_arrive(serial, trying)) {
                error.clear();
                scan_bytes = scan_bytes || serial.bytes_received > 0;
                hipnuc_serial_close(&serial);
                if (++candidate == candidates.size()) {
                    candidate = 0;
                    searching = scan_bytes ? "bytes but no valid frames at any baud rate: check that the device outputs "
                                             "HI91, HI83 or HI81"
                                           : "no data at any baud rate: check the wiring and that the device output is on";
                    scan_bytes = false;
                }
            } else {
                error.clear();
                scan_bytes = false;
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
                bytes_at_report = serial.bytes_received;
                imu_count = 0;
                opened_at = now;
            }
        }
        if (serial.is_open) {
            hipnuc_sample_t sample;
            const uint64_t bytes_before = serial.bytes_received;
            const int result = hipnuc_serial_read_sample(&serial, &sample, kPollMs);
            now = Clock::now();
            if (serial.bytes_received != bytes_before) read_at = now;
            if (result > 0) {
                if (serial.bytes_received == bytes_before) ++batched;
                // The device began sending the frame at least the line time of
                // its bytes and of the bytes read after it before that read.
                const size_t bytes = frame_bytes(serial, sample);
                const auto sent = read_at - line_time(bytes + serial.received_size - serial.received_offset, baudrate);
                report.frame(sample, now);
                const bool imu = has_imu(sample);
                output.measurement(sample, imu ? increasing(sent) : sent, imu);
                if (imu) {
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
        } else if (report.silent(now) && serial.bytes_received > bytes_at_report) {
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
        bytes_at_report = serial.bytes_received;
        status.values = {{"baudrate", std::to_string(serial.is_open ? baudrate : 0)},
                         {"device", device.model},
                         {"firmware", device.firmware},
                         {"bytes", std::to_string(serial.bytes_received)},
                         {"crc_errors", std::to_string(serial.binary.crc_error_count)},
                         {"invalid_frames", std::to_string(serial.binary.invalid_count)},
                         {"receive_errors", std::to_string(serial.receive_errors)},
                         {"nmea_checksum_errors", std::to_string(serial.nmea.checksum_error_count)},
                         {"batched_frames", std::to_string(batched)},
                         {"stamp_adjustments", std::to_string(increasing.adjusted())}};
        report.publish(status, now);
        if (reconnect) hipnuc_serial_close(&serial);
    }
    hipnuc_serial_close(&serial);
}

}  // namespace hipnuc_imu
