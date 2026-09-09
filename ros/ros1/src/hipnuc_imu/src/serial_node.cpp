// One decoded frame produces one sample. No cross-frame measurement cache.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <hipnuc_imu/HipnucImu.h>

#include "hipnuc_serial.h"
#include "hipnuc_convert.hpp"

using SteadyClock = std::chrono::steady_clock;

class SerialNode {
public:
    SerialNode() : pnh_("~")
    {
        pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        pnh_.param<int>("baudrate", baudrate_, 115200);
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
        pnh_.param<bool>("publish_imu", publish_imu_, true);
        pnh_.param<bool>("publish_mag", publish_mag_, true);
        pnh_.param<bool>("publish_temperature", publish_temperature_, true);
        pnh_.param<bool>("publish_hipnuc", publish_hipnuc_, true);
        if (port_.empty() || baudrate_ <= 0 || frame_id_.empty())
            throw std::invalid_argument("port/frame_id must be nonempty and baudrate positive");
        ros::NodeHandle nh;
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
        mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
        temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
        full_pub_ = nh.advertise<hipnuc_imu::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
        ROS_INFO("Requires device ENU output configuration; the driver does not verify or change it.");
    }

    ~SerialNode() { hipnuc_serial_close(&serial_); }

    void run()
    {
        auto last_diag = SteadyClock::now();
        auto next_retry = last_diag;
        auto next_warning = last_diag;
        while (ros::ok()) {
            auto current = SteadyClock::now();
            if (!serial_.is_open && current >= next_retry) {
                const bool opened = hipnuc_serial_open(&serial_, port_.c_str(), baudrate_) == 0;
                next_retry = current + std::chrono::seconds(1);
                if (opened) {
                    received_sample_ = false;
                    bytes_at_last_diag_ = 0;
                    ROS_INFO("opened %s at %d baud", port_.c_str(), baudrate_);
                } else if (current >= next_warning) {
                    ROS_WARN("cannot open %s: %s", port_.c_str(), hipnuc_serial_last_error(&serial_));
                    next_warning = current + std::chrono::seconds(5);
                }
            }
            if (serial_.is_open) {
                hipnuc_sample_t sample;
                const int result = hipnuc_serial_read_sample(&serial_, &sample, 50);
                if (result > 0) publish(sample);
                if (result < 0) {
                    ROS_ERROR("%s read failed: %s; reopening", port_.c_str(), hipnuc_serial_last_error(&serial_));
                    hipnuc_serial_close(&serial_);
                    received_sample_ = false;
                    next_retry = SteadyClock::now() + std::chrono::seconds(1);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            // Failure and idle paths must still service ROS and diagnostics.
            ros::spinOnce();
            current = SteadyClock::now();
            const double elapsed = std::chrono::duration<double>(current - last_diag).count();
            if (elapsed >= 1.0) {
                publish_diagnostics(current, elapsed);
                last_diag = current;
            }
        }
    }

private:
    void publish(const hipnuc_sample_t &s)
    {
        const auto stamp = ros::Time::now();
        ++frames_;
        last_frame_ = SteadyClock::now();
        received_sample_ = true;
        if (publish_imu_ && hipnuc_ros::has_imu(s)) {
            sensor_msgs::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_.publish(m);
        }
        if (publish_mag_ && (s.valid & HIPNUC_VALID_MAG) && hipnuc_ros::finite_vector(s.mag)) {
            sensor_msgs::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_.publish(m);
        }
        if (publish_temperature_ && (s.valid & HIPNUC_VALID_TEMPERATURE) && std::isfinite(s.temperature)) {
            sensor_msgs::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_.publish(m);
        }
        if (publish_hipnuc_) {
            hipnuc_imu::HipnucImu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_hipnuc(s, m);
            full_pub_.publish(m);
        }
    }

    void publish_diagnostics(SteadyClock::time_point current, double elapsed)
    {
        diagnostic_msgs::DiagnosticArray arr;
        diagnostic_msgs::DiagnosticStatus st;
        arr.header.stamp = ros::Time::now();
        st.name = ros::this_node::getName() + ": serial";
        st.hardware_id = port_;
        const double age = received_sample_ ? std::chrono::duration<double>(current - last_frame_).count() : -1.0;
        if (!serial_.is_open) {
            st.level = st.ERROR;
            st.message = "port not open";
        } else if (age < 0.0 || age > 2.0) {
            st.level = st.WARN;
            st.message = serial_.bytes_received > bytes_at_last_diag_ ? "bytes received but no valid frames (check baudrate/output)" : "no data";
        } else {
            st.level = st.OK;
            st.message = "receiving";
        }
        // Nothing else reaches the console after a successful open: report each change.
        if (st.level != last_level_) {
            last_level_ = st.level;
            if (st.level == st.OK) ROS_INFO("%s", st.message.c_str());
            else ROS_WARN("%s", st.message.c_str());
        }
        auto kv = [&](const char *key, const std::string &value) {
            diagnostic_msgs::KeyValue entry;
            entry.key = key;
            entry.value = value;
            st.values.push_back(entry);
        };
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string((frames_ - frames_at_last_diag_) / elapsed));
        kv("connection_bytes", std::to_string(serial_.bytes_received));
        kv("connection_crc_errors", std::to_string(serial_.binary.crc_error_count));
        kv("connection_receive_errors", std::to_string(serial_.receive_errors));
        kv("connection_invalid_frames", std::to_string(serial_.binary.invalid_count));
        kv("connection_nmea_checksum_errors", std::to_string(serial_.nmea.checksum_error_count));
        kv("last_error", hipnuc_serial_last_error(&serial_));
        bytes_at_last_diag_ = serial_.bytes_received;
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_.publish(arr);
    }

    ros::NodeHandle pnh_;
    std::string port_, frame_id_;
    int baudrate_;
    bool publish_imu_, publish_mag_, publish_temperature_, publish_hipnuc_;
    hipnuc_serial_t serial_{};
    uint64_t frames_ = 0, frames_at_last_diag_ = 0;
    uint64_t bytes_at_last_diag_ = 0;
    bool received_sample_ = false;
    int last_level_ = -1;
    SteadyClock::time_point last_frame_{};
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher temp_pub_;
    ros::Publisher full_pub_;
    ros::Publisher diag_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hipnuc_serial");
    try {
        SerialNode node;
        node.run();
    } catch (const std::exception &error) {
        ROS_ERROR("%s", error.what());
        return 1;
    }
    return 0;
}
