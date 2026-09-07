// HiPNUC serial driver node (ROS 2). Reads HI91/HI81/HI83 binary frames and
// GGA/RMC sentences from a serial port and publishes standard sensor
// messages plus the full-field hipnuc_msgs/HipnucImu.

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <hipnuc_msgs/msg/hipnuc_imu.hpp>

#include "hipnuc_dec.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"
#include "hipnuc_convert.hpp"
#include "posix_serial.hpp"

using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node {
public:
    SerialNode() : Node("hipnuc_serial")
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
        gnss_frame_id_ = declare_parameter<std::string>("gnss_frame_id", "gnss_antenna");
        enu_frame_id_ = declare_parameter<std::string>("enu_frame_id", "enu");
        auto p = [this](const char *name, bool def) { return declare_parameter<bool>(name, def); };
        publish_imu_ = p("publish_imu", true);
        publish_mag_ = p("publish_mag", true);
        publish_env_ = p("publish_temperature_pressure", true);
        publish_fix_ = p("publish_navsatfix", true);
        publish_vel_ = p("publish_velocity", true);
        publish_full_ = p("publish_hipnuc", true);

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
        press_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("imu/pressure", 10);
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
        vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("ins/velocity", 10);
        full_pub_ = create_publisher<hipnuc_msgs::msg::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        std::memset(&raw_, 0, sizeof(raw_));
        std::memset(&nmea_, 0, sizeof(nmea_));
    }

    // Blocking read loop; returns when ROS shuts down.
    void run()
    {
        uint8_t buf[512];
        auto last_diag = now();
        while (rclcpp::ok()) {
            if (!serial_.is_open()) {
                std::string err = serial_.open(port_, baudrate_);
                if (!err.empty()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "cannot open %s: %s (check the cable, the dialout group and the port name)",
                                         port_.c_str(), err.c_str());
                    rclcpp::sleep_for(1s);
                    continue;
                }
                RCLCPP_INFO(get_logger(), "opened %s at %d baud", port_.c_str(), baudrate_);
                std::memset(&raw_, 0, sizeof(raw_));
                std::memset(&nmea_, 0, sizeof(nmea_));
            }
            int n = serial_.read(buf, sizeof(buf), 100);
            if (n < 0) {
                RCLCPP_ERROR(get_logger(), "%s disconnected; reopening", port_.c_str());
                serial_.close();
                continue;
            }
            for (int i = 0; i < n; ++i) feed(buf[i]);
            bytes_ += n;
            rclcpp::spin_some(shared_from_this());
            if (now() - last_diag > 1s) {
                publish_diagnostics();
                last_diag = now();
            }
        }
    }

private:
    void feed(uint8_t byte)
    {
        hipnuc_sample_t s;
        int ret = hipnuc_input(&raw_, byte);
        if (ret > 0 && hipnuc_sample_from_raw(&raw_, &s)) publish(s);
        if (nmea_input(&nmea_, byte) > 0 && hipnuc_sample_from_nmea(&nmea_, &s)) publish(s);
    }

    void publish(const hipnuc_sample_t &s)
    {
        const auto stamp = now();
        frames_++;
        last_frame_ = stamp;
        if (publish_imu_ && (s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_QUAT))) {
            sensor_msgs::msg::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_->publish(m);
        }
        if (publish_mag_ && (s.valid & HIPNUC_VALID_MAG)) {
            sensor_msgs::msg::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_->publish(m);
        }
        if (publish_env_ && (s.valid & HIPNUC_VALID_TEMPERATURE)) {
            sensor_msgs::msg::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_->publish(m);
        }
        if (publish_env_ && (s.valid & HIPNUC_VALID_PRESSURE)) {
            sensor_msgs::msg::FluidPressure m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_pressure(s, m);
            press_pub_->publish(m);
        }
        if (publish_fix_ && (s.valid & HIPNUC_VALID_POSITION)) {
            sensor_msgs::msg::NavSatFix m;
            m.header.stamp = stamp;
            m.header.frame_id = gnss_frame_id_;
            hipnuc_ros::fill_navsatfix(s, m);
            fix_pub_->publish(m);
        }
        if (publish_vel_ && (s.valid & HIPNUC_VALID_VELOCITY_ENU)) {
            geometry_msgs::msg::TwistWithCovarianceStamped m;
            m.header.stamp = stamp;
            m.header.frame_id = enu_frame_id_;
            hipnuc_ros::fill_velocity_enu(s, m);
            vel_pub_->publish(m);
        }
        if (publish_full_) {
            hipnuc_msgs::msg::HipnucImu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_hipnuc(s, m);
            full_pub_->publish(m);
        }
    }

    void publish_diagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus st;
        arr.header.stamp = now();
        st.name = std::string(get_name()) + ": serial";
        st.hardware_id = port_;
        const double age = last_frame_.nanoseconds() ? (now() - last_frame_).seconds() : -1.0;
        if (!serial_.is_open()) { st.level = st.ERROR; st.message = "port not open"; }
        else if (age < 0 || age > 2.0) { st.level = st.WARN; st.message = bytes_ ? "no valid frames (baudrate?)" : "no data"; }
        else { st.level = st.OK; st.message = "receiving"; }
        auto kv = [&](const char *k, const std::string &v) {
            diagnostic_msgs::msg::KeyValue e; e.key = k; e.value = v; st.values.push_back(e);
        };
        kv("bytes", std::to_string(bytes_));
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string(frames_ - frames_at_last_diag_));
        kv("crc_errors", std::to_string(raw_.crc_error_count));
        kv("invalid_frames", std::to_string(raw_.invalid_count));
        kv("nmea_checksum_errors", std::to_string(nmea_.checksum_error_count));
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_->publish(arr);
    }

    std::string port_, frame_id_, gnss_frame_id_, enu_frame_id_;
    int baudrate_;
    bool publish_imu_, publish_mag_, publish_env_, publish_fix_, publish_vel_, publish_full_;
    hipnuc_ros::PosixSerial serial_;
    hipnuc_raw_t raw_;
    nmea_raw_t nmea_;
    uint64_t bytes_ = 0, frames_ = 0, frames_at_last_diag_ = 0;
    rclcpp::Time last_frame_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr press_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<hipnuc_msgs::msg::HipnucImu>::SharedPtr full_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
