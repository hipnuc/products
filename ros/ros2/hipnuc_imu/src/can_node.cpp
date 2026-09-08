// One decoded frame produces one sample. No cross-frame measurement cache.
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <hipnuc_msgs/msg/hipnuc_imu.hpp>

#include "hipnuc_j1939.h"
#include "hipnuc_convert.hpp"
#include "socketcan.hpp"

using SteadyClock = std::chrono::steady_clock;

class CanNode : public rclcpp::Node {
public:
    CanNode() : Node("hipnuc_can")
    {
        interface_ = declare_parameter<std::string>("interface", "can0");
        node_id_ = declare_parameter<int>("node_id", 8);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
        publish_imu_ = declare_parameter<bool>("publish_imu", true);
        publish_mag_ = declare_parameter<bool>("publish_mag", true);
        publish_temperature_ = declare_parameter<bool>("publish_temperature", true);
        publish_hipnuc_ = declare_parameter<bool>("publish_hipnuc", true);
        if (interface_.empty() || node_id_ < 0 || node_id_ > 255 || frame_id_.empty())
            throw std::invalid_argument("interface/frame_id must be nonempty and node_id in 0..255");
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
        full_pub_ = create_publisher<hipnuc_msgs::msg::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        RCLCPP_INFO(get_logger(), "Requires device ENU output configuration; the driver does not verify or change it.");
    }

    void run()
    {
        auto last_diag = SteadyClock::now();
        auto next_retry = last_diag;
        auto next_warning = last_diag;
        while (rclcpp::ok()) {
            auto current = SteadyClock::now();
            if (!can_.is_open() && current >= next_retry) {
                const std::string error = can_.open(interface_);
                const bool opened = error.empty();
                next_retry = current + std::chrono::seconds(1);
                if (opened) {
                    received_sample_ = false;
                    other_nodes_at_open_ = other_nodes_;
                    RCLCPP_INFO(get_logger(), "listening on %s for source address %d", interface_.c_str(), node_id_);
                } else if (current >= next_warning) {
                    RCLCPP_WARN(get_logger(), "cannot open %s: %s", interface_.c_str(), error.c_str());
                    next_warning = current + std::chrono::seconds(5);
                }
            }
            if (can_.is_open()) {
                hipnuc_can_frame_t frame;
                const int result = can_.read(frame, 50);
                if (result > 0) handle(frame);
                if (result < 0) {
                    RCLCPP_ERROR(get_logger(), "%s read failed; reopening", interface_.c_str());
                    can_.close();
                    received_sample_ = false;
                    next_retry = SteadyClock::now() + std::chrono::seconds(1);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            // Failure and idle paths must still service ROS and diagnostics.
            rclcpp::spin_some(shared_from_this());
            current = SteadyClock::now();
            const double elapsed = std::chrono::duration<double>(current - last_diag).count();
            if (elapsed >= 1.0) {
                publish_diagnostics(current, elapsed);
                last_diag = current;
            }
        }
    }

private:
    void handle(const hipnuc_can_frame_t &frame)
    {
        hipnuc_sample_t sample;
        const int type = hipnuc_j1939_parse(&frame, &sample, nullptr);
        if (type < 0) { ++invalid_; return; }
        if (type == HIPNUC_J1939_MSG_NONE) return;
        if (sample.node_id != node_id_) { ++other_nodes_; return; }
        publish(sample);
    }

    void publish(const hipnuc_sample_t &s)
    {
        const auto stamp = now();
        ++frames_;
        last_frame_ = SteadyClock::now();
        received_sample_ = true;
        if (publish_imu_ && hipnuc_ros::has_imu(s)) {
            sensor_msgs::msg::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_->publish(m);
        }
        if (publish_mag_ && (s.valid & HIPNUC_VALID_MAG) && hipnuc_ros::finite_vector(s.mag)) {
            sensor_msgs::msg::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_->publish(m);
        }
        if (publish_temperature_ && (s.valid & HIPNUC_VALID_TEMPERATURE) && std::isfinite(s.temperature)) {
            sensor_msgs::msg::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_->publish(m);
        }
        if (publish_hipnuc_) {
            hipnuc_msgs::msg::HipnucImu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_hipnuc(s, m);
            full_pub_->publish(m);
        }
    }

    void publish_diagnostics(SteadyClock::time_point current, double elapsed)
    {
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus st;
        arr.header.stamp = now();
        st.name = std::string(get_name()) + ": can";
        st.hardware_id = interface_;
        const double age = received_sample_ ? std::chrono::duration<double>(current - last_frame_).count() : -1.0;
        if (!can_.is_open()) {
            st.level = st.ERROR;
            st.message = "interface not open";
        } else if (age < 0.0 || age > 2.0) {
            st.level = st.WARN;
            st.message = other_nodes_ > other_nodes_at_open_ ? "no frames from the configured source address" : "no valid frames";
        } else {
            st.level = st.OK;
            st.message = "receiving";
        }
        auto kv = [&](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = value;
            st.values.push_back(entry);
        };
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string((frames_ - frames_at_last_diag_) / elapsed));
        kv("invalid_frames", std::to_string(invalid_));
        kv("frames_from_other_nodes", std::to_string(other_nodes_));
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_->publish(arr);
    }

    std::string interface_, frame_id_;
    int node_id_;
    bool publish_imu_, publish_mag_, publish_temperature_, publish_hipnuc_;
    hipnuc_ros::SocketCan can_;
    uint64_t frames_ = 0, frames_at_last_diag_ = 0;
    uint64_t invalid_ = 0, other_nodes_ = 0, other_nodes_at_open_ = 0;
    bool received_sample_ = false;
    SteadyClock::time_point last_frame_{};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<hipnuc_msgs::msg::HipnucImu>::SharedPtr full_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<CanNode>();
        node->run();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("hipnuc"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
