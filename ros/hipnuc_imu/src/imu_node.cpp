// ROS 2 node for a serial HiPNUC device.
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <hipnuc_imu/msg/hipnuc_imu.hpp>

#include "convert.hpp"
#include "driver.hpp"

namespace hipnuc_imu {

class ImuNode : public rclcpp::Node, private Output {
public:
    explicit ImuNode(const rclcpp::NodeOptions &options) : Node("hipnuc_imu", options)
    {
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        startup.description = "Applied at startup; restart the node to change it.";
        settings_.port = declare_parameter<std::string>("port", "/dev/ttyUSB0", startup);
        settings_.baudrate = declare_parameter<int>("baudrate", 115200, startup);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link", startup);
        const bool publish_imu = declare_parameter<bool>("publish_imu", true, startup);
        const bool publish_data_raw = declare_parameter<bool>("publish_data_raw", false, startup);
        const bool publish_mag = declare_parameter<bool>("publish_mag", false, startup);
        const bool publish_temperature = declare_parameter<bool>("publish_temperature", false, startup);
        const bool publish_hipnuc = declare_parameter<bool>("publish_hipnuc", false, startup);
        if (frame_id_.empty()) throw std::invalid_argument("frame_id must not be empty");
        if (settings_.port.empty() || settings_.baudrate <= 0)
            throw std::invalid_argument("port must not be empty and baudrate must be positive");

        // Depth and history can be overridden at startup. Keep reliable: a best
        // effort publisher never connects to reliable subscribers (FAST-LIO ROS 2).
        rclcpp::PublisherOptions options_qos;
        options_qos.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        if (publish_imu) imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100, options_qos);
        if (publish_data_raw) raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100, options_qos);
        if (publish_mag) mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100, options_qos);
        if (publish_temperature)
            temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10, options_qos);
        if (publish_hipnuc) full_pub_ = create_publisher<msg::HipnucImu>("hipnuc/imu", 100, options_qos);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        diag_name_ = get_fully_qualified_name();

        auto context = get_node_base_interface()->get_context();
        worker_ = std::thread([this, context] {
            run(settings_, *this, [this, context] { return !stop_ && rclcpp::ok(context); });
        });
    }

    ~ImuNode() override
    {
        stop_ = true;
        if (worker_.joinable()) worker_.join();
    }

private:
    // False while a simulated clock has not yet reached the measurement.
    bool stamp(Clock::time_point at, rclcpp::Time &out)
    {
        const rclcpp::Time clock = now();
        const rclcpp::Duration age(Clock::now() - at);
        if (clock.nanoseconds() <= age.nanoseconds()) return false;
        out = clock - age;
        return true;
    }

    void log(Level level, const std::string &text) override
    {
        if (level == Level::info) RCLCPP_INFO(get_logger(), "%s", text.c_str());
        else if (level == Level::warn) RCLCPP_WARN(get_logger(), "%s", text.c_str());
        else RCLCPP_ERROR(get_logger(), "%s", text.c_str());
    }

    void measurement(const hipnuc_sample_t &s, Clock::time_point at, bool imu) override
    {
        rclcpp::Time time;
        if (!stamp(at, time)) return;
        if (imu && (imu_pub_ || raw_pub_)) {
            sensor_msgs::msg::Imu m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_imu(s, m);
            if (imu_pub_) imu_pub_->publish(m);
            if (raw_pub_) {
                clear_orientation(m);
                raw_pub_->publish(m);
            }
        }
        if (mag_pub_ && (s.valid & HIPNUC_VALID_MAG) && finite_vector(s.mag)) {
            sensor_msgs::msg::MagneticField m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_mag(s, m);
            mag_pub_->publish(m);
        }
        if (temp_pub_ && (s.valid & HIPNUC_VALID_TEMPERATURE) && std::isfinite(s.temperature)) {
            sensor_msgs::msg::Temperature m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_temperature(s, m);
            temp_pub_->publish(m);
        }
        if (full_pub_) {
            msg::HipnucImu m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_hipnuc(s, m);
            full_pub_->publish(m);
        }
    }

    void status(const Status &st) override
    {
        using diagnostic_msgs::msg::DiagnosticStatus;
        diagnostic_msgs::msg::DiagnosticArray array;
        DiagnosticStatus diag;
        array.header.stamp = now();
        diag.name = diag_name_;
        diag.hardware_id = settings_.port;
        diag.level = st.level == Level::info ? DiagnosticStatus::OK
                   : st.level == Level::warn ? DiagnosticStatus::WARN
                   : DiagnosticStatus::ERROR;
        diag.message = st.message;
        for (const auto &value : st.values) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = value.first;
            entry.value = value.second;
            diag.values.push_back(entry);
        }
        array.status.push_back(diag);
        diag_pub_->publish(array);
    }

    Settings settings_;
    std::string frame_id_, diag_name_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_, raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<msg::HipnucImu>::SharedPtr full_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

}  // namespace hipnuc_imu

RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_imu::ImuNode)
