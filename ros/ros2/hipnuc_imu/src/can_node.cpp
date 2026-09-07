// HiPNUC CAN (J1939 / CANFD83) driver node (ROS 2). Frames from the
// configured source address are merged into one sample; the merged sample
// is published when the trigger PGN arrives (the last PGN of the device's
// output cycle) and then cleared, so fields never outlive one cycle.

#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <hipnuc_msgs/msg/hipnuc_imu.hpp>

#include "hipnuc_j1939.h"
#include "hipnuc_convert.hpp"
#include "socketcan.hpp"

using namespace std::chrono_literals;

class CanNode : public rclcpp::Node {
public:
    CanNode() : Node("hipnuc_can")
    {
        interface_ = declare_parameter<std::string>("interface", "can0");
        node_id_ = declare_parameter<int>("node_id", HIPNUC_J1939_DEFAULT_NODE);
        trigger_pgn_ = declare_parameter<int>("trigger_pgn", HIPNUC_J1939_PGN_YAW);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
        gnss_frame_id_ = declare_parameter<std::string>("gnss_frame_id", "gnss_antenna");
        enu_frame_id_ = declare_parameter<std::string>("enu_frame_id", "enu");
        publish_full_ = declare_parameter<bool>("publish_hipnuc", true);

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100);
        mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
        temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
        fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", 10);
        vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("ins/velocity", 10);
        full_pub_ = create_publisher<hipnuc_msgs::msg::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        hipnuc_sample_clear(&merged_);
    }

    void run()
    {
        auto last_diag = now();
        while (rclcpp::ok()) {
            if (!can_.is_open()) {
                std::string err = can_.open(interface_);
                if (!err.empty()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "cannot open %s: %s (is the interface up?)", interface_.c_str(), err.c_str());
                    rclcpp::sleep_for(1s);
                    continue;
                }
                RCLCPP_INFO(get_logger(), "listening on %s for node %d", interface_.c_str(), node_id_);
            }
            hipnuc_can_frame_t frame;
            int r = can_.read(frame, 100);
            if (r < 0) {
                RCLCPP_ERROR(get_logger(), "%s read failed; reopening", interface_.c_str());
                can_.close();
                continue;
            }
            if (r > 0) handle(frame);
            rclcpp::spin_some(shared_from_this());
            if (now() - last_diag > 1s) {
                publish_diagnostics();
                last_diag = now();
            }
        }
    }

private:
    void handle(const hipnuc_can_frame_t &frame)
    {
        hipnuc_sample_t part;
        int type = hipnuc_j1939_parse(&frame, &part, nullptr);
        if (type < 0) { invalid_++; return; }
        if (type == HIPNUC_J1939_MSG_NONE) return;
        if (part.node_id != node_id_) { other_nodes_++; return; }
        frames_++;
        last_frame_ = now();
        if (type == HIPNUC_J1939_MSG_CANFD83) {
            publish(part);                       // one frame carries a complete sample
            return;
        }
        hipnuc_j1939_merge(&merged_, &part);
        if (static_cast<int>(hipnuc_j1939_pgn(frame.id)) == trigger_pgn_) {
            publish(merged_);
            hipnuc_sample_clear(&merged_);
        }
    }

    void publish(const hipnuc_sample_t &s)
    {
        const auto stamp = now();
        if (s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_QUAT)) {
            sensor_msgs::msg::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_->publish(m);
        }
        if (s.valid & HIPNUC_VALID_MAG) {
            sensor_msgs::msg::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_->publish(m);
        }
        if (s.valid & HIPNUC_VALID_TEMPERATURE) {
            sensor_msgs::msg::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_->publish(m);
        }
        if (s.valid & HIPNUC_VALID_POSITION) {
            sensor_msgs::msg::NavSatFix m;
            m.header.stamp = stamp;
            m.header.frame_id = gnss_frame_id_;
            hipnuc_ros::fill_navsatfix(s, m);
            fix_pub_->publish(m);
        }
        if (s.valid & HIPNUC_VALID_VELOCITY_ENU) {
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
        st.name = std::string(get_name()) + ": can";
        st.hardware_id = interface_;
        const double age = last_frame_.nanoseconds() ? (now() - last_frame_).seconds() : -1.0;
        if (!can_.is_open()) { st.level = st.ERROR; st.message = "interface not open"; }
        else if (age < 0 || age > 2.0) { st.level = st.WARN; st.message = other_nodes_ ? "no frames from the configured node id" : "no frames"; }
        else { st.level = st.OK; st.message = "receiving"; }
        auto kv = [&](const char *k, const std::string &v) {
            diagnostic_msgs::msg::KeyValue e; e.key = k; e.value = v; st.values.push_back(e);
        };
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string(frames_ - frames_at_last_diag_));
        kv("invalid_frames", std::to_string(invalid_));
        kv("frames_from_other_nodes", std::to_string(other_nodes_));
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_->publish(arr);
    }

    std::string interface_, frame_id_, gnss_frame_id_, enu_frame_id_;
    int node_id_, trigger_pgn_;
    bool publish_full_;
    hipnuc_ros::SocketCan can_;
    hipnuc_sample_t merged_;
    uint64_t frames_ = 0, frames_at_last_diag_ = 0, invalid_ = 0, other_nodes_ = 0;
    rclcpp::Time last_frame_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_pub_;
    rclcpp::Publisher<hipnuc_msgs::msg::HipnucImu>::SharedPtr full_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
