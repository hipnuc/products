#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
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
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        startup.description = "Applied at startup; restart the node to change this parameter.";
        interface_ = declare_parameter<std::string>("interface", "can0", startup);
        node_id_ = declare_parameter<int>("node_id", 8, startup);
        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link", startup);
        publish_imu_ = declare_parameter<bool>("publish_imu", true, startup);
        publish_mag_ = declare_parameter<bool>("publish_mag", true, startup);
        publish_temperature_ = declare_parameter<bool>("publish_temperature", true, startup);
        publish_hipnuc_ = declare_parameter<bool>("publish_hipnuc", true, startup);
        imu_assembly_timeout_ms_ = declare_parameter<int>("imu_assembly_timeout_ms", 3, startup);
        imu_required_valid_ = imu_required_mask(declare_parameter<std::vector<std::string>>(
            "imu_required_fields", std::vector<std::string>{"acc", "gyr"}, startup));
        if (interface_.empty() || node_id_ < 0 || node_id_ > 255 || frame_id_.empty() || imu_assembly_timeout_ms_ <= 0)
            throw std::invalid_argument(
                "interface/frame_id must be nonempty, node_id in 0..255, and imu_assembly_timeout_ms positive");
        rclcpp::PublisherOptions publisher_options;
        // Let a deployment relax reliability or depth without rebuilding, e.g.
        // qos_overrides./imu/data.publisher.reliability:=best_effort.
        publisher_options.qos_overriding_options =
            rclcpp::QosOverridingOptions::with_default_policies();
        if (publish_imu_)
            imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 100, publisher_options);
        if (publish_mag_)
            mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100, publisher_options);
        if (publish_temperature_)
            temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10, publisher_options);
        if (publish_hipnuc_)
            full_pub_ = create_publisher<hipnuc_msgs::msg::HipnucImu>("hipnuc/imu", 100, publisher_options);
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        RCLCPP_INFO(get_logger(), "Requires device ENU output configuration; the driver does not verify or change it.");
    }

    void run()
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(shared_from_this());
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
                    clear_imu_cache();
                    reset_imu_field_watches(current);
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
                    RCLCPP_ERROR(get_logger(), "%s read failed: %s; reopening", interface_.c_str(), can_.last_error());
                    can_.close();
                    received_sample_ = false;
                    clear_imu_cache();
                    next_retry = SteadyClock::now() + std::chrono::seconds(1);
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            // Failure and idle paths must still service ROS and diagnostics.
            executor.spin_some();
            current = SteadyClock::now();
            const double elapsed = std::chrono::duration<double>(current - last_diag).count();
            if (elapsed >= 1.0) {
                publish_diagnostics(current, elapsed);
                last_diag = current;
            }
        }
    }

private:
    struct ImuCache {
        hipnuc_sample_t sample{};
        SteadyClock::time_point started_at{};
        bool active = false;
    };

    struct ImuFieldWatch {
        SteadyClock::time_point last_received{};
        bool seen = false;
        bool missing_reported = false;
    };

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
        const auto received_at = SteadyClock::now();
        ++frames_;
        last_frame_ = received_at;
        received_sample_ = true;
        note_imu_fields(s, received_at);
        update_and_publish_imu(s, stamp, received_at);
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

    // Classic J1939 splits one IMU epoch across separate ACC, GYR and QUAT
    // PGNs.  Retain the newest value of each; a fast PGN may overwrite its
    // cache several times before the slowest PGN completes the next sample.
    void update_and_publish_imu(const hipnuc_sample_t &s, const rclcpp::Time &stamp,
                                SteadyClock::time_point received_at)
    {
        if (!publish_imu_) return;

        const bool has_acc = (s.valid & HIPNUC_VALID_ACC) && hipnuc_ros::finite_vector(s.acc);
        const bool has_gyr = (s.valid & HIPNUC_VALID_GYR) && hipnuc_ros::finite_vector(s.gyr);
        const bool has_quat = hipnuc_ros::quaternion_norm(s) > 0.0;
        const uint64_t received_valid = (has_acc ? HIPNUC_VALID_ACC : 0) |
                                        (has_gyr ? HIPNUC_VALID_GYR : 0) |
                                        (has_quat ? HIPNUC_VALID_QUAT : 0);
        if ((received_valid & imu_required_valid_) == 0) return;

        // A J1939 PGN has no measurement sequence number. Treat the first
        // field as the start of one assembly window. Once that window has
        // expired, discard every partial field and let this incoming field
        // begin the next window; it must never be paired with old values.
        if (imu_cache_.active && imu_cache_timed_out(received_at)) clear_imu_cache();
        if (!imu_cache_.active) {
            imu_cache_.active = true;
            imu_cache_.started_at = received_at;
        }

        if (has_acc && (imu_required_valid_ & HIPNUC_VALID_ACC)) {
            std::copy(s.acc, s.acc + 3, imu_cache_.sample.acc);
            imu_cache_.sample.valid |= HIPNUC_VALID_ACC;
        }
        if (has_gyr && (imu_required_valid_ & HIPNUC_VALID_GYR)) {
            std::copy(s.gyr, s.gyr + 3, imu_cache_.sample.gyr);
            imu_cache_.sample.valid |= HIPNUC_VALID_GYR;
        }
        if (has_quat && (imu_required_valid_ & HIPNUC_VALID_QUAT)) {
            std::copy(s.quat, s.quat + 4, imu_cache_.sample.quat);
            imu_cache_.sample.valid |= HIPNUC_VALID_QUAT;
        }

        // Clearing the entire cache after each publish means every component
        // must arrive again, so the rate follows the slowest PGN.
        if ((imu_cache_.sample.valid & imu_required_valid_) != imu_required_valid_) {
            return;
        }

        sensor_msgs::msg::Imu m;
        m.header.stamp = stamp;
        m.header.frame_id = frame_id_;
        hipnuc_ros::fill_imu(imu_cache_.sample, m);
        imu_pub_->publish(m);
        clear_imu_cache();
    }

    bool imu_cache_timed_out(SteadyClock::time_point received_at) const
    {
        return received_at - imu_cache_.started_at > std::chrono::milliseconds(imu_assembly_timeout_ms_);
    }

    static uint64_t imu_required_mask(const std::vector<std::string> &fields)
    {
        uint64_t mask = 0;
        for (const auto &field : fields) {
            uint64_t bit = 0;
            if (field == "acc") bit = HIPNUC_VALID_ACC;
            else if (field == "gyr") bit = HIPNUC_VALID_GYR;
            else if (field == "quat") bit = HIPNUC_VALID_QUAT;
            else throw std::invalid_argument("imu_required_fields entries must be acc, gyr, or quat");
            if (mask & bit) throw std::invalid_argument("imu_required_fields must not contain duplicates");
            mask |= bit;
        }
        if (mask == 0) throw std::invalid_argument("imu_required_fields must not be empty");
        return mask;
    }

    void clear_imu_cache()
    {
        imu_cache_ = ImuCache{};
    }

    void reset_imu_field_watches(SteadyClock::time_point started_at)
    {
        imu_watch_started_at_ = started_at;
        acc_watch_ = ImuFieldWatch{};
        gyr_watch_ = ImuFieldWatch{};
        quat_watch_ = ImuFieldWatch{};
    }

    void note_imu_fields(const hipnuc_sample_t &s, SteadyClock::time_point received_at)
    {
        auto note = [&](uint64_t bit, bool received, ImuFieldWatch &watch) {
            if ((imu_required_valid_ & bit) && received) {
                watch.last_received = received_at;
                watch.seen = true;
                watch.missing_reported = false;
            }
        };
        note(HIPNUC_VALID_ACC, (s.valid & HIPNUC_VALID_ACC) && hipnuc_ros::finite_vector(s.acc), acc_watch_);
        note(HIPNUC_VALID_GYR, (s.valid & HIPNUC_VALID_GYR) && hipnuc_ros::finite_vector(s.gyr), gyr_watch_);
        note(HIPNUC_VALID_QUAT, hipnuc_ros::quaternion_norm(s) > 0.0, quat_watch_);
    }

    void warn_if_imu_field_missing(const char *name, uint64_t bit, ImuFieldWatch &watch,
                                   SteadyClock::time_point current)
    {
        if (!(imu_required_valid_ & bit)) return;
        const auto last = watch.seen ? watch.last_received : imu_watch_started_at_;
        if (current - last < std::chrono::seconds(3) || watch.missing_reported) return;
        RCLCPP_WARN(get_logger(), "required IMU field '%s' has not been received for 3 seconds; check device data output",
                    name);
        watch.missing_reported = true;
    }

    void warn_if_required_imu_fields_missing(SteadyClock::time_point current)
    {
        if (!publish_imu_ || !can_.is_open() || imu_watch_started_at_ == SteadyClock::time_point{}) return;
        warn_if_imu_field_missing("acc", HIPNUC_VALID_ACC, acc_watch_, current);
        warn_if_imu_field_missing("gyr", HIPNUC_VALID_GYR, gyr_watch_, current);
        warn_if_imu_field_missing("quat", HIPNUC_VALID_QUAT, quat_watch_, current);
    }

    void publish_diagnostics(SteadyClock::time_point current, double elapsed)
    {
        diagnostic_msgs::msg::DiagnosticArray arr;
        diagnostic_msgs::msg::DiagnosticStatus st;
        arr.header.stamp = now();
        st.name = std::string(get_fully_qualified_name()) + ": can";
        st.hardware_id = interface_;
        const double age = received_sample_ ? std::chrono::duration<double>(current - last_frame_).count() : -1.0;
        if (!can_.is_open()) {
            st.level = st.ERROR;
            st.message = "interface not open";
        } else if (age < 0.0 || age > 2.0) {
            st.level = st.WARN;
            st.message = other_nodes_ > other_nodes_at_last_diag_ ? "frames from other source addresses (check node_id)"
                                                                  : "no frames (check bitrate and wiring)";
        } else {
            st.level = st.OK;
            st.message = "receiving";
        }
        // Nothing else reaches the console after a successful open: report each change.
        if (st.level != last_level_) {
            last_level_ = st.level;
            if (st.level == st.OK) RCLCPP_INFO(get_logger(), "%s", st.message.c_str());
            else RCLCPP_WARN(get_logger(), "%s", st.message.c_str());
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
        other_nodes_at_last_diag_ = other_nodes_;
        arr.status.push_back(st);
        diag_pub_->publish(arr);
        warn_if_required_imu_fields_missing(current);
    }

    std::string interface_, frame_id_;
    int node_id_, imu_assembly_timeout_ms_;
    uint64_t imu_required_valid_ = 0;
    bool publish_imu_, publish_mag_, publish_temperature_, publish_hipnuc_;
    ImuCache imu_cache_;
    SteadyClock::time_point imu_watch_started_at_{};
    ImuFieldWatch acc_watch_, gyr_watch_, quat_watch_;
    hipnuc_ros::SocketCan can_;
    uint64_t frames_ = 0, frames_at_last_diag_ = 0;
    uint64_t invalid_ = 0, other_nodes_ = 0, other_nodes_at_last_diag_ = 0;
    bool received_sample_ = false;
    int last_level_ = -1;
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
