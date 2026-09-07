// HiPNUC CAN (J1939 / CANFD83) driver node (ROS 1). Frames from the
// configured source address are merged into one sample, published when the
// trigger PGN arrives and then cleared, so fields never outlive one cycle.

#include <cstring>
#include <string>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <hipnuc_imu/HipnucImu.h>

#include "hipnuc_j1939.h"
#include "hipnuc_convert.hpp"
#include "socketcan.hpp"

class CanNode {
public:
    CanNode() : pnh_("~")
    {
        pnh_.param<std::string>("interface", interface_, "can0");
        pnh_.param("node_id", node_id_, static_cast<int>(HIPNUC_J1939_DEFAULT_NODE));
        pnh_.param("trigger_pgn", trigger_pgn_, static_cast<int>(HIPNUC_J1939_PGN_YAW));
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
        pnh_.param<std::string>("gnss_frame_id", gnss_frame_id_, "gnss_antenna");
        pnh_.param<std::string>("enu_frame_id", enu_frame_id_, "enu");
        pnh_.param("publish_hipnuc", publish_full_, true);

        ros::NodeHandle nh;
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
        mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
        temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
        fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gnss/fix", 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("ins/velocity", 10);
        full_pub_ = nh.advertise<hipnuc_imu::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
        hipnuc_sample_clear(&merged_);
    }

    void run()
    {
        ros::Time last_diag = ros::Time::now();
        while (ros::ok()) {
            if (!can_.is_open()) {
                std::string err = can_.open(interface_);
                if (!err.empty()) {
                    ROS_WARN_THROTTLE(5.0, "cannot open %s: %s (is the interface up?)", interface_.c_str(), err.c_str());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                ROS_INFO("listening on %s for node %d", interface_.c_str(), node_id_);
            }
            hipnuc_can_frame_t frame;
            int r = can_.read(frame, 100);
            if (r < 0) {
                ROS_ERROR("%s read failed; reopening", interface_.c_str());
                can_.close();
                continue;
            }
            if (r > 0) handle(frame);
            ros::spinOnce();
            if ((ros::Time::now() - last_diag).toSec() > 1.0) {
                publish_diagnostics();
                last_diag = ros::Time::now();
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
        last_frame_ = ros::Time::now();
        if (type == HIPNUC_J1939_MSG_CANFD83) {
            publish(part);
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
        const ros::Time stamp = ros::Time::now();
        if (s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_QUAT)) {
            sensor_msgs::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_.publish(m);
        }
        if (s.valid & HIPNUC_VALID_MAG) {
            sensor_msgs::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_.publish(m);
        }
        if (s.valid & HIPNUC_VALID_TEMPERATURE) {
            sensor_msgs::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_.publish(m);
        }
        if (s.valid & HIPNUC_VALID_POSITION) {
            sensor_msgs::NavSatFix m;
            m.header.stamp = stamp;
            m.header.frame_id = gnss_frame_id_;
            hipnuc_ros::fill_navsatfix(s, m);
            fix_pub_.publish(m);
        }
        if (s.valid & HIPNUC_VALID_VELOCITY_ENU) {
            geometry_msgs::TwistWithCovarianceStamped m;
            m.header.stamp = stamp;
            m.header.frame_id = enu_frame_id_;
            hipnuc_ros::fill_velocity_enu(s, m);
            vel_pub_.publish(m);
        }
        if (publish_full_) {
            hipnuc_imu::HipnucImu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_hipnuc(s, m);
            full_pub_.publish(m);
        }
    }

    void publish_diagnostics()
    {
        diagnostic_msgs::DiagnosticArray arr;
        diagnostic_msgs::DiagnosticStatus st;
        arr.header.stamp = ros::Time::now();
        st.name = ros::this_node::getName() + ": can";
        st.hardware_id = interface_;
        const double age = last_frame_.isZero() ? -1.0 : (ros::Time::now() - last_frame_).toSec();
        if (!can_.is_open()) { st.level = st.ERROR; st.message = "interface not open"; }
        else if (age < 0 || age > 2.0) { st.level = st.WARN; st.message = other_nodes_ ? "no frames from the configured node id" : "no frames"; }
        else { st.level = st.OK; st.message = "receiving"; }
        auto kv = [&](const char *k, const std::string &v) {
            diagnostic_msgs::KeyValue e; e.key = k; e.value = v; st.values.push_back(e);
        };
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string(frames_ - frames_at_last_diag_));
        kv("invalid_frames", std::to_string(invalid_));
        kv("frames_from_other_nodes", std::to_string(other_nodes_));
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_.publish(arr);
    }

    ros::NodeHandle pnh_;
    std::string interface_, frame_id_, gnss_frame_id_, enu_frame_id_;
    int node_id_, trigger_pgn_;
    bool publish_full_;
    hipnuc_ros::SocketCan can_;
    hipnuc_sample_t merged_;
    uint64_t frames_ = 0, frames_at_last_diag_ = 0, invalid_ = 0, other_nodes_ = 0;
    ros::Time last_frame_;
    ros::Publisher imu_pub_, mag_pub_, temp_pub_, fix_pub_, vel_pub_, full_pub_, diag_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hipnuc_can");
    CanNode node;
    node.run();
    return 0;
}
