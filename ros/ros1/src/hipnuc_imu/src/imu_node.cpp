// ROS 1 node for a serial HiPNUC device.
#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <hipnuc_imu/HipnucImu.h>

#include "convert.hpp"
#include "driver.hpp"

namespace hipnuc_imu {

class ImuNode : private Output {
public:
    ImuNode() : pnh_("~")
    {
        pnh_.param<std::string>("port", settings_.port, "/dev/ttyUSB0");
        pnh_.param<int>("baudrate", settings_.baudrate, 115200);
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
        bool publish_imu, publish_data_raw, publish_mag, publish_temperature, publish_hipnuc;
        pnh_.param<bool>("publish_imu", publish_imu, true);
        pnh_.param<bool>("publish_data_raw", publish_data_raw, false);
        pnh_.param<bool>("publish_mag", publish_mag, false);
        pnh_.param<bool>("publish_temperature", publish_temperature, false);
        pnh_.param<bool>("publish_hipnuc", publish_hipnuc, false);
        if (frame_id_.empty()) throw std::invalid_argument("frame_id must not be empty");
        if (settings_.port.empty() || settings_.baudrate <= 0)
            throw std::invalid_argument("port must not be empty and baudrate must be positive");

        ros::NodeHandle nh;
        if (publish_imu) imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
        if (publish_data_raw) raw_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 100);
        if (publish_mag) mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
        if (publish_temperature) temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
        if (publish_hipnuc) full_pub_ = nh.advertise<hipnuc_imu::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
        diag_name_ = ros::this_node::getName();
    }

    void spin()
    {
        // Serves the simulated clock while the driver loop runs here.
        ros::AsyncSpinner spinner(1);
        spinner.start();
        run(settings_, *this, [] { return ros::ok(); });
    }

private:
    // False while a simulated clock has not yet reached the measurement;
    // ros::Time cannot go below zero.
    static bool stamp(Clock::time_point at, ros::Time &out)
    {
        const ros::Time clock = ros::Time::now();
        const ros::Duration age(std::chrono::duration<double>(Clock::now() - at).count());
        if (clock.toSec() <= age.toSec()) return false;
        out = clock - age;
        return true;
    }

    void log(Level level, const std::string &text) override
    {
        if (level == Level::info) ROS_INFO("%s", text.c_str());
        else if (level == Level::warn) ROS_WARN("%s", text.c_str());
        else ROS_ERROR("%s", text.c_str());
    }

    void measurement(const hipnuc_sample_t &s, Clock::time_point at, bool imu) override
    {
        ros::Time time;
        if (!stamp(at, time)) return;
        if (imu && (imu_pub_ || raw_pub_)) {
            sensor_msgs::Imu m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_imu(s, m);
            if (imu_pub_) imu_pub_.publish(m);
            if (raw_pub_) {
                clear_orientation(m);
                raw_pub_.publish(m);
            }
        }
        if (mag_pub_ && (s.valid & HIPNUC_VALID_MAG) && finite_vector(s.mag)) {
            sensor_msgs::MagneticField m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_mag(s, m);
            mag_pub_.publish(m);
        }
        if (temp_pub_ && (s.valid & HIPNUC_VALID_TEMPERATURE) && std::isfinite(s.temperature)) {
            sensor_msgs::Temperature m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_temperature(s, m);
            temp_pub_.publish(m);
        }
        if (full_pub_) {
            hipnuc_imu::HipnucImu m;
            m.header.stamp = time;
            m.header.frame_id = frame_id_;
            fill_hipnuc(s, m);
            full_pub_.publish(m);
        }
    }

    void status(const Status &st) override
    {
        using diagnostic_msgs::DiagnosticStatus;
        diagnostic_msgs::DiagnosticArray array;
        DiagnosticStatus diag;
        array.header.stamp = ros::Time::now();
        diag.name = diag_name_;
        diag.hardware_id = settings_.port;
        diag.level = st.level == Level::info ? DiagnosticStatus::OK
                   : st.level == Level::warn ? DiagnosticStatus::WARN
                   : DiagnosticStatus::ERROR;
        diag.message = st.message;
        for (const auto &value : st.values) {
            diagnostic_msgs::KeyValue entry;
            entry.key = value.first;
            entry.value = value.second;
            diag.values.push_back(entry);
        }
        array.status.push_back(diag);
        diag_pub_.publish(array);
    }

    ros::NodeHandle pnh_;
    Settings settings_;
    std::string frame_id_, diag_name_;
    ros::Publisher imu_pub_, raw_pub_, mag_pub_, temp_pub_, full_pub_, diag_pub_;
};

}  // namespace hipnuc_imu

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hipnuc_imu");
    try {
        hipnuc_imu::ImuNode node;
        node.spin();
    } catch (const std::exception &error) {
        ROS_ERROR("%s", error.what());
        return 1;
    }
    return 0;
}
