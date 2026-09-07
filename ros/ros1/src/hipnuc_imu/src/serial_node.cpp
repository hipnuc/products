// HiPNUC serial driver node (ROS 1). Reads HI91/HI81/HI83 binary frames and
// GGA/RMC sentences from a serial port and publishes standard sensor
// messages plus the full-field hipnuc_imu/HipnucImu.

#include <cstring>
#include <string>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <hipnuc_imu/HipnucImu.h>

#include "hipnuc_dec.h"
#include "hipnuc_sample.h"
#include "nmea_dec.h"
#include "hipnuc_convert.hpp"
#include "posix_serial.hpp"

class SerialNode {
public:
    SerialNode() : pnh_("~")
    {
        pnh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        pnh_.param("baudrate", baudrate_, 115200);
        pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
        pnh_.param<std::string>("gnss_frame_id", gnss_frame_id_, "gnss_antenna");
        pnh_.param<std::string>("enu_frame_id", enu_frame_id_, "enu");
        pnh_.param("publish_imu", publish_imu_, true);
        pnh_.param("publish_mag", publish_mag_, true);
        pnh_.param("publish_temperature_pressure", publish_env_, true);
        pnh_.param("publish_navsatfix", publish_fix_, true);
        pnh_.param("publish_velocity", publish_vel_, true);
        pnh_.param("publish_hipnuc", publish_full_, true);

        ros::NodeHandle nh;
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
        mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
        temp_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 10);
        press_pub_ = nh.advertise<sensor_msgs::FluidPressure>("imu/pressure", 10);
        fix_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gnss/fix", 10);
        vel_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("ins/velocity", 10);
        full_pub_ = nh.advertise<hipnuc_imu::HipnucImu>("hipnuc/imu", 100);
        diag_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

        std::memset(&raw_, 0, sizeof(raw_));
        std::memset(&nmea_, 0, sizeof(nmea_));
    }

    void run()
    {
        uint8_t buf[512];
        ros::Time last_diag = ros::Time::now();
        while (ros::ok()) {
            if (!serial_.is_open()) {
                std::string err = serial_.open(port_, baudrate_);
                if (!err.empty()) {
                    ROS_WARN_THROTTLE(5.0, "cannot open %s: %s (check the cable, the dialout group and the port name)",
                                      port_.c_str(), err.c_str());
                    ros::Duration(1.0).sleep();
                    continue;
                }
                ROS_INFO("opened %s at %d baud", port_.c_str(), baudrate_);
                std::memset(&raw_, 0, sizeof(raw_));
                std::memset(&nmea_, 0, sizeof(nmea_));
            }
            int n = serial_.read(buf, sizeof(buf), 100);
            if (n < 0) {
                ROS_ERROR("%s disconnected; reopening", port_.c_str());
                serial_.close();
                continue;
            }
            for (int i = 0; i < n; ++i) feed(buf[i]);
            bytes_ += n;
            ros::spinOnce();
            if ((ros::Time::now() - last_diag).toSec() > 1.0) {
                publish_diagnostics();
                last_diag = ros::Time::now();
            }
        }
    }

private:
    void feed(uint8_t byte)
    {
        hipnuc_sample_t s;
        if (hipnuc_input(&raw_, byte) > 0 && hipnuc_sample_from_raw(&raw_, &s)) publish(s);
        if (nmea_input(&nmea_, byte) > 0 && hipnuc_sample_from_nmea(&nmea_, &s)) publish(s);
    }

    void publish(const hipnuc_sample_t &s)
    {
        const ros::Time stamp = ros::Time::now();
        frames_++;
        last_frame_ = stamp;
        if (publish_imu_ && (s.valid & (HIPNUC_VALID_ACC | HIPNUC_VALID_GYR | HIPNUC_VALID_QUAT))) {
            sensor_msgs::Imu m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_imu(s, m);
            imu_pub_.publish(m);
        }
        if (publish_mag_ && (s.valid & HIPNUC_VALID_MAG)) {
            sensor_msgs::MagneticField m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_mag(s, m);
            mag_pub_.publish(m);
        }
        if (publish_env_ && (s.valid & HIPNUC_VALID_TEMPERATURE)) {
            sensor_msgs::Temperature m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_temperature(s, m);
            temp_pub_.publish(m);
        }
        if (publish_env_ && (s.valid & HIPNUC_VALID_PRESSURE)) {
            sensor_msgs::FluidPressure m;
            m.header.stamp = stamp;
            m.header.frame_id = frame_id_;
            hipnuc_ros::fill_pressure(s, m);
            press_pub_.publish(m);
        }
        if (publish_fix_ && (s.valid & HIPNUC_VALID_POSITION)) {
            sensor_msgs::NavSatFix m;
            m.header.stamp = stamp;
            m.header.frame_id = gnss_frame_id_;
            hipnuc_ros::fill_navsatfix(s, m);
            fix_pub_.publish(m);
        }
        if (publish_vel_ && (s.valid & HIPNUC_VALID_VELOCITY_ENU)) {
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
        st.name = ros::this_node::getName() + ": serial";
        st.hardware_id = port_;
        const double age = last_frame_.isZero() ? -1.0 : (ros::Time::now() - last_frame_).toSec();
        if (!serial_.is_open()) { st.level = st.ERROR; st.message = "port not open"; }
        else if (age < 0 || age > 2.0) { st.level = st.WARN; st.message = bytes_ ? "no valid frames (baudrate?)" : "no data"; }
        else { st.level = st.OK; st.message = "receiving"; }
        auto kv = [&](const char *k, const std::string &v) {
            diagnostic_msgs::KeyValue e; e.key = k; e.value = v; st.values.push_back(e);
        };
        kv("bytes", std::to_string(bytes_));
        kv("frames", std::to_string(frames_));
        kv("frame_rate_hz", std::to_string(frames_ - frames_at_last_diag_));
        kv("crc_errors", std::to_string(raw_.crc_error_count));
        kv("invalid_frames", std::to_string(raw_.invalid_count));
        kv("nmea_checksum_errors", std::to_string(nmea_.checksum_error_count));
        frames_at_last_diag_ = frames_;
        arr.status.push_back(st);
        diag_pub_.publish(arr);
    }

    ros::NodeHandle pnh_;
    std::string port_, frame_id_, gnss_frame_id_, enu_frame_id_;
    int baudrate_;
    bool publish_imu_, publish_mag_, publish_env_, publish_fix_, publish_vel_, publish_full_;
    hipnuc_ros::PosixSerial serial_;
    hipnuc_raw_t raw_;
    nmea_raw_t nmea_;
    uint64_t bytes_ = 0, frames_ = 0, frames_at_last_diag_ = 0;
    ros::Time last_frame_;
    ros::Publisher imu_pub_, mag_pub_, temp_pub_, press_pub_, fix_pub_, vel_pub_, full_pub_, diag_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hipnuc_serial");
    SerialNode node;
    node.run();
    return 0;
}
