#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>               //ACC GYR QUAT
#include <sensor_msgs/MagneticField.h>     //MAG
#include <sensor_msgs/FluidPressure.h>     //PR
#include <sensor_msgs/Temperature.h>       //TEMP
#include <geometry_msgs/Vector3Stamped.h>         //EUL

#include <sys/time.h>
#include <signal.h>
#include "serial_port.h"
#include "hipnuc_lib_package/hipnuc_dec.h"

class SerialIMUNode
{
    public:
    SerialIMUNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SerialIMUNode();

    void run();
    void open();
    void close();

    private:
    static void signal_handler(int sig);
    void alarm_handler(int sig);
    void loadParameters();
    void initPublishers();
    void initSerialPort();
    void initAlarm();

    void publish_imu_data( hipnuc_raw_t *data, sensor_msgs::Imu              *imu_data);
    void publish_mag_data( hipnuc_raw_t *data, sensor_msgs::MagneticField    *mag_data);
    void publish_eul_data( hipnuc_raw_t *data, geometry_msgs::Vector3Stamped *eul_data);
    void publish_pre_data( hipnuc_raw_t *data, sensor_msgs::FluidPressure    *pre_data);
    void publish_temp_data(hipnuc_raw_t *data, sensor_msgs::Temperature      *temp_data);

    ros::NodeHandle nh_, pnh_;
    ros::Publisher imu_pub_, eul_pub_, mag_pub_, pre_pub_, temp_pub_;
 

    std::unique_ptr<SerialPort> serial_port_;
    
    std::string serial_port_name_;
    int baud_rate_;
    std::string frame_id_;
    std::string imu_topic_, eul_topic_, mag_topic_, pre_topic_, temp_topic_;
    std::string imu_axes;
    hipnuc_raw_t raw;
    
    const int MAX_RECONNECT_ATTEMPTS = 20;
    int reconnect_count = 0;

    int timeout;
    int timeout_sec;
    int get_port_timeout_ms = 0;
    struct itimerval timer;
    static SerialIMUNode* instance_;

    sensor_msgs::Imu imu_msg;
    sensor_msgs::Temperature temp_msg;
    sensor_msgs::FluidPressure pre_msg;
    sensor_msgs::MagneticField mag_msg;
    geometry_msgs::Vector3Stamped eul_msg;

    bool imu_enable, mag_enable, eul_enable, pre_enable, temp_enable;
};