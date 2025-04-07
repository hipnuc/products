#include <ros/ros.h>
#include "imu_pub.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try
    {
        ROS_INFO("READ IMU START");
        SerialIMUNode node(nh, pnh);
        ros::Duration(1).sleep();
        node.open();

        node.run();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Node crashed: %s", e.what());
        return 1;
    }
    
    return 0;
}



