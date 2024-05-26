#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_imu");
	ros::NodeHandle nl;
	execlp("rostopic", "rostopic", "echo", "/rawimu_data",NULL);

	ros::spin();
}