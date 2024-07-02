//subscriber imu specifcation data package
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_gnss");
	ros::NodeHandle n;
	execlp("rostopic", "rostopic", "echo", "/NavSatFix_data",NULL);

	ros::spin();
}

