//subscriber imu specifcation data package
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>


int main(int argc, char **argv)
{
	ros::init(argc,argv,"sub_spec");
	ros::NodeHandle n;
	execlp("rostopic", "rostopic", "echo", "/IMU_data",NULL);

	ros::spin();

}
