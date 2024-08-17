#include <unistd.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iomanip>

rclcpp::Node::SharedPtr nh = nullptr;
using namespace std;

void nav_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	cout << "header:" << "\n";
	cout << "	" << "stamp:"<< "\n";
	cout << "	  " << "secs: " << msg->header.stamp.sec<< "\n";
	cout << "	  " << "nanosecs: " << msg->header.stamp.nanosec << "\n";
	cout << "	" << "frame_id: " << msg->header.frame_id << "\n" ;
	cout << "status:" << "\n";
	cout << "	" << "status: " << static_cast<int>(msg->status.status)<< "\n";
	cout << "	" << "service: " << fixed << setprecision(8) << msg->status.service << "\n";
	cout << "latitude: " << fixed << setprecision(8) << msg->latitude << "\n";
	cout << "longitude: " << fixed << setprecision(8) << msg->longitude << "\n";
	cout << "altitude: " << fixed << setprecision(8) << msg->altitude << "\n";
	cout  << "orientation_covariance: [ " << fixed << setprecision(1) << msg->position_covariance[0];
	cout  << ", " << msg->position_covariance[1];
	cout  << ", " << msg->position_covariance[2];
	cout  << ", " << msg->position_covariance[3];
	cout  << ", " << msg->position_covariance[4];
	cout  << ", " << msg->position_covariance[5];
	cout  << ", " << msg->position_covariance[6];
	cout  << ", " << msg->position_covariance[7];
	cout  << ", " << msg->position_covariance[8] << "]" << "\n";
	cout << "position_covariance_type: " << static_cast<int>(msg->position_covariance_type) << "\n" << "---" << endl;
}


int main(int argc,const char* argv[])
{
	rclcpp::init(argc, argv);
	nh = std::make_shared<rclcpp::Node>("sub_nav");
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
	gps_sub = nh->create_subscription<sensor_msgs::msg::NavSatFix>("NavSatFix_data", 10, nav_callback);
	rclcpp::spin(nh);
	rclcpp::shutdown();

	return 0;
}
