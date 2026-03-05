#include <unistd.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <iomanip>

class canImuSubscriber : public rclcpp::Node
{
	public:
	canImuSubscriber() : Node("can_imu_subscriber")
	{
		this->declare_parameter<std::string>("sub_topic", "/imu/canopen_msg");
		std::string topic = this->get_parameter("sub_topic").as_string();

		imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
			topic,
			rclcpp::SensorDataQoS(),
			std::bind(&canImuSubscriber::topic_callback, this, std::placeholders::_1));
		RCLCPP_INFO(this->get_logger(), "IMU Subscriber started on %s", topic.c_str());
	}

	private:
	void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		msg_count++;
		if (msg_count % 100)
			return ;
		
		std::ostringstream oss;

		oss << "\n========================================\n";
		oss << "Message Count: " << msg_count;
		oss << "\n========================================\n";

		//header
		oss << "header: \n";
		oss << "	stamp: \n";
		oss << "		secs: " << msg->header.stamp.sec << "\n";
		oss << "		nanosecs: " << msg->header.stamp.nanosec << "\n";
		oss << "	frame_id: " << msg->header.frame_id << "\n";

		//Orientation
		oss << "\nOrientation: \n";
		oss << "x: " << std::fixed << std::setprecision(18) << msg->orientation.x << "\n";
		oss << "y: " << std::fixed << std::setprecision(18) << msg->orientation.y << "\n";
		oss << "z: " << std::fixed << std::setprecision(18) << msg->orientation.z << "\n";
		oss << "w: " << std::fixed << std::setprecision(18) << msg->orientation.w << "\n";

		oss << "orientation_covariance: [";
		for (int i = 0; i < 9; i++) {
			oss << std::fixed << std::setprecision(1) << msg->orientation_covariance[i];
			if (i < 8) oss << ", ";
		}
		oss << "]\n";

		//angular
		oss << "\nAngular_velocity:\n";
		oss << "	x: " << std::fixed << std::setprecision(18) << msg->angular_velocity.x << "\n";
		oss << "	y: " << std::fixed << std::setprecision(18) << msg->angular_velocity.y << "\n";
		oss << "	z: " << std::fixed << std::setprecision(18) << msg->angular_velocity.z << "\n";

		oss << "angular_velocity_covariance: [";
		for (int i = 0; i < 9; i++) {
			oss << std::fixed << std::setprecision(1) << msg->angular_velocity_covariance[i];
			if (i < 8) oss << ", ";
		}
		oss << "]\n";

		//Linear acceleration
		oss << "\nLinear acceleration\n";
		oss << "	x: " << std::fixed << std::setprecision(18) << msg->linear_acceleration.x << "\n";
		oss << "	y: " << std::fixed << std::setprecision(18) << msg->linear_acceleration.y << "\n";
		oss << "	z: " << std::fixed << std::setprecision(18) << msg->linear_acceleration.z << "\n";

		oss << "linear_acceleration_covariance: [";
		for (int i = 0; i < 9; i++) {
			oss << std::fixed << std::setprecision(1) << msg->linear_acceleration_covariance[i];
			if (i < 8) oss << ", ";
		}
		oss << "]\n";
		
		oss << "\n========================================\n";
		
		RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
	}

	private:
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	uint64_t msg_count{0};
	
};


int main(int argc,const char* argv[])
{
	#if 0
	rclcpp::init(argc, argv);
	nh = std::make_shared<rclcpp::Node>("imu_sub");
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub ;
	imu_sub = nh->create_subscription<sensor_msgs::msg::Imu>("/IMU_data", rclcpp::SensorDataQoS(), topic_callback);
	rclcpp::spin(nh);
	rclcpp::shutdown();
	#endif

	rclcpp::init(argc, argv);
	
	rclcpp::spin(std::make_shared<canImuSubscriber>());
	rclcpp::shutdown();

	return 0;
}
