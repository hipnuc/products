#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"{
#endif
#include <poll.h>

#include "hipnuc_lib_package/hipnuc_dec.h"

#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)
#define BUF_SIZE    (1024)
#ifdef __cplusplus
}
#endif



namespace hipnuc_driver
{
	using namespace std::chrono_literals;
	using namespace std;
	static hipnuc_raw_t raw;

	class IMUPublisher : public rclcpp::Node
	{
		public:
			int fd = 0;
			uint8_t buf[BUF_SIZE] = {0};
			IMUPublisher(const rclcpp::NodeOptions &options) : Node("IMU_publisher", options)	
			{
				this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
				this->declare_parameter<int>("baud_rate", 115200);
				this->declare_parameter<std::string>("frame_id", "base_link");
				this->declare_parameter<std::string>("imu_topic", "/IMU_data");

				this->get_parameter("serial_port", serial_port);
				this->get_parameter("baud_rate", baud_rate);
				this->get_parameter("frame_id", frame_id);
				this->get_parameter("imu_topic", imu_topic);

				RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
				RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
				RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
				RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
				
				imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 20);

				fd = open_ttyport(serial_port, baud_rate);

				while(1)
				 	imu_read();
					
				close(fd);

			}

		private: 
			void imu_read(void)
			{
				struct pollfd p;
				p.fd = fd;
				p.events = POLLIN;
				
				int rpoll = poll(&p, 1, 1);

				if(rpoll == 0)
					return ;

				int n = read(fd, buf, sizeof(buf));

				for(int i = 0; i < n; i++)
				{
					int rev = hipnuc_input(&raw, buf[i]);
					
					if(rev)
					{
						auto imu_data = std::make_unique<sensor_msgs::msg::Imu>();
						imu_data->orientation.w = raw.hi91.quat[0];
						imu_data->orientation.x = raw.hi91.quat[1];	
						imu_data->orientation.y = raw.hi91.quat[2];
						imu_data->orientation.z = raw.hi91.quat[3];
						imu_data->angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
						imu_data->angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
						imu_data->angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
						imu_data->linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
						imu_data->linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
						imu_data->linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;

						imu_data->header.frame_id = frame_id;
						imu_data->header.stamp = rclcpp::Clock().now();
						
						imu_pub->publish(std::move(imu_data));
					}
				}

				memset(buf,0,sizeof(buf));
			}

			int open_ttyport(std::string tty_port, int baud)
			{
				const char *port_device = tty_port.c_str();
				int serial_port = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
				if (serial_port < 0)
				{
					perror("Error opening serial port");
					return -1;
				}

				struct termios2 tty;

				if (ioctl(serial_port, TCGETS2, &tty) != 0)
				{
					perror("Error from TCGETS2 ioctl");
					close(serial_port);
					return -1;
				}

				tty.c_cflag &= ~CBAUD;
				tty.c_cflag |= BOTHER;

				tty.c_ispeed = baud;
				tty.c_ospeed = baud;

				tty.c_cflag |= CS8;
				tty.c_cflag &= ~PARENB;
				tty.c_cflag &= ~CSTOPB;
				tty.c_cflag &= ~CRTSCTS;

				tty.c_lflag &= ~ICANON;
				tty.c_lflag &= ~ECHO;
				tty.c_lflag &= ~ECHOE;
				tty.c_lflag &= ~ECHONL;
				tty.c_lflag &= ~ISIG;

				tty.c_iflag &= ~(IXON | IXOFF | IXANY);
				tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

				tty.c_cc[VTIME] = 10;
				tty.c_cc[VMIN] = 0;

				if (ioctl(serial_port, TCSETS2, &tty) != 0)
				{
					perror("Error from TCSETS2 IOCTL");
					close(serial_port);
					return -1;
				}

				return serial_port;
			}

			std::string serial_port;
			int baud_rate;
			std::string frame_id;
			std::string imu_topic;

			rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
	};
};


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	rclcpp::spin(std::make_shared<hipnuc_driver::IMUPublisher>(options));
	rclcpp::shutdown();

	return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_driver::IMUPublisher)
