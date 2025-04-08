#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

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

using namespace std::chrono_literals;
using namespace std;
static hipnuc_raw_t raw;

class IMUPublisher : public rclcpp::Node
{
	public:
		int fd = 0;
		uint8_t buf[BUF_SIZE] = {0};
		IMUPublisher() : Node("IMU_publisher")	
		{
			this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
			this->declare_parameter<int>("baud_rate", 460800);
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
			
			imu_data.header.frame_id = frame_id;
			imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 20);

			fd = open_serial(serial_port, baud_rate);

			while(1)
				imu_read();
		}

	private: 
		void imu_read(void)
		{
			struct pollfd p;
			p.fd = fd;
			p.events = POLLIN;
			
			int rpoll = poll(&p, 1, 5);

			if(rpoll == 0)
				return ;

			int n = read(fd, buf, sizeof(buf));

			for(int i = 0; i < n; i++)
			{
				int rev = hipnuc_input(&raw, buf[i]);
				
				if(rev)
				{
					imu_data.orientation.w = raw.hi91.quat[0];
					imu_data.orientation.x = raw.hi91.quat[1];	
					imu_data.orientation.y = raw.hi91.quat[2];
					imu_data.orientation.z = raw.hi91.quat[3];
					imu_data.angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
					imu_data.angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
					imu_data.angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
					imu_data.linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
					imu_data.linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
					imu_data.linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;

					imu_data.header.stamp = rclcpp::Clock().now();
					
					imu_pub->publish(imu_data);
				}
			}

			memset(buf,0,sizeof(buf));
		}

		int open_serial(std::string port, int baud)
		{
			const char* port_device = port.c_str();
			int fd = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
		
			if(fd == -1)
			{
				perror("unable to open serial port");
				exit(0);
			}
			
			if(fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
				cout << "fcntl failed" << "\n" << endl;
			else
				fcntl(fd, F_SETFL, O_NONBLOCK);

			struct termios options;
			memset(&options, 0, sizeof(options));
			tcgetattr(fd, &options);
	
			switch(baud)
			{
				case 115200:
					cfsetispeed(&options, B115200);
					cfsetospeed(&options, B115200);
				break;
				case 460800:
					cfsetispeed(&options, B460800);
					cfsetospeed(&options, B460800);
				break;
				case 921600:
					cfsetispeed(&options, B921600);
					cfsetospeed(&options, B921600);
				break;
			}

			options.c_cflag &= ~PARENB; 
			options.c_cflag &= ~CSTOPB; 
			options.c_cflag &= ~CSIZE;  
			options.c_cflag |= HUPCL;   
			options.c_cflag |= CS8;     
			options.c_cflag &= ~CRTSCTS; 
			options.c_cflag |= CREAD | CLOCAL; 

			options.c_iflag &= ~(IXON | IXOFF | IXANY); 
			options.c_iflag &= ~(INLCR|ICRNL); 

			options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

			options.c_oflag &= ~OPOST; 
			options.c_oflag &= ~(ONLCR|OCRNL); 

			options.c_cc[VMIN] = 0;  
			options.c_cc[VTIME] = 0; 
			tcsetattr(fd, TCSANOW, &options);

			return fd;
		}


		std::string serial_port;
		int baud_rate;
		std::string frame_id;
		std::string imu_topic;
		sensor_msgs::msg::Imu imu_data = sensor_msgs::msg::Imu();
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
};


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUPublisher>());
	rclcpp::shutdown();

	return 0;
}
