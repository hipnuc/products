#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
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
#include "hipnuc_lib_package/nmea_decode.h"

#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)

#define ACC_FACTOR   (0.0048828)
#define GYR_FACTOR	 (0.001)
#define EUL_FACTOR	 (0.001)
#define QUA_FACTOR   (0.0001)
#define NAV_FACTOR   (0.0000001)
#define MSL_FACTOR   (0.001)

#define BUF_SIZE    (1024)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static hipnuc_raw_t raw;
static nmea_raw_t raw_gnss;

void publish_ins_data(hipnuc_raw_t *data, sensor_msgs::msg::Imu *imu_data, sensor_msgs::msg::NavSatFix *NavSatFix_data);
void publish_gps_data(nmea_raw_t *data, sensor_msgs::msg::NavSatFix *NavSatFix_data);

class INSPublisher : public rclcpp::Node
{
	public:
		int fd = 0;
		uint8_t buf[BUF_SIZE] = {0};
		INSPublisher() : Node("INS_publisher")	
		{
			this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
			this->declare_parameter<int>("baud_rate", 115200);
			this->declare_parameter<std::string>("frame_id", "base_link");
			this->declare_parameter<std::string>("imu_topic", "/IMU_data");
			this->declare_parameter<std::string>("nav_topic", "/NavSatFix_data");

			this->get_parameter("serial_port", serial_port);
			this->get_parameter("baud_rate", baud_rate);
			this->get_parameter("frame_id", frame_id);
			this->get_parameter("imu_topic", imu_topic);
			this->get_parameter("nav_topic", nav_topic);

			RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
			RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
			RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
			RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
			RCLCPP_INFO(this->get_logger(), "nav_topic: %s\r\n", nav_topic.c_str());
			
			imu_data.header.frame_id = frame_id;
			imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);

			nav_data.header.frame_id = frame_id;
			nav_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(nav_topic, 10);

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
				// int rev = hipnuc_input(&raw, buf[i]); /* HI81 */
				int rev = input_nmea(&raw_gnss, buf[i]); /* GGA RMC SXT */
				
				if(rev)
				{
					/* publish NavSatFix data */
					publish_gps_data(&raw_gnss, &nav_data);
					
					nav_data.header.stamp = rclcpp::Clock().now();

					nav_pub->publish(nav_data);

					/* publish imu data */
					// publish_ins_data(&raw, &imu_data, &nav_data);
					// imu_data.header.stamp = rclcpp::Clock().now();
					// imu_pub->publish(imu_data);
					
					rev = 0;
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


		int baud_rate;
		std::string serial_port;
		std::string frame_id;
		std::string imu_topic, nav_topic, gps_topic, gst_topic;

		sensor_msgs::msg::Imu imu_data = sensor_msgs::msg::Imu();
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

		sensor_msgs::msg::NavSatFix nav_data = sensor_msgs::msg::NavSatFix();
		rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_pub;

};


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<INSPublisher>());
	rclcpp::shutdown();

	return 0;
}



void publish_ins_data(hipnuc_raw_t *data, sensor_msgs::msg::Imu *imu_data, sensor_msgs::msg::NavSatFix *NavSatFix_data)
{
	if(data->hi81.tag == 0x81)
	{
		/* IMU */
		imu_data->orientation.x = (float)data->hi81.quat[1] * QUA_FACTOR;
		imu_data->orientation.y = (float)data->hi81.quat[2] * QUA_FACTOR;
		imu_data->orientation.z = (float)data->hi81.quat[3] * QUA_FACTOR;
		imu_data->orientation.w = (float)data->hi81.quat[0] * QUA_FACTOR;
		imu_data->angular_velocity.x = (float)data->hi81.gyr_b[0] * GYR_FACTOR;
		imu_data->angular_velocity.y = (float)data->hi81.gyr_b[1] * GYR_FACTOR;
		imu_data->angular_velocity.z = (float)data->hi81.gyr_b[2] * GYR_FACTOR;
		imu_data->linear_acceleration.x = (float)data->hi81.acc_b[0] * ACC_FACTOR;
		imu_data->linear_acceleration.y = (float)data->hi81.acc_b[1] * ACC_FACTOR;
		imu_data->linear_acceleration.z = (float)data->hi81.acc_b[2] * ACC_FACTOR;

		/* NavSatFix */
		NavSatFix_data->latitude = data->hi81.ins_lat * NAV_FACTOR;
		NavSatFix_data->longitude = data->hi81.ins_lon * NAV_FACTOR;
		NavSatFix_data->altitude = data->hi81.ins_msl * MSL_FACTOR;
		NavSatFix_data->status.status = data->hi81.solq_pos;

	}
}

void publish_imu_data(hipnuc_raw_t *data, sensor_msgs::msg::Imu *imu_data)
{
	if(data->hi91.tag == 0x91)
	{
		imu_data->orientation.x = data->hi91.quat[1];
		imu_data->orientation.y = data->hi91.quat[2];
		imu_data->orientation.z = data->hi91.quat[3];
		imu_data->orientation.w = data->hi91.quat[0];
		imu_data->angular_velocity.x = data->hi91.gyr[0] * DEG_TO_RAD;
		imu_data->angular_velocity.y = data->hi91.gyr[1] * DEG_TO_RAD;
		imu_data->angular_velocity.z = data->hi91.gyr[2] * DEG_TO_RAD;
		imu_data->linear_acceleration.x = data->hi91.acc[0] * GRA_ACC;
		imu_data->linear_acceleration.y = data->hi91.acc[1] * GRA_ACC;
		imu_data->linear_acceleration.z = data->hi91.acc[2] * GRA_ACC;
	}  
	if(data->hi92.tag == 0x92)
	{
		imu_data->orientation.x = (float)data->hi92.quat[1] * QUA_FACTOR;
		imu_data->orientation.y = (float)data->hi92.quat[2] * QUA_FACTOR;
		imu_data->orientation.z = (float)data->hi92.quat[3] * QUA_FACTOR;
		imu_data->orientation.w = (float)data->hi92.quat[0] * QUA_FACTOR;
		imu_data->angular_velocity.x = (float)data->hi92.gyr_b[0] * GYR_FACTOR;
		imu_data->angular_velocity.y = (float)data->hi92.gyr_b[1] * GYR_FACTOR;
		imu_data->angular_velocity.z = (float)data->hi92.gyr_b[2] * GYR_FACTOR;
		imu_data->linear_acceleration.x = (float)data->hi92.acc_b[0] * ACC_FACTOR;
		imu_data->linear_acceleration.y = (float)data->hi92.acc_b[1] * ACC_FACTOR;
		imu_data->linear_acceleration.z = (float)data->hi92.acc_b[2] * ACC_FACTOR;
	}
}

void publish_gps_data(nmea_raw_t *data, sensor_msgs::msg::NavSatFix *NavSatFix_data)
{
	if (!strncmp(data->type, "GGA", 3))
    {
		NavSatFix_data->latitude = data->gga.lat;
		NavSatFix_data->longitude = data->gga.lon;
		NavSatFix_data->altitude = data->gga.alt;
		NavSatFix_data->status.status = data->gga.status;
    }
    else if (!strncmp(data->type, "RMC", 3))
    {
        NavSatFix_data->latitude = data->rmc.lat;
		NavSatFix_data->longitude = data->rmc.lon;
	}
	else if (!strncmp(data->type, "SXT", 3))
    {
        NavSatFix_data->latitude = data->sxt.lat;
		NavSatFix_data->longitude = data->sxt.lon;
		NavSatFix_data->altitude = data->sxt.alt;
		NavSatFix_data->status.status = data->sxt.solq;
    }
}
