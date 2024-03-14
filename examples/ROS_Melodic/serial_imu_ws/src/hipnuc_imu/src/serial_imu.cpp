//serial_imu.cpp
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include "hipnuc.h"

#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024


#define ACC_FACTOR   (0.0048828)
#define GYR_FACTOR	 (0.001)
#define EUL_FACTOR	 (0.001)


void publish_imu_data(hipnuc_raw_t *data, sensor_msgs::Imu *imu_data);

#ifdef __cplusplus
}
#endif

tf2::Quaternion q;
geometry_msgs::Quaternion quaternion_msg;


int baud_rate;
std::string imu_serial;
std::string frame_id;
std::string imu_topic;
static hipnuc_raw_t raw;
ros::Publisher IMU_pub;
sensor_msgs::Imu imu_data;
int fd = 0;
static uint8_t buf[BUF_SIZE];

int n = 0;
int rev = 0;
struct pollfd p;

double time_sec;
int rpoll;

void read_imu(void)
{
	rpoll = poll(&p, 1, 5);

	if(rpoll == 0)
		return ;
	n = read(fd, buf, sizeof(buf));

	if(n > 0)
	{
		for(int i = 0; i < n; i++)
		{
			rev = hipnuc_input(&raw, buf[i]);
			if(rev)
			{
				imu_data.header.stamp = ros::Time::now();
				publish_imu_data(&raw, &imu_data);
				IMU_pub.publish(imu_data);
				rev = 0;
			}

		}
	}
}

int open_port(std::string port_device, int baud)
{
	const char* port_device1 = port_device.c_str();
	int fd = open(port_device1, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd == -1)
	{
		perror("open_port: Unable to open SerialPort");
		puts("Please check the usb port name!!!");
		exit(0);
	}

	if(fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
		printf("fcntl failed\n");
	else
		fcntl(fd, F_SETFL,  O_NONBLOCK);
  
	if(isatty(STDIN_FILENO) == 0)
		printf("standard input is not a terminal device\n");
	else 
		printf("isatty success!\n");

	struct termios options;
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
		default:
		ROS_ERROR("SERIAL PORT BAUD RATE ERROR");
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
	return (fd);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hipnuc_imu");
	ros::NodeHandle n;

	ros::param::param<std::string>("/imu_serial", imu_serial, "/dev/ttyUSB0");
	ros::param::param<int>("/baud_rate", baud_rate, 115200 );
	ros::param::param<std::string>("/frame_id", frame_id, "base_link");
	ros::param::param<std::string>("/imu_topic", imu_topic, "/IMU_data");

    IMU_pub = n.advertise<sensor_msgs::Imu>(imu_topic, 5);

	fd = open_port(imu_serial, baud_rate);
	
	imu_data.header.frame_id = frame_id;
	
	p.fd = fd;
	p.events = POLLIN;

	while(ros::ok())
	{
		read_imu();
	}
	
	return 0;
}

void publish_imu_data(hipnuc_raw_t *data, sensor_msgs::Imu *imu_data)
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
		q.setRPY(data->hi92.pitch * EUL_FACTOR * DEG_TO_RAD, data->hi92.roll * EUL_FACTOR * DEG_TO_RAD, data->hi92.yaw * EUL_FACTOR * DEG_TO_RAD);
		quaternion_msg = tf2::toMsg(q);
		imu_data->orientation.x = quaternion_msg.x;
		imu_data->orientation.y = quaternion_msg.y;
		imu_data->orientation.z = quaternion_msg.z;
		imu_data->orientation.w = quaternion_msg.w;
		imu_data->angular_velocity.x = (float)data->hi92.gyr_b[0] * DEG_TO_RAD * GYR_FACTOR;
		imu_data->angular_velocity.y = (float)data->hi92.gyr_b[1] * DEG_TO_RAD * GYR_FACTOR;
		imu_data->angular_velocity.z = (float)data->hi92.gyr_b[2] * DEG_TO_RAD * GYR_FACTOR;
		imu_data->linear_acceleration.x = (float)data->hi92.acc_b[0] * ACC_FACTOR;
		imu_data->linear_acceleration.y = (float)data->hi92.acc_b[1] * ACC_FACTOR;
		imu_data->linear_acceleration.z = (float)data->hi92.acc_b[2] * ACC_FACTOR;
	}

}


