//serial_imu.cpp
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "ch_serial.h"

#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data);

#ifdef __cplusplus
}
#endif


int baud_rate;
std::string imu_serial;
static raw_t raw;
ros::Publisher IMU_pub;
sensor_msgs::Imu imu_data;
int fd = 0;
static uint8_t buf[2048];

void read_imu(void)
{
	int n = 0;
	int rev = 0;
	n = read(fd, buf, sizeof(buf));

	if(n > 0)
	{
		for(int i = 0; i < n; i++)
		{
			rev = ch_serial_input(&raw, buf[i]);
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
	int fd = open(port_device1, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd == -1)
	{
		perror("open_port: Unable to open SerialPort");
		puts("Please check the usb port name!!!");
		exit(0);
	}

	if(fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
		printf("fcntl failed\n");
	else
		fcntl(fd, F_SETFL, O_NONBLOCK);
  
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

	ros::param::param<std::string>("/imu_serial", imu_serial, "/dev/ttyUSB1");
	ros::param::param<int>("/baud_rate", baud_rate, 921600);

    IMU_pub = n.advertise<sensor_msgs::Imu>("/IMU_data", 5);

	fd = open_port(imu_serial, baud_rate);
	
	imu_data.header.frame_id = "base_link";

	while(ros::ok())
	{
		read_imu();
	}
	
	return 0;
}

void publish_imu_data(raw_t *data, sensor_msgs::Imu *imu_data)
{	
	imu_data->orientation.x = data->imu.quat[1];
	imu_data->orientation.y = data->imu.quat[2];
	imu_data->orientation.z = data->imu.quat[3];
	imu_data->orientation.w = data->imu.quat[0];
	imu_data->angular_velocity.x = data->imu.gyr[0] * DEG_TO_RAD;
	imu_data->angular_velocity.y = data->imu.gyr[1] * DEG_TO_RAD;
	imu_data->angular_velocity.z = data->imu.gyr[2] * DEG_TO_RAD;
	imu_data->linear_acceleration.x = data->imu.acc[0] * GRA_ACC;
	imu_data->linear_acceleration.y = data->imu.acc[1] * GRA_ACC;
	imu_data->linear_acceleration.z = data->imu.acc[2] * GRA_ACC;
}


