//serial_imu.cpp
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <hipnuc_imu/Imu_0x91_msg.h>
#include <hipnuc_imu/Imu_data_package.h>

#ifdef __cplusplus 
extern "C"{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <unistd.h> 
#include <fcntl.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <poll.h>
#include "hipnuc.h"

// #define IMU_SERIAL   "/dev/ttyUSB0"
// #define BAUD         (B115200)
#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

void publish_0x91_data(hipnuc_raw_t *data, hipnuc_imu::Imu_0x91_msg *data_imu);

#ifdef __cplusplus
}
#endif

std::string imu_port;
int serial_baud;
std::string frame_id;
std::string imu_topic;

hipnuc_imu::Imu_0x91_msg imu_0x91_msg;
ros::Publisher Imu_0x91_pub;

static hipnuc_raw_t raw;

static int frame_rate;
static int frame_count;
int fd = 0;
static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate = frame_count;
		frame_count = 0;
		alarm(1);
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

void read_imu(void)
{
	int n = 0; 
	int rev = 0;
	struct pollfd p;
	p.fd = fd;
	p.events = POLLIN;
	double time_sec;

	int rpoll = poll(&p, 1, 5);

	if(rpoll == 0)
		return ;
	n = read(fd, buf, sizeof(buf));

	if(n > 0)
	{
		for (int i = 0; i < n; i++)
		{
			rev = hipnuc_input(&raw, buf[i]);

			if(rev)
			{
				imu_0x91_msg.header.stamp = ros::Time::now();
				publish_0x91_data(&raw, &imu_0x91_msg);
				Imu_0x91_pub.publish(imu_0x91_msg);
				rev = 0;
				frame_count++;
			}

		}
	}
}


int main(int argc, char** argv)
{
	int rev = 0;
	ros::init(argc, argv, "hipnuc_imu_0x91");
	ros::NodeHandle n;

	ros::param::param<std::string>("/imu_serial", imu_port, "/dev/ttyUSB1");
	ros::param::param<int>("/baud_rate", serial_baud, 921600);
	ros::param::param<std::string>("/frame_id_costom", frame_id, "base_0x91_link");
	ros::param::param<std::string>("/imu_topic_costom", imu_topic, "/imu_0x91_package");

	Imu_0x91_pub = n.advertise<hipnuc_imu::Imu_0x91_msg>(imu_topic, 10);

	fd = open_port(imu_port, serial_baud);

	imu_0x91_msg.header.frame_id = frame_id;

	signal(SIGALRM,timer);

	alarm(1);
	
	while(ros::ok())
	{
		read_imu();
	}
    
	return 0;
}

void imu_data_package(hipnuc_raw_t *data, hipnuc_imu::Imu_data_package *data_imu, int num)
{
	data_imu->frame_rate = frame_rate;

	data_imu->tag = data->hi91.tag;

	data_imu->time = data->hi91.ts;

	data_imu->prs = data->hi91.prs;

	data_imu->acc_x = data->hi91.acc[0];
	data_imu->acc_y = data->hi91.acc[1];
	data_imu->acc_z = data->hi91.acc[2];

	data_imu->gyr_x = data->hi91.gyr[0];
	data_imu->gyr_y = data->hi91.gyr[1];
	data_imu->gyr_z = data->hi91.gyr[2];

	data_imu->mag_x = data->hi91.mag[0];
	data_imu->mag_y = data->hi91.mag[1];
	data_imu->mag_z = data->hi91.mag[2];

	data_imu->eul_r = data->hi91.roll;
	data_imu->eul_p = data->hi91.pitch;
	data_imu->eul_y = data->hi91.yaw;

	data_imu->quat_w = data->hi91.quat[0];
	data_imu->quat_x = data->hi91.quat[1];
	data_imu->quat_y = data->hi91.quat[2];
	data_imu->quat_z = data->hi91.quat[3];
}

void publish_0x91_data(hipnuc_raw_t *data, hipnuc_imu::Imu_0x91_msg *data_imu)
{
	imu_data_package(data, &(data_imu->imu_data),1);
}

