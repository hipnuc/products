//serial_imu.cpp
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <hipnuc_imu/hipnuc_imu_msg.h>
#include <hipnuc_imu/Imu_0x91_package.h>
#include <hipnuc_imu/Imu_0x92_package.h>
#include <hipnuc_imu/Ins_0x81_package.h>

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

#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     1024

#define ACC_FACTOR   (0.0048828)
#define GYR_FACTOR	 (0.001)
#define MAG_FACTOR   (0.030517)
#define EUL_FACTOR	 (0.001)
#define QUA_FACTOR   (0.0001)

void publish_data_package(hipnuc_raw_t *data, hipnuc_imu::hipnuc_imu_msg *data_imu);

#ifdef __cplusplus
}
#endif


hipnuc_imu::hipnuc_imu_msg himu_msg;
ros::Publisher  Hipnuc_imu_pub;

static hipnuc_raw_t raw;

static int frame_rate_0x91, frame_rate_0x92;
static int frame_count_0x91, frame_count_0x92;

static uint8_t buf[2048];

void timer(int sig)
{
	if(SIGALRM == sig)
	{
		frame_rate_0x91 = frame_count_0x91;
		frame_count_0x91 = 0;
		frame_rate_0x92 = frame_count_0x92;
		frame_count_0x92 = 0;
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

void read_imu(int fd, struct pollfd *p)
{
	int n = 0; 
	int rev = 0;
	int rpoll = poll(p, 1, 5);

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
				himu_msg.header.stamp = ros::Time::now();
				publish_data_package(&raw, &himu_msg);
				rev = 0;
			}

		}
	}
}


int main(int argc, char** argv)
{
	int rev = 0;
	ros::init(argc, argv, "hipnuc_imu_msg");
	ros::NodeHandle n;

	std::string imu_port;
	int serial_baud;
	std::string frame_id;
	std::string imu_topic;
	int fd = 0;
	struct pollfd p;

	ros::param::param<std::string>("/imu_serial", imu_port, "/dev/ttyUSB0");
	ros::param::param<int>("/baud_rate", serial_baud, 115200);
	ros::param::param<std::string>("/frame_id_costom", frame_id, "base_link_hipnuc");
	ros::param::param<std::string>("/imu_topic_costom", imu_topic, "/imu_package_hipnuc");

	Hipnuc_imu_pub = n.advertise<hipnuc_imu::hipnuc_imu_msg>(imu_topic, 10);

	fd = open_port(imu_port, serial_baud);

	himu_msg.header.frame_id = frame_id;

	signal(SIGALRM,timer);

	alarm(1);

	p.fd = fd;
	p.events = POLLIN;
	
	while(ros::ok())
	{
		read_imu(fd, &p);
	}
    
	return 0;
}

void imu_0x91_data(hipnuc_raw_t *data, hipnuc_imu::hipnuc_imu_msg *data_imu)
{
	frame_count_0x91++;
	data_imu->hi91_data.frame_rate = frame_rate_0x91;

	data_imu->hi91_data.tag = data->hi91.tag;

	data_imu->hi91_data.system_time = data->hi91.ts;
	data_imu->hi91_data.air_pressure = data->hi91.prs;
	data_imu->hi91_data.pps_sync_stamp = data->hi91.pps_sync_ms;
	data_imu->hi91_data.temperature = data->hi91.temp;

	data_imu->hi91_data.acc_x = data->hi91.acc[0];
	data_imu->hi91_data.acc_y = data->hi91.acc[1];
	data_imu->hi91_data.acc_z = data->hi91.acc[2];

	data_imu->hi91_data.gyr_x = data->hi91.gyr[0];
	data_imu->hi91_data.gyr_y = data->hi91.gyr[1];
	data_imu->hi91_data.gyr_z = data->hi91.gyr[2];

	data_imu->hi91_data.mag_x = data->hi91.mag[0];
	data_imu->hi91_data.mag_y = data->hi91.mag[1];
	data_imu->hi91_data.mag_z = data->hi91.mag[2];

	data_imu->hi91_data.eul_r = data->hi91.roll;
	data_imu->hi91_data.eul_p = data->hi91.pitch;
	data_imu->hi91_data.eul_y = data->hi91.yaw;

	data_imu->hi91_data.quat_w = data->hi91.quat[0];
	data_imu->hi91_data.quat_x = data->hi91.quat[1];
	data_imu->hi91_data.quat_y = data->hi91.quat[2];
	data_imu->hi91_data.quat_z = data->hi91.quat[3];

}

void imu_0x92_data(hipnuc_raw_t *data, hipnuc_imu::hipnuc_imu_msg *data_imu)
{
	frame_count_0x92++;
	data_imu->hi92_data.frame_rate = frame_rate_0x92;

	data_imu->hi92_data.tag = data->hi92.tag;

	data_imu->hi92_data.air_pressure = data->hi92.air_pressure;
	data_imu->hi92_data.pps_sync_stamp = data->hi92.sync_time;
	data_imu->hi92_data.temperature = data->hi92.temperature;

	data_imu->hi92_data.acc_x = data->hi92.acc_b[0] * ACC_FACTOR;
	data_imu->hi92_data.acc_y = data->hi92.acc_b[1] * ACC_FACTOR;
	data_imu->hi92_data.acc_z = data->hi92.acc_b[2] * ACC_FACTOR;

	data_imu->hi92_data.gyr_x = data->hi92.gyr_b[0] * GYR_FACTOR;
	data_imu->hi92_data.gyr_y = data->hi92.gyr_b[1] * GYR_FACTOR;
	data_imu->hi92_data.gyr_z = data->hi92.gyr_b[2] * GYR_FACTOR;

	data_imu->hi92_data.mag_x = data->hi92.mag_b[0] * MAG_FACTOR;
	data_imu->hi92_data.mag_y = data->hi92.mag_b[1] * MAG_FACTOR;
	data_imu->hi92_data.mag_z = data->hi92.mag_b[2] * MAG_FACTOR;

	data_imu->hi92_data.eul_r = data->hi92.roll * EUL_FACTOR;
	data_imu->hi92_data.eul_p = data->hi92.pitch * EUL_FACTOR;
	data_imu->hi92_data.eul_y = data->hi92.yaw * EUL_FACTOR;

	data_imu->hi92_data.quat_w = data->hi92.quat[0] * QUA_FACTOR;
	data_imu->hi92_data.quat_x = data->hi92.quat[1] * QUA_FACTOR;
	data_imu->hi92_data.quat_y = data->hi92.quat[2] * QUA_FACTOR;
	data_imu->hi92_data.quat_z = data->hi92.quat[3] * QUA_FACTOR;
}


void publish_data_package(hipnuc_raw_t *data, hipnuc_imu::hipnuc_imu_msg *data_imu)
{
	if(data->hi91.tag == 0x91)
		imu_0x91_data(data, data_imu);
	if(data->hi92.tag == 0x92)
		imu_0x92_data(data, data_imu);

	Hipnuc_imu_pub.publish(*data_imu);
	memset(data_imu, 0, sizeof(data_imu));
}

