//subscriiber 0x91 data package 

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <hipnuc_imu/hipnuc_imu_msg.h>
#include <hipnuc_imu/Imu_0x91_package.h>
#include <hipnuc_imu/Imu_0x92_package.h>
#include "hipnuc.h"

void imu_callback_hipnuc(const hipnuc_imu::hipnuc_imu_msg imu_msg_hipnuc);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"sub_0x91");

	ros::NodeHandle n;
	std::string imu_topic;

	ros::param::param<std::string>("/imu_topic_costom", imu_topic, "/imu_package_hipnuc");
	
	ros::Subscriber imu_0x91_sub = n.subscribe(imu_topic, 10, imu_callback_hipnuc);

	ros::spin();
}


void imu_callback_hipnuc(const hipnuc_imu::hipnuc_imu_msg imu_msg_hipnuc)
{
	printf("\033c");

	if(imu_msg_hipnuc.hi91_data.tag == 0x91)
	{
		printf("%16s %#8x\n", "DataPackage Tag:", imu_msg_hipnuc.hi91_data.tag);
		printf("%16s %6dHz\r\n","Frame Rate:", imu_msg_hipnuc.hi91_data.frame_rate);
		printf("%16s %8f\n", "Prs(hPa):", imu_msg_hipnuc.hi91_data.air_pressure);

		printf("%16s %8d\n", "Temperature:", imu_msg_hipnuc.hi91_data.temperature);

		printf("%16s %d days  %d:%d:%d:%d\n","Run Time:", imu_msg_hipnuc.hi91_data.system_time / 86400000, imu_msg_hipnuc.hi91_data.system_time / 3600000 % 24, imu_msg_hipnuc.hi91_data.system_time / 60000 % 60, imu_msg_hipnuc.hi91_data.system_time / 1000 % 60, imu_msg_hipnuc.hi91_data.system_time % 1000);

		printf("%16s %8d\r\n","PPS TIme:", imu_msg_hipnuc.hi91_data.pps_sync_stamp);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Acc(G):", imu_msg_hipnuc.hi91_data.acc_x, imu_msg_hipnuc.hi91_data.acc_y, imu_msg_hipnuc.hi91_data.acc_z);

		printf("%16s %8.2f %8.2f %8.2f\r\n", "Gyr(deg/s):", imu_msg_hipnuc.hi91_data.gyr_x, imu_msg_hipnuc.hi91_data.gyr_y, imu_msg_hipnuc.hi91_data.gyr_z);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Mag(uT):", imu_msg_hipnuc.hi91_data.mag_x, imu_msg_hipnuc.hi91_data.mag_y, imu_msg_hipnuc.hi91_data.mag_z);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Eul(R P Y):", imu_msg_hipnuc.hi91_data.eul_r, imu_msg_hipnuc.hi91_data.eul_p, imu_msg_hipnuc.hi91_data.eul_y);

		printf("%16s %8.6f %8.6f %8.6f %8.6f\r\n", "Quat(W X Y Z):", imu_msg_hipnuc.hi91_data.quat_w, imu_msg_hipnuc.hi91_data.quat_x, imu_msg_hipnuc.hi91_data.quat_y, imu_msg_hipnuc.hi91_data.quat_z);
		putchar(10);
	}

	if(imu_msg_hipnuc.hi92_data.tag == 0x92)
	{
		printf("%16s %#8x\n", "DataPackage Tag:", imu_msg_hipnuc.hi92_data.tag);
		printf("%16s %6dHz\r\n","Frame Rate:", imu_msg_hipnuc.hi92_data.frame_rate);
		printf("%16s %8f\n", "Prs(hPa):", imu_msg_hipnuc.hi92_data.air_pressure);

		printf("%16s %8d\n", "Temperature:", imu_msg_hipnuc.hi92_data.temperature);

		printf("%16s %8d\r\n","PPS TIme:", imu_msg_hipnuc.hi92_data.pps_sync_stamp);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Acc(G):", imu_msg_hipnuc.hi92_data.acc_x, imu_msg_hipnuc.hi92_data.acc_y, imu_msg_hipnuc.hi92_data.acc_z);

		printf("%16s %8.2f %8.2f %8.2f\r\n", "Gyr(deg/s):", imu_msg_hipnuc.hi92_data.gyr_x, imu_msg_hipnuc.hi92_data.gyr_y, imu_msg_hipnuc.hi92_data.gyr_z);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Mag(uT):", imu_msg_hipnuc.hi92_data.mag_x, imu_msg_hipnuc.hi92_data.mag_y, imu_msg_hipnuc.hi92_data.mag_z);

		printf("%16s %8.3f %8.3f %8.3f\r\n", "Eul(R P Y):", imu_msg_hipnuc.hi92_data.eul_r, imu_msg_hipnuc.hi92_data.eul_p, imu_msg_hipnuc.hi92_data.eul_y);

		printf("%16s %8.6f %8.6f %8.6f %8.6f\r\n", "Quat(W X Y Z):", imu_msg_hipnuc.hi92_data.quat_w, imu_msg_hipnuc.hi92_data.quat_x, imu_msg_hipnuc.hi92_data.quat_y, imu_msg_hipnuc.hi92_data.quat_z);
		putchar(10);
	}
}
