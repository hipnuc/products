//subscriiber 0x91 data package 

#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <serial_port/Imu_0x91_msg.h>
#include "ch_serial.h"

void imu_0x91_callback(const serial_port::Imu_0x91_msg imu_0x91_msg);

int main(int argc,char **argv)
{
	ros::init(argc,argv,"sub_0x91");

	ros::NodeHandle n;
	
	ros::Subscriber imu_0x91_sub = n.subscribe("/imu_0x91_package", 10, imu_0x91_callback);

	ros::spin();
}


void imu_0x91_callback(const serial_port::Imu_0x91_msg imu_0x91_msg)
{
	printf("\033c");

	printf("     Devie ID:%6d\n",imu_0x91_msg.imu_data.id);

	printf("     Prs(hPa): %6f\n", imu_0x91_msg.imu_data.prs);

	printf("    Run times: %d days  %d:%d:%d:%d\n",imu_0x91_msg.imu_data.time / 86400000, imu_0x91_msg.imu_data.time / 3600000 % 24, imu_0x91_msg.imu_data.time / 60000 % 60, imu_0x91_msg.imu_data.time / 1000 % 60, imu_0x91_msg.imu_data.time % 1000);

	printf("   Frame Rate:  %4dHz\r\n", imu_0x91_msg.imu_data.frame_rate);

	printf("       Acc(G):%8.3f %8.3f %8.3f\r\n", imu_0x91_msg.imu_data.acc_x, imu_0x91_msg.imu_data.acc_y, imu_0x91_msg.imu_data.acc_z);

	printf("   Gyr(deg/s):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.imu_data.gyr_x, imu_0x91_msg.imu_data.gyr_y, imu_0x91_msg.imu_data.gyr_z);

	printf("      Mag(uT):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.imu_data.mag_x, imu_0x91_msg.imu_data.mag_y, imu_0x91_msg.imu_data.mag_z);

	printf("   Eul(R P Y):%8.2f %8.2f %8.2f\r\n", imu_0x91_msg.imu_data.eul_r, imu_0x91_msg.imu_data.eul_p, imu_0x91_msg.imu_data.eul_y);

	printf("Quat(W X Y Z):%8.3f %8.3f %8.3f %8.3f\r\n", imu_0x91_msg.imu_data.quat_w, imu_0x91_msg.imu_data.quat_x, imu_0x91_msg.imu_data.quat_y, imu_0x91_msg.imu_data.quat_z);
}
