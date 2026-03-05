#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cerrno>
#include <cstring>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"{
#endif
#include <poll.h>

#include "hipnuc_dec.h"

#define GRA_ACC          (9.8)
#define DEG_TO_RAD       (0.01745329)
#define UTESLA_TO_TESLA  (0.000001)
#define BUF_SIZE         (1024)
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
			
			IMUPublisher(const rclcpp::NodeOptions &options) : Node("IMU_publisher", options)	
			{
				this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
				this->declare_parameter<int>("baud_rate", 115200);
				this->declare_parameter<std::string>("frame_id", "base_link");
				this->declare_parameter<std::string>("imu_topic", "/IMU_data");
				this->declare_parameter<std::string>("euler_topic", "/euler_data");
				this->declare_parameter<std::string>("magnetic_topic", "/magnetic_data");
				this->declare_parameter<bool>("imu_switch", true);
				this->declare_parameter<bool>("euler_switch", false);
				this->declare_parameter<bool>("magnetic_switch", false);

				this->get_parameter("serial_port", serial_port);
				this->get_parameter("baud_rate", baud_rate);
				this->get_parameter("frame_id", frame_id);
				this->get_parameter("imu_topic", imu_topic);
				this->get_parameter("euler_topic", euler_topic);
				this->get_parameter("magnetic_topic", magnetic_topic);
				this->get_parameter("imu_switch", imu_switch);
				this->get_parameter("euler_switch", euler_switch);
				this->get_parameter("magnetic_switch", magnetic_switch);

				RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
				RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
				RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
				RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "euler_topic: %s\r\n", euler_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "magnetic_topic: %s\r\n", magnetic_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "imu_switch: %d\r\n", imu_switch);
				RCLCPP_INFO(this->get_logger(), "euler_switch: %d\r\n", euler_switch);
				RCLCPP_INFO(this->get_logger(), "magnetic_switch: %d\r\n", magnetic_switch);
				
				imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
				euler_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic, rclcpp::SensorDataQoS());
				magnetic_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic, rclcpp::SensorDataQoS());
				
				fd = open_ttyport(serial_port, baud_rate);

				if (fd < 0) {
					RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
					return;
				}

				running.store(true);
				reader_thread = std::thread([this]() {this->read_loop(); });
			}

			~IMUPublisher() override
			{
				running.store(false);

				if (reader_thread.joinable())
					reader_thread.join();
				
				if (fd >= 0) {
					close(fd);
					fd = -1;
				}
			}

		private: 
			void read_loop()
			{
				struct pollfd p{};
				p.fd = fd;
				p.events = POLLIN;

				while(rclcpp::ok() && running.load())
				{
					int rpoll = poll(&p, 1, 2);
					if (rpoll == 0)
						continue;
					
					if (rpoll < 0) {
						if (errno == EINTR) 
							continue;

						RCLCPP_ERROR(this->get_logger(), "poll() failed: %s", std::strerror(errno));
						break;
					}

					if (!(p.revents & POLLIN))
						continue;

					int n = read(fd, buf, sizeof(buf));
					if (n < 0) {
						if (errno == EAGAIN || errno == EWOULDBLOCK) 
							continue;

						RCLCPP_ERROR(this->get_logger(), "read() failed: %s", std::strerror(errno));
						break;
					}

					if (n == 0) {
						std::this_thread::sleep_for(10ms);
						continue;
					}

					for (int i = 0; i < n; i++) {
						int rev = hipnuc_input(&raw, buf[i]);
						if (!rev) continue;

						
                        if (raw.hi83.tag == 0x83)
                        {
                            uint32_t bm = raw.hi83.data_bitmap;
                            if (bm & HI83_BMAP_QUAT)
                            {
                                imu_msg.orientation.w = raw.hi83.quat[0];
                                imu_msg.orientation.x = raw.hi83.quat[1];
                                imu_msg.orientation.y = raw.hi83.quat[2];
                                imu_msg.orientation.z = raw.hi83.quat[3];
                            }
                            if (bm & HI83_BMAP_GYR_B)
                            {
                                imu_msg.angular_velocity.x = raw.hi83.gyr_b[0];
                                imu_msg.angular_velocity.y = raw.hi83.gyr_b[1];
                                imu_msg.angular_velocity.z = raw.hi83.gyr_b[2];
                            }
                            if (bm & HI83_BMAP_ACC_B)
                            {
                                imu_msg.linear_acceleration.x = raw.hi83.acc_b[0];
                                imu_msg.linear_acceleration.y = raw.hi83.acc_b[1];
                                imu_msg.linear_acceleration.z = raw.hi83.acc_b[2];
                            }
							if (bm & HI83_BMAP_MAG_B)
							{
								magnetic_msg.magnetic_field.x = raw.hi83.mag_b[0] * UTESLA_TO_TESLA ;
								magnetic_msg.magnetic_field.y = raw.hi83.mag_b[1] * UTESLA_TO_TESLA ;
								magnetic_msg.magnetic_field.z = raw.hi83.mag_b[2] * UTESLA_TO_TESLA ;
							}

							if (bm & HI83_BMAP_RPY)
							{
								euler_msg.vector.x = raw.hi83.rpy[0] * DEG_TO_RAD;
								euler_msg.vector.y = raw.hi83.rpy[1] * DEG_TO_RAD;
								euler_msg.vector.z = raw.hi83.rpy[2] * DEG_TO_RAD;
							}
                        }
                        else if (raw.hi91.tag == 0x91)
                        {
                            imu_msg.orientation.w = raw.hi91.quat[0];
                            imu_msg.orientation.x = raw.hi91.quat[1]; 
                            imu_msg.orientation.y = raw.hi91.quat[2];
                            imu_msg.orientation.z = raw.hi91.quat[3];
                            imu_msg.angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
                            imu_msg.angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
                            imu_msg.angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
                            imu_msg.linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
                            imu_msg.linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
                            imu_msg.linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;

							euler_msg.vector.x = raw.hi91.roll  * DEG_TO_RAD;
							euler_msg.vector.y = raw.hi91.pitch * DEG_TO_RAD;
							euler_msg.vector.z = raw.hi91.yaw   * DEG_TO_RAD;

							magnetic_msg.magnetic_field.x = raw.hi91.mag[0] * UTESLA_TO_TESLA ;
							magnetic_msg.magnetic_field.y = raw.hi91.mag[1] * UTESLA_TO_TESLA ;
							magnetic_msg.magnetic_field.z = raw.hi91.mag[2] * UTESLA_TO_TESLA ;
                        }
						else 
							continue;

						if (imu_switch) {
							imu_msg.header.frame_id = frame_id;
							imu_msg.header.stamp = rclcpp::Clock().now();
							imu_pub->publish(imu_msg);
						}
                       
						if (magnetic_switch) {
							magnetic_msg.header.stamp = rclcpp::Clock().now();
							magnetic_msg.header.frame_id = frame_id;
							magnetic_pub->publish(magnetic_msg);
						}
						
						if (euler_switch) {
							euler_msg.header.stamp = rclcpp::Clock().now();
							euler_msg.header.frame_id = frame_id;
							euler_pub->publish(euler_msg);
						}
						
					}
				}
			}

			int open_ttyport(const std::string &tty_port, int baud)
			{
				const char *port_device = tty_port.c_str();
				int serial_port = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
				if (serial_port < 0)
				{
					RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s: %s", port_device, std::strerror(errno));
					return -1;
				}

				struct termios2 tty {};

				if (ioctl(serial_port, TCGETS2, &tty) != 0)
				{
					RCLCPP_ERROR(this->get_logger(), "TCGETS2 failed: %s ", std::strerror(errno));
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
					RCLCPP_ERROR(this->get_logger(), "TCSETS2 failed : %s", std::strerror(errno));
					close(serial_port);
					return -1;
				}

				return serial_port;
			}

			int fd = 0;
			uint8_t buf[BUF_SIZE] = {0};

			std::string serial_port;
			int baud_rate;
			std::string frame_id;
			bool imu_switch;
			bool euler_switch;
			bool magnetic_switch;
			std::string imu_topic;
			std::string euler_topic;
			std::string magnetic_topic;

			// auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
			sensor_msgs::msg::Imu imu_msg;
			sensor_msgs::msg::MagneticField magnetic_msg;
			geometry_msgs::msg::Vector3Stamped euler_msg;

			rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
			rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub;
			rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub;

			std::atomic<bool> running{false};
			std::thread reader_thread;
	};
}


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
