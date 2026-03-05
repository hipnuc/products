#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

#ifdef __cplusplus
extern "C" {
#endif

#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hipnuc_dec.h"
#include "canopen_parser.h"
#include "hipnuc_can_common.h"
#include "hipnuc_j1939_parser.h"

#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)

#define TPDO1_BASE  0x180
#define TPDO2_BASE  0x280
#define TPDO3_BASE  0x380
#define TPDO4_BASE  0x480
#define TPDO6_BASE  0x680
#define TPDO7_BASE  0x780

#define J1939_PGN_TIME       0xFF2F
#define J1939_PGN_ACCEL      0xFF34
#define J1939_PGN_GYRO       0xFF37
#define J1939_PGN_MAG        0xFF3A
#define J1939_PGN_PITCH_ROLL 0xFF3D
#define J1939_PGN_YAW        0xFF41
#define J1939_PGN_QUAT       0xFF46
#define J1939_PGN_INCLINE    0xFF4A

#ifdef __cplusplus
}
#endif

namespace hipnuc_driver {
class CanImuPublisher : public rclcpp::Node
{
  public:
  CanImuPublisher(const rclcpp::NodeOptions &options)
  : Node("canImuPublisher", options)
  {
    can_port = this->declare_parameter<std::string>("can_port", "can0");
    baudrate = this->declare_parameter<int>("baud_rate", 500000);
    node_id  = this->declare_parameter<int>("node_id", 8);
    frame_id = this->declare_parameter<std::string>("frame_id", "imu_link");
    imu_topic_canopen = this->declare_parameter<std::string>("imu_topic_canopen", "/imu/canopen_msg");
    euler_topic_canopen = this->declare_parameter<std::string>("euler_topic_canopen", "/euler/canopen_msg");

    imu_topic_j1939 = this->declare_parameter<std::string>("imu_topic_j1939", "/imu/j1939_msg");
    mag_topic_j1939 = this->declare_parameter<std::string>("mag_topic_j1939", "/mag/j1939_msg");
    euler_topic_j1939 = this->declare_parameter<std::string>("euler_topic_j1939", "/euler/j1939_msg");

    imu_switch = this->declare_parameter<bool>("imu_switch", false);
    euler_switch = this->declare_parameter<bool>("euler_switch", false);
    mag_switch = this->declare_parameter<bool>("mag_switch", false);

    RCLCPP_INFO(this->get_logger(), "can_port: %s\r\n", can_port.c_str());
    RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baudrate);
    RCLCPP_INFO(this->get_logger(), "node_id : %d\r\n", node_id);
    RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic_canopen: %s\r\n", imu_topic_canopen.c_str());
    RCLCPP_INFO(this->get_logger(), "euler_topic_canopen: %s\r\n", euler_topic_canopen.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic_j1939: %s\r\n", imu_topic_j1939.c_str());
    RCLCPP_INFO(this->get_logger(), "mag_topic_j1939: %s\r\n", mag_topic_j1939.c_str());
    RCLCPP_INFO(this->get_logger(), "euler_topic_j1939: %s\r\n", euler_topic_j1939.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_switch: %d\r\n", imu_switch);
    RCLCPP_INFO(this->get_logger(), "euler_switch: %d\r\n", euler_switch);
    RCLCPP_INFO(this->get_logger(), "mag_switch: %d\r\n", mag_switch);

    imu_canopen_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_canopen, rclcpp::SensorDataQoS());
    euler_canopen_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic_canopen, rclcpp::SensorDataQoS());
    imu_j1939_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_j1939, rclcpp::SensorDataQoS());
    mag_j1939_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_j1939, rclcpp::SensorDataQoS());
    euler_j1939_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic_j1939, rclcpp::SensorDataQoS());
    
    open_can_socket();

    reader_thread = std::thread([this]() { this->read_loop();});

    RCLCPP_INFO(this->get_logger(), "Reading SocketCAN from %s, publishing imu data", can_port.c_str());
  }

  ~CanImuPublisher() override
  {
    stop = true;
    if (can_fd >= 0)
      ::shutdown(can_fd, SHUT_RDWR);

    if (reader_thread.joinable()) 
      reader_thread.join();

    if (can_fd >= 0)
      ::close(can_fd);
  }

  private:
    void open_can_socket(void)
    {
      can_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (can_fd < 0)
        throw std::runtime_error(std::string("socket(PF_CAN) failed: ") + strerror(errno));

      struct ifreq ifr;
      memset(&ifr, 0, sizeof(ifr));
      strncpy(ifr.ifr_name, can_port.c_str(), IFNAMSIZ - 1);

      if (::ioctl(can_fd, SIOCGIFINDEX, &ifr) < 0) 
        throw std::runtime_error(std::string("ioctl(SIOCGIFINDEX) failed: ") + strerror(errno));

      sockaddr_can addr;
      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (::bind(can_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) 
        throw std::runtime_error(std::string("bind(AF_CAN) failed: ") + strerror(errno));
    }
 
    void handle_j1939_frame(const can_frame &frame)
    {
      imu_frame.can_id = frame.can_id; 
      imu_frame.can_dlc = frame.can_dlc;
      memcpy(imu_frame.data, frame.data, sizeof(imu_frame.data));
      hipnuc_j1939_parse_frame(&imu_frame, &imu_j1939_data);

      if (frame.can_dlc < 6)
        return; 

      switch((frame.can_id & 0xffff00) >> 8)
      {
        case J1939_PGN_TIME:
          j1939_data_flag |= 1 << 0;
          break;
        case J1939_PGN_ACCEL:
          j1939_data_flag |= 1 << 1;
          break;
        case J1939_PGN_GYRO:
          j1939_data_flag |= 1 << 2;
          break;
        case J1939_PGN_MAG:
          j1939_data_flag |= 1 << 3;
          break;
        case J1939_PGN_PITCH_ROLL:
          j1939_data_flag |= 1 << 4;
          break;
        case J1939_PGN_YAW:
          j1939_data_flag |= 1 << 5;
          break;
        case J1939_PGN_QUAT:
          j1939_data_flag |= 1 << 6;
          break;
        default:
          break;
      }

      if (imu_switch && (j1939_data_flag & ((1 << 6) | (1 << 2) | (1 << 1))) == ((1 << 6) | (1 << 2) | (1 << 1)))
      {
        j1939_data_flag &= ~((1 << 6) | (1 << 2) | (1 << 1));
        imu_j1939_msg.header.stamp = this->now();
        imu_j1939_msg.header.frame_id = frame_id;

        imu_j1939_msg.linear_acceleration.x = imu_j1939_data.acc_x * GRA_ACC;
        imu_j1939_msg.linear_acceleration.y = imu_j1939_data.acc_y * GRA_ACC;
        imu_j1939_msg.linear_acceleration.z = imu_j1939_data.acc_z * GRA_ACC;

        imu_j1939_msg.angular_velocity.x = imu_j1939_data.gyr_x * DEG_TO_RAD;
        imu_j1939_msg.angular_velocity.y = imu_j1939_data.gyr_y * DEG_TO_RAD;
        imu_j1939_msg.angular_velocity.z = imu_j1939_data.gyr_z * DEG_TO_RAD;

        imu_j1939_msg.orientation.w = imu_j1939_data.quat_w;
        imu_j1939_msg.orientation.x = imu_j1939_data.quat_x;
        imu_j1939_msg.orientation.y = imu_j1939_data.quat_y;
        imu_j1939_msg.orientation.z = imu_j1939_data.quat_z;

        imu_j1939_pub->publish(imu_j1939_msg);
      }

      if (euler_switch && (j1939_data_flag & ((1 << 4) | (1 << 5))) == ((1 << 4) | (1 << 5)))
      {
        j1939_data_flag &= ~((1 << 4) | (1 << 5));

        euler_j1939_msg.header.stamp = this->now();
        euler_j1939_msg.header.frame_id = frame_id;

        euler_j1939_msg.vector.x = imu_j1939_data.roll * DEG_TO_RAD;
        euler_j1939_msg.vector.y = imu_j1939_data.pitch * DEG_TO_RAD;
        euler_j1939_msg.vector.z = imu_j1939_data.imu_yaw * DEG_TO_RAD;

        euler_j1939_pub->publish(euler_j1939_msg);
      }

      if (mag_switch && (j1939_data_flag & (1 << 3)) == (1 << 3))
      {
        j1939_data_flag &= ~(1 << 3);

        mag_j1939_msg.header.stamp = this->now();
        mag_j1939_msg.header.frame_id = frame_id;

        mag_j1939_msg.magnetic_field.x = imu_j1939_data.mag_x * 1e-6;
        mag_j1939_msg.magnetic_field.y = imu_j1939_data.mag_y * 1e-6;
        mag_j1939_msg.magnetic_field.z = imu_j1939_data.mag_z * 1e-6;

        mag_j1939_pub->publish(mag_j1939_msg);
      }
      
    }

    void handle_canopen_frame(const can_frame &frame)
    {
        imu_frame.can_id = frame.can_id & CAN_SFF_MASK;  
        imu_frame.can_dlc = frame.can_dlc;
        memcpy(imu_frame.data, frame.data, sizeof(imu_frame.data));
        canopen_parse_frame(&imu_frame, &imu_canopen_data); 

        if (frame.can_dlc < 6)
          return;

        std::ostringstream oss;
        oss << can_port << " " << std::hex << std::uppercase << std::setfill('0') << std::setw(3) 
        << frame.can_id << " [" << std::dec << static_cast<int>(frame.can_dlc) << "] ";

        for (int i = 0; i < frame.can_dlc; i++)
          oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2) 
          << static_cast<int>(frame.data[i]) << " ";
        
        switch(frame.can_id & 0x780)
        {
          case TPDO1_BASE:
            canopen_data_flag |= 1 << 0;
            oss << "  " << std::setw(3) << static_cast<float>(imu_canopen_data.acc_x) << " " << static_cast<float>(imu_canopen_data.acc_y) << " " << static_cast<float>(imu_canopen_data.acc_z) << " "; 
            break;
          case TPDO2_BASE:
            canopen_data_flag |= 1 << 1;
            oss << "  " << std::setw(3) << static_cast<float>(imu_canopen_data.gyr_x) << " " << static_cast<float>(imu_canopen_data.gyr_y) << " " << static_cast<float>(imu_canopen_data.gyr_z) << " " ;
            break;
          case TPDO3_BASE:
            canopen_data_flag |= 1 << 2;
            oss << "  " << std::setw(3) << static_cast<float>(imu_canopen_data.roll * 180 / 3.1415926) << " " << static_cast<float>(imu_canopen_data.pitch * 180 / 3.1415926) << " " << static_cast<float>(imu_canopen_data.imu_yaw * 180 / 3.1415926) << " ";
            break;
          case TPDO4_BASE:
            canopen_data_flag |= 1 << 3;
            oss << "  " << std::setw(3) << static_cast<float>(imu_canopen_data.quat_w) << " " << static_cast<float>(imu_canopen_data.quat_x) << " " << static_cast<float>(imu_canopen_data.quat_y) << " " << static_cast<float>(imu_canopen_data.quat_z) << " ";
            break;
          case TPDO6_BASE:
            canopen_data_flag |= 1 << 4;
            break;
          case TPDO7_BASE:
            canopen_data_flag |= 1 << 5;
            break;
          default:
            oss << "  " << "unknow can id: " << static_cast<int>(frame.can_id);
            break;
        }

        /* read data */
        // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        if (imu_switch && (canopen_data_flag & (1 | (1 << 1) | (1 << 3))) == (1 | (1 << 1) | (1 << 3)))
        {
          canopen_data_flag &= ~(1 | (1 << 1) | (1 << 3));

          imu_canopen_msg.header.stamp = this->now();
          imu_canopen_msg.header.frame_id = frame_id;
          
          imu_canopen_msg.linear_acceleration.x = imu_canopen_data.acc_x;
          imu_canopen_msg.linear_acceleration.y = imu_canopen_data.acc_y;
          imu_canopen_msg.linear_acceleration.z = imu_canopen_data.acc_z;

          imu_canopen_msg.angular_velocity.x = imu_canopen_data.gyr_x;
          imu_canopen_msg.angular_velocity.y = imu_canopen_data.gyr_y;
          imu_canopen_msg.angular_velocity.z = imu_canopen_data.gyr_z;

          imu_canopen_msg.orientation.w = imu_canopen_data.quat_w;
          imu_canopen_msg.orientation.x = imu_canopen_data.quat_x;
          imu_canopen_msg.orientation.y = imu_canopen_data.quat_y;
          imu_canopen_msg.orientation.z = imu_canopen_data.quat_z;

          imu_canopen_pub->publish(imu_canopen_msg);
        }

        if (euler_switch && (canopen_data_flag & (1 << 2)) == (1 << 2))
        {
          canopen_data_flag &= ~(1 << 2);

          euler_canopen_msg.header.stamp = this->now();
          euler_canopen_msg.header.frame_id = frame_id;

          euler_canopen_msg.vector.x = imu_canopen_data.roll;
          euler_canopen_msg.vector.y = imu_canopen_data.pitch;
          euler_canopen_msg.vector.z = imu_canopen_data.imu_yaw;

          euler_canopen_pub->publish(euler_canopen_msg);
        }
    }

    void read_loop()
    {
      while (rclcpp::ok() && !stop) {
        struct can_frame frame;
        const int nbytes = ::read(can_fd, &frame, sizeof(frame));
        if (nbytes < 0) {
          if (!stop) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "CAN read error: %s", strerror(errno));
            continue;
          }
        }

        if (static_cast<size_t>(nbytes) < sizeof(struct can_frame))
          continue;

        if ((frame.can_id & 0x7f) != node_id)
          continue;
        
        if (frame.can_id & CAN_EFF_FLAG)
          handle_j1939_frame(frame);
        else
          handle_canopen_frame(frame);
      }
    }

    private:
    uint16_t canopen_data_flag = 0, j1939_data_flag = 0;
    std::string can_port;
    std::string frame_id;
    std::string imu_topic_canopen, imu_topic_j1939;
    std::string euler_topic_canopen, euler_topic_j1939;
    std::string mag_topic_j1939;
    bool imu_switch;
    bool euler_switch;
    bool mag_switch;
    int can_fd{-1};
    int baudrate;
    int node_id{-1};

    hipnuc_can_frame_t imu_frame;
    can_sensor_data_t imu_canopen_data, imu_j1939_data;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_canopen_pub, imu_j1939_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_j1939_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_canopen_pub, euler_j1939_pub;

    std::thread reader_thread;
    std::atomic<bool> stop{false};
    
    sensor_msgs::msg::Imu imu_canopen_msg, imu_j1939_msg;
    sensor_msgs::msg::MagneticField mag_j1939_msg;
    geometry_msgs::msg::Vector3Stamped euler_canopen_msg, euler_j1939_msg;
};
}

int main(int argc, const char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  try {
    auto node = std::make_shared<hipnuc_driver::CanImuPublisher>(options);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }

  rclcpp::shutdown();
  return 0;

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_driver::CanImuPublisher)
