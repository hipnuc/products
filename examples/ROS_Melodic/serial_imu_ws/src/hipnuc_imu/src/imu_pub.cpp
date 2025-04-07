#include "imu_pub.h"

#define GRA_ACC      (9.8)
#define DEG_TO_RAD   (0.01745329)
#define BUF_SIZE     (1024)
#define ACC_FACTOR   (0.0048828)
#define GYR_FACTOR	 (0.001)
#define MAG_FACTOR   (0.030517)
#define EUL_FACTOR	 (0.001)
#define QUA_FACTOR   (0.0001)

SerialIMUNode::SerialIMUNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
        instance_ = this;
        memset(&raw, 0, sizeof(raw));
        loadParameters();
        initPublishers();
        initSerialPort();
        initAlarm();
        
    }

SerialIMUNode::~SerialIMUNode()
{
    serial_port_->close();  
}

void SerialIMUNode::run()
{
    while (ros::ok() && reconnect_count < 20)
    {
        if (!serial_port_) 
        {
            ROS_ERROR("Serial port not initialized");
            return;
        }

        std::vector<uint8_t> raw_data;
        if (serial_port_->read(raw_data))
        {
            for (int i = 0; i < raw_data.size(); i++)
            {
                uint8_t rev = hipnuc_input(&raw, raw_data[i]);

                if(rev)
                {
                    if (imu_enable)
                    {
                        imu_msg.header.stamp = ros::Time::now();
                        publish_imu_data(&raw, &imu_msg);
                        imu_pub_.publish(imu_msg);
                    }
                    
                    if (mag_enable)
                    {
                        mag_msg.header.stamp = ros::Time::now();
                        publish_mag_data(&raw, &mag_msg);
                        mag_pub_.publish(mag_msg);
                    }

                    if (eul_enable)
                    {
                        eul_msg.header.stamp = ros::Time::now();
                        publish_eul_data(&raw, &eul_msg);
                        eul_pub_.publish(eul_msg);
                    }

                    if (pre_enable)
                    {
                        pre_msg.header.stamp = ros::Time::now();
                        publish_pre_data(&raw, &pre_msg);
                        pre_pub_.publish(pre_msg);
                    }

                    if (temp_enable)
                    {
                        temp_msg.header.stamp = ros::Time::now();
                        publish_temp_data(&raw, &temp_msg);
                        temp_pub_.publish(temp_msg);
                    }
                    
                    rev = 0;
                    if(setitimer(ITIMER_REAL, &timer, NULL) == -1)
                    {
                        perror("rev setitimer error");
                        return;
                    }
                }
            }
        }

        ros::spinOnce();
    }
}

void SerialIMUNode::open()
{
    if (!serial_port_ || !serial_port_->open()) 
    {
        ROS_ERROR("Failed to open serial port");
        return;
    }
    ROS_INFO("OPEN SERIAL SUCCESS");
    reconnect_count = 0;
}

void SerialIMUNode::close()
{
    if (!serial_port_ || !serial_port_->close())
    {
        ROS_ERROR("Failed to open serial port");
        return ;
    }
    reconnect_count = 0;
}

void SerialIMUNode::signal_handler(int sig) 
{
    

    if (instance_) {
        instance_->alarm_handler(sig);
    }
}

void SerialIMUNode::alarm_handler(int sig)
{
    if (sig != SIGALRM)
        return ;

    ros::Time now = ros::Time::now();
    static ros::Time last_reconnect_attempt = now;

    if  ((now - last_reconnect_attempt).toSec() < 1.0)
        return ;

    last_reconnect_attempt = now;

    reconnect_count++;
    ROS_WARN("Serial port connection lost, Reconnecttion attempt %d/%d", reconnect_count, MAX_RECONNECT_ATTEMPTS);

    if (serial_port_)
    {
        serial_port_->close();
        ros::Duration(0.5).sleep();
    }

    try
    {
        open();
        if (serial_port_ && serial_port_->isOpen())
            ROS_INFO("Successfully reconnected to serial port");
        else
            ROS_INFO("Failed to reconnect to serial port");
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Exception during reconnection: %s", e.what());
    }

    struct itimerval timer;
    timer.it_value.tv_sec = timeout_sec;
    timer.it_value.tv_usec = timeout * 1000;
    timer.it_interval.tv_sec = 1;
    timer.it_interval.tv_usec = 0;

    if (setitimer(ITIMER_REAL, &timer, NULL) == -1)
        ROS_ERROR("Failed to reset timer: %s", strerror(errno));
    
}

void SerialIMUNode::loadParameters() 
{
    pnh_.param<std::string>("imu_serial", serial_port_name_, "/dev/ttyUSB1");
    pnh_.param<int>("baud_rate", baud_rate_, 460800);
    pnh_.param<std::string>("frame_id", frame_id_, "imu_link");

    pnh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
    pnh_.param<std::string>("mag_topic", mag_topic_, "/imu/msg");
    pnh_.param<std::string>("eul_topic", eul_topic_, "/imu/eul");
    pnh_.param<std::string>("pre_topic", pre_topic_, "/imu/pre");
    pnh_.param<std::string>("temp_topic", temp_topic_, "/imu/temp");

    pnh_.param<bool>("imu_enable", imu_enable, false);
    pnh_.param<bool>("mag_enable", mag_enable, false);
    pnh_.param<bool>("eul_enable", eul_enable, false);
    pnh_.param<bool>("pre_enable", pre_enable, false);
    pnh_.param<bool>("temp_enable", temp_enable, false);

    pnh_.param<std::string>("imu_axes", imu_axes, "ENU");    
    pnh_.param<int>("port_timeout_ms", get_port_timeout_ms, 500);

    imu_msg.header.frame_id = frame_id_;
    mag_msg.header.frame_id = frame_id_;
    eul_msg.header.frame_id = frame_id_;
    pre_msg.header.frame_id = frame_id_;
    temp_msg.header.frame_id = frame_id_;

    timeout_sec = int(get_port_timeout_ms / 1000);
    timeout = get_port_timeout_ms % 1000;
}

void SerialIMUNode::initPublishers()
{
    imu_pub_   = pnh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);                  //acc gyr  quat
    eul_pub_   = pnh_.advertise<geometry_msgs::Vector3Stamped>(eul_topic_, 10);     //eul
    mag_pub_   = pnh_.advertise<sensor_msgs::MagneticField>(mag_topic_, 10);        //mag
    pre_pub_   = pnh_.advertise<sensor_msgs::FluidPressure>(pre_topic_, 10);        //pre
    temp_pub_  = pnh_.advertise<sensor_msgs::Temperature>(temp_topic_, 10);         //temp
}

void SerialIMUNode::initSerialPort()
{
    try
    {
        serial_port_ = std::make_unique<SerialPort>(
            serial_port_name_,
            baud_rate_
        );
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Failed to initialize: %s", e.what());
        ros::shutdown();
    }
}

void SerialIMUNode::initAlarm()
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = signal_handler;
    sa.sa_flags = SA_RESTART;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGALRM, &sa, NULL) == -1)
    {
        ROS_ERROR("Failed to set signal handler: %s", strerror(errno));
        return ;
    }

    memset(&timer, 0, sizeof(timer));

    timer.it_value.tv_sec = timeout_sec;
    timer.it_value.tv_usec = timeout * 1000;

    timer.it_interval.tv_sec = 1;
    timer.it_interval.tv_usec = 0;

    if (setitimer(ITIMER_REAL, &timer, NULL) == -1)
    {
        ROS_ERROR("Faild to set timer: %s", strerror(errno));
        return ;
    }

    ROS_INFO("Alarm initialized with timeout: %d.%03d seconds", timeout_sec, timeout);
}

void SerialIMUNode::publish_imu_data(hipnuc_raw_t *data, sensor_msgs::Imu *imu_data)
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
    else if(data->hi92.tag == 0x92)
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

void SerialIMUNode::publish_mag_data(hipnuc_raw_t *data, sensor_msgs::MagneticField *mag_data)
{
    if (data->hi91.tag == 0x91)
    {
        mag_data->magnetic_field.x = data->hi91.mag[0] / 10000.0;
        mag_data->magnetic_field.y = data->hi91.mag[1] / 10000.0;
        mag_data->magnetic_field.z = data->hi91.mag[2] / 10000.0;
    }
    else if(data->hi92.tag == 0x92)
    {
        mag_data->magnetic_field.x = data->hi92.mag_b[0] * MAG_FACTOR / 10000.0;
        mag_data->magnetic_field.y = data->hi92.mag_b[1] * MAG_FACTOR / 10000.0; 
        mag_data->magnetic_field.z = data->hi92.mag_b[2] * MAG_FACTOR / 10000.0;
    }
}

void SerialIMUNode::publish_eul_data(hipnuc_raw_t *data, geometry_msgs::Vector3Stamped *eul_data)
{
    if (data->hi91.tag == 0x91)
    {
        eul_data->vector.x = data->hi91.roll;
        eul_data->vector.y = data->hi91.pitch;
        eul_data->vector.z = data->hi91.yaw;
    }
    else if (data->hi92.tag == 0x92)
    {
        eul_data->vector.x = data->hi92.roll * EUL_FACTOR;
        eul_data->vector.y = data->hi92.pitch * EUL_FACTOR;
        eul_data->vector.z = data->hi92.yaw * EUL_FACTOR;
    }
}

void SerialIMUNode::publish_pre_data(hipnuc_raw_t *data, sensor_msgs::FluidPressure *pre_data)
{
    if (data->hi91.tag == 0x91)
    {
        pre_data->fluid_pressure = data->hi91.air_pressure;
    }
    else if (data->hi92.tag == 0x92)
    {
        pre_data->fluid_pressure = data->hi92.air_pressure + 100 * 1000;
    }
}

void SerialIMUNode::publish_temp_data(hipnuc_raw_t *data, sensor_msgs::Temperature *temp_data)
{
    if (data->hi91.tag == 0x91)
    {
        temp_data->temperature = data->hi91.temp;   
    }
    else if (data->hi92.tag ==  0x92)
    {
        temp_data->temperature = data->hi92.temperature;
    }
}

SerialIMUNode* SerialIMUNode::instance_ = nullptr;