#include <array>
#include <cassert>
#include <cstdint>
#include <limits>

#include "hipnuc_convert.hpp"
#ifdef __linux__
#include "socketcan.hpp"
#endif

// Minimal message-shaped fixtures keep conversion tests independent of ROS.
struct Vector { double x = 0, y = 0, z = 0; };
struct Quaternion { double x = 0, y = 0, z = 0, w = 0; };
struct Imu {
    Quaternion orientation;
    Vector angular_velocity, linear_acceleration;
    std::array<double, 9> orientation_covariance{}, angular_velocity_covariance{}, linear_acceleration_covariance{};
};
struct Magnetic {
    Vector magnetic_field;
    std::array<double, 9> magnetic_field_covariance{};
};
struct Temperature { double temperature = 0, variance = -1; };
struct Product {
    uint8_t source{};
    uint64_t valid{};
    uint8_t node_id{};
    uint16_t main_status{};
    bool gyro_bias_converged{};
    bool attitude_converged{};
    bool magnetic_disturbance{};
    bool device_static{};
    bool magnetometer_aiding{};
    uint8_t ins_status{};
    std::array<float, 3> acceleration{};
    std::array<float, 3> angular_velocity{};
    std::array<float, 3> magnetic_field{};
    float roll{};
    float pitch{};
    float yaw{};
    float heading{};
    std::array<float, 4> quaternion_wxyz{};
    std::array<float, 2> inclination{};
    uint64_t device_time_us{};
    bool utc_valid{};
    uint16_t utc_year{};
    uint8_t utc_month{};
    uint8_t utc_day{};
    uint8_t utc_hour{};
    uint8_t utc_minute{};
    uint8_t utc_second{};
    uint16_t utc_millisecond{};
    uint16_t gps_week{};
    uint32_t gps_tow_ms{};
    float temperature{};
    float pressure{};
    std::array<float, 3> heave_surge_sway{};
    std::array<float, 3> heave_surge_sway_frequency{};
    double longitude{};
    double latitude{};
    double altitude_msl{};
    double gnss_longitude{};
    double gnss_latitude{};
    double gnss_altitude_msl{};
    std::array<float, 3> gnss_velocity_enu{};
    float geoid_separation{};
    std::array<float, 3> velocity_enu{};
    std::array<float, 3> acceleration_enu{};
    uint8_t position_quality{};
    uint8_t position_satellites{};
    uint8_t heading_quality{};
    uint8_t heading_satellites{};
    float pdop{};
    float hdop{};
    float differential_age{};
    float odometer_speed{};
    float speed_over_ground{};
    float course_over_ground{};
    uint8_t nmea_status{};
    uint8_t nmea_mode{};
};

static void current_sample_only()
{
    hipnuc_sample_t sample{};
    Imu message;
    sample.valid = HIPNUC_VALID_ACC;
    sample.acc[0] = 1.25f;
    sample.acc[2] = 9.8f;
    sample.gyr[0] = 99; // invalid storage must not become a measurement
    sample.quat[0] = 0.5f;
    assert(hipnuc_ros::has_imu(sample));
    hipnuc_ros::fill_imu(sample, message);
    assert(message.linear_acceleration.x == 1.25);
    assert(message.angular_velocity.x == 0);
    assert(message.orientation_covariance[0] == -1);
    assert(message.angular_velocity_covariance[0] == -1);
    assert(message.linear_acceleration_covariance[0] == 0);
    sample.valid = HIPNUC_VALID_GYR;
    sample.gyr[1] = 0.125f;
    hipnuc_ros::fill_imu(sample, message);
    assert(message.linear_acceleration.x == 0);
    assert(message.linear_acceleration_covariance[0] == -1);
    assert(message.angular_velocity.y == 0.125);
    assert(message.angular_velocity_covariance[0] == 0);
    sample.valid = HIPNUC_VALID_ROLL_PITCH | HIPNUC_VALID_HEADING;
    sample.heading = 1;
    assert(!hipnuc_ros::has_imu(sample));
    hipnuc_ros::fill_imu(sample, message);
    assert(message.orientation_covariance[0] == -1);
    sample.valid = HIPNUC_VALID_QUAT;
    sample.quat[0] = 1;
    sample.quat[1] = sample.quat[2] = sample.quat[3] = 0;
    hipnuc_ros::fill_imu(sample, message);
    assert(message.orientation.w == 1);
    for (double covariance : message.orientation_covariance) assert(covariance == 0);
    sample.quat[0] = 0;
    assert(!hipnuc_ros::has_imu(sample));
    hipnuc_ros::fill_imu(sample, message);
    assert(message.orientation_covariance[0] == -1);
    sample.quat[0] = 0.9999f; // quantized identity remains a unit quaternion in ROS
    hipnuc_ros::fill_imu(sample, message);
    assert(message.orientation.w == 1.0);
}

static void independent_product_fields()
{
    hipnuc_sample_t sample{};
    sample.source = HIPNUC_SOURCE_HI83;
    sample.valid = HIPNUC_VALID_POSITION | HIPNUC_VALID_GNSS_POSITION |
                   HIPNUC_VALID_GNSS_ALTITUDE | HIPNUC_VALID_HEAVE_FREQUENCY |
                   HIPNUC_VALID_HDOP | HIPNUC_VALID_NMEA_MODE;
    sample.longitude = 121.5;
    sample.latitude = 31.5;
    sample.gnss_longitude = 121.6;
    sample.gnss_latitude = 31.6;
    sample.gnss_altitude_msl = 12.5;
    sample.gnss_vel_enu[1] = 0.75f;
    sample.heave_hz[2] = 0.25f;
    sample.hdop = 0.5f;
    sample.nmea_mode = 'E'; // preserve estimated mode; do not invent a fix
    sample.gps_week = 2200;
    sample.gps_tow_ms = 123456;
    sample.magnetometer_aiding = 1;
    Product message;
    hipnuc_ros::fill_hipnuc(sample, message);
    assert(message.valid == sample.valid);
    assert(message.valid > UINT32_MAX);
    assert(message.longitude == 121.5 && message.gnss_longitude == 121.6);
    assert(message.gnss_altitude_msl == 12.5);
    assert(message.heave_surge_sway_frequency[2] == 0.25f);
    assert(message.nmea_mode == 'E');
    assert(message.gps_week == 2200 && message.gps_tow_ms == 123456);
    assert(message.magnetometer_aiding);
    assert(!hipnuc_ros::has_imu(sample));
}

static void physical_units()
{
    hipnuc_sample_t sample{};
    sample.mag[0] = 0.00003f;
    sample.temperature = 25.5f;
    Magnetic magnetic;
    Temperature temperature;
    hipnuc_ros::fill_mag(sample, magnetic);
    hipnuc_ros::fill_temperature(sample, temperature);
    assert(magnetic.magnetic_field.x == sample.mag[0]);
    assert(temperature.temperature == 25.5 && temperature.variance == 0);
}

static void product_pressure_is_preserved_without_claiming_validity()
{
    hipnuc_sample_t sample{};
    sample.valid = HIPNUC_VALID_PRESSURE;
    Product message;
    // Zero can be an unimplemented sensor; positive values can also be stale.
    // The product message preserves both without asserting sensor validity.
    for (float pressure : {0.0f, 100676.0f}) {
        sample.pressure = pressure;
        hipnuc_ros::fill_hipnuc(sample, message);
        assert(message.pressure == pressure);
        assert(message.valid == HIPNUC_VALID_PRESSURE);
    }
}

static void nonfinite_measurements_are_not_estimates()
{
    hipnuc_sample_t sample{};
    Imu message;
    sample.valid = HIPNUC_VALID_ACC | HIPNUC_VALID_GYR;
    sample.acc[0] = std::numeric_limits<float>::quiet_NaN();
    sample.gyr[1] = std::numeric_limits<float>::infinity();
    assert(!hipnuc_ros::has_imu(sample));
    hipnuc_ros::fill_imu(sample, message);
    assert(message.linear_acceleration_covariance[0] == -1);
    assert(message.angular_velocity_covariance[0] == -1);
    sample.gyr[1] = 0.5f;
    assert(hipnuc_ros::has_imu(sample));
    hipnuc_ros::fill_imu(sample, message);
    assert(message.angular_velocity.y == 0.5);
    assert(message.linear_acceleration_covariance[0] == -1);
}

int main()
{
    current_sample_only();
    independent_product_fields();
    physical_units();
    product_pressure_is_preserved_without_claiming_validity();
    nonfinite_measurements_are_not_estimates();
#ifdef __linux__
    hipnuc_ros::SocketCan can;
    assert(!can.is_open());
    assert(!can.open(std::string(100, 'x')).empty());
    assert(!can.is_open());
    hipnuc_can_frame_t frame{};
    assert(can.read(frame, 0) == -1);
    can.close();
#endif
}
