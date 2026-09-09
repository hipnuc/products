// Conversion from hipnuc_sample_t to ROS messages. Shared by the ROS 1 and
// ROS 2 packages: the message types are template parameters because the
// field names are identical in both generations.
//
// Conventions (REP 103 / sensor_msgs):
//   * Imu: orientation is the device quaternion (body -> navigation, WXYZ on
//     the wire, xyzw here); angular_velocity rad/s; linear_acceleration is
//     specific force in m/s^2 (gravity not removed). A covariance whose first
//     element is -1 means "not provided"; all zeros means "unknown".
//   * Standard orientation requires ENU device configuration.
//   * The node stamps each decoded sample with its ROS clock before publishing;
//     this is neither a kernel receive timestamp nor the device sampling time.

#ifndef HIPNUC_ROS_CONVERT_HPP
#define HIPNUC_ROS_CONVERT_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "hipnuc_sample.h"

namespace hipnuc_ros {

inline bool finite_vector(const float values[3])
{
    return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

// A quaternion's scale does not change its rotation. Normalize finite nonzero
// values for ROS; zero/nonfinite values cannot provide orientation.
inline double quaternion_norm(const hipnuc_sample_t &s)
{
    if (!(s.valid & HIPNUC_VALID_QUAT)) return 0.0;
    double squared = 0.0;
    for (float value : s.quat) {
        if (!std::isfinite(value)) return 0.0;
        squared += static_cast<double>(value) * value;
    }
    return std::sqrt(squared);
}

// Measurements belong to this sample only; heading cannot supply orientation.
// sensor_msgs/Imu permits partial measurements; fill_imu marks missing quantities
// with -1 covariance. Consumers must support those markers or require full input.
inline bool has_imu(const hipnuc_sample_t &s)
{
    const bool acc = (s.valid & HIPNUC_VALID_ACC) && finite_vector(s.acc);
    const bool gyr = (s.valid & HIPNUC_VALID_GYR) && finite_vector(s.gyr);
    return acc || gyr || quaternion_norm(s) > 0.0;
}

template <class Imu>
void fill_imu(const hipnuc_sample_t &s, Imu &m)
{
    std::fill(m.orientation_covariance.begin(), m.orientation_covariance.end(), 0.0);
    std::fill(m.angular_velocity_covariance.begin(), m.angular_velocity_covariance.end(), 0.0);
    std::fill(m.linear_acceleration_covariance.begin(), m.linear_acceleration_covariance.end(), 0.0);
    m.angular_velocity.x = m.angular_velocity.y = m.angular_velocity.z = 0.0;
    m.linear_acceleration.x = m.linear_acceleration.y = m.linear_acceleration.z = 0.0;
    const double norm = quaternion_norm(s);
    if (norm > 0.0) {
        m.orientation.w = s.quat[0] / norm;
        m.orientation.x = s.quat[1] / norm;
        m.orientation.y = s.quat[2] / norm;
        m.orientation.z = s.quat[3] / norm;
        m.orientation_covariance[0] = 0.0;
    } else {
        m.orientation.w = 1.0;
        m.orientation.x = m.orientation.y = m.orientation.z = 0.0;
        m.orientation_covariance[0] = -1.0;
    }
    if ((s.valid & HIPNUC_VALID_GYR) && finite_vector(s.gyr)) {
        m.angular_velocity.x = s.gyr[0];
        m.angular_velocity.y = s.gyr[1];
        m.angular_velocity.z = s.gyr[2];
        m.angular_velocity_covariance[0] = 0.0;
    } else {
        m.angular_velocity_covariance[0] = -1.0;
    }
    if ((s.valid & HIPNUC_VALID_ACC) && finite_vector(s.acc)) {
        m.linear_acceleration.x = s.acc[0];
        m.linear_acceleration.y = s.acc[1];
        m.linear_acceleration.z = s.acc[2];
        m.linear_acceleration_covariance[0] = 0.0;
    } else {
        m.linear_acceleration_covariance[0] = -1.0;
    }
}

template <class MagneticField>
void fill_mag(const hipnuc_sample_t &s, MagneticField &m)
{
    m.magnetic_field.x = s.mag[0];
    m.magnetic_field.y = s.mag[1];
    m.magnetic_field.z = s.mag[2];
    std::fill(m.magnetic_field_covariance.begin(), m.magnetic_field_covariance.end(), 0.0);
}

template <class Temperature>
void fill_temperature(const hipnuc_sample_t &s, Temperature &m)
{
    m.temperature = s.temperature;
    m.variance = 0.0;
}

// Product message: preserve the decoder's independent field-presence bits.
// Values without their bit are placeholders, never measurements.
template <class HipnucImu>
void fill_hipnuc(const hipnuc_sample_t &s, HipnucImu &m)
{
    m.source = static_cast<uint8_t>(s.source);
    m.valid = s.valid;
    m.node_id = s.node_id;
    m.main_status = s.main_status;
    m.gyro_bias_converged = s.gyro_bias_converged != 0;
    m.attitude_converged = s.attitude_converged != 0;
    m.magnetic_disturbance = s.magnetic_disturbance != 0;
    m.device_static = s.device_static != 0;
    m.magnetometer_aiding = s.magnetometer_aiding != 0;
    m.ins_status = s.ins_status;
    for (int i = 0; i < 3; ++i) {
        m.acceleration[i] = s.acc[i];
        m.angular_velocity[i] = s.gyr[i];
        m.magnetic_field[i] = s.mag[i];
        m.velocity_enu[i] = s.vel_enu[i];
        m.acceleration_enu[i] = s.acc_enu[i];
        m.heave_surge_sway[i] = s.heave_m[i];
        m.heave_surge_sway_frequency[i] = s.heave_hz[i];
        m.gnss_velocity_enu[i] = s.gnss_vel_enu[i];
    }
    m.roll = s.roll;
    m.pitch = s.pitch;
    m.yaw = s.yaw;
    m.heading = s.heading;
    for (int i = 0; i < 4; ++i) m.quaternion_wxyz[i] = s.quat[i];
    m.inclination[0] = s.inclination[0];
    m.inclination[1] = s.inclination[1];
    m.inclination_yaw = s.inclination_yaw;
    m.device_time_us = s.device_time_us;
    m.utc_valid = (s.valid & HIPNUC_VALID_UTC) != 0;
    m.utc_year = s.utc.year;
    m.utc_month = s.utc.month;
    m.utc_day = s.utc.day;
    m.utc_hour = s.utc.hour;
    m.utc_minute = s.utc.minute;
    m.utc_second = s.utc.second;
    m.utc_millisecond = s.utc.millisecond;
    m.gps_week = s.gps_week;
    m.gps_tow_ms = s.gps_tow_ms;
    m.temperature = s.temperature;
    m.pressure = s.pressure;
    m.longitude = s.longitude;
    m.latitude = s.latitude;
    m.altitude_msl = s.altitude_msl;
    m.gnss_longitude = s.gnss_longitude;
    m.gnss_latitude = s.gnss_latitude;
    m.gnss_altitude_msl = s.gnss_altitude_msl;
    m.geoid_separation = s.undulation;
    m.position_quality = s.position_quality;
    m.position_satellites = s.position_satellites;
    m.heading_quality = s.heading_quality;
    m.heading_satellites = s.heading_satellites;
    m.pdop = s.pdop;
    m.hdop = s.hdop;
    m.differential_age = s.diff_age;
    m.odometer_speed = s.odometer_speed;
    m.speed_over_ground = s.sog;
    m.course_over_ground = s.cog;
    m.nmea_status = static_cast<uint8_t>(s.nmea_status);
    m.nmea_mode = static_cast<uint8_t>(s.nmea_mode);
}

}  // namespace hipnuc_ros

#endif  // HIPNUC_ROS_CONVERT_HPP
