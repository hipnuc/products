// Conversion from hipnuc_sample_t to ROS messages. Shared by the ROS 1 and
// ROS 2 packages: the message types are template parameters because the
// field names are identical in both generations.
//
// Conventions (REP 103 / sensor_msgs):
//   * Imu: orientation is the device quaternion (body -> navigation, WXYZ on
//     the wire, xyzw here); angular_velocity rad/s; linear_acceleration is
//     specific force in m/s^2 (gravity not removed). A covariance whose first
//     element is -1 means "not provided"; all zeros means "unknown".
//   * NavSatFix: WGS84, altitude above the ellipsoid = MSL + geoid
//     separation; NaN when the separation is unknown. Status is derived from
//     the GGA quality code.
//   * The time stamp is set by the node from its own clock.

#ifndef HIPNUC_ROS_CONVERT_HPP
#define HIPNUC_ROS_CONVERT_HPP

#include <cmath>
#include <cstdint>
#include <limits>

#include "hipnuc_sample.h"

namespace hipnuc_ros {

// GGA quality -> sensor_msgs NavSatStatus.status constants (-1, 0, 1, 2).
inline int8_t navsat_status(uint8_t gga_quality)
{
    switch (gga_quality) {
    case 0: return -1;      // STATUS_NO_FIX
    case 1: return 0;       // STATUS_FIX
    case 2: return 1;       // STATUS_SBAS_FIX (differential)
    case 4: case 5: return 2;  // STATUS_GBAS_FIX (RTK fixed / float)
    default: return 0;
    }
}

template <class Imu>
void fill_imu(const hipnuc_sample_t &s, Imu &m)
{
    if (s.valid & HIPNUC_VALID_QUAT) {
        m.orientation.w = s.quat[0];
        m.orientation.x = s.quat[1];
        m.orientation.y = s.quat[2];
        m.orientation.z = s.quat[3];
        m.orientation_covariance[0] = 0.0;
    } else {
        m.orientation.w = 1.0;
        m.orientation.x = m.orientation.y = m.orientation.z = 0.0;
        m.orientation_covariance[0] = -1.0;
    }
    if (s.valid & HIPNUC_VALID_GYR) {
        m.angular_velocity.x = s.gyr[0];
        m.angular_velocity.y = s.gyr[1];
        m.angular_velocity.z = s.gyr[2];
        m.angular_velocity_covariance[0] = 0.0;
    } else {
        m.angular_velocity_covariance[0] = -1.0;
    }
    if (s.valid & HIPNUC_VALID_ACC) {
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
}

template <class Temperature>
void fill_temperature(const hipnuc_sample_t &s, Temperature &m)
{
    m.temperature = s.temperature;
    m.variance = 0.0;
}

template <class FluidPressure>
void fill_pressure(const hipnuc_sample_t &s, FluidPressure &m)
{
    m.fluid_pressure = s.pressure;
    m.variance = 0.0;
}

template <class NavSatFix>
void fill_navsatfix(const hipnuc_sample_t &s, NavSatFix &m)
{
    m.status.status = (s.valid & HIPNUC_VALID_GNSS_QUALITY) ? navsat_status(s.position_quality) : 0;
    m.status.service = 1;  // SERVICE_GPS; the device does not report constellations
    m.latitude = s.latitude;
    m.longitude = s.longitude;
    m.altitude = (s.valid & HIPNUC_VALID_UNDULATION) ? s.altitude_msl + s.undulation
                                                     : std::numeric_limits<double>::quiet_NaN();
    m.position_covariance_type = 0;  // COVARIANCE_TYPE_UNKNOWN
}

template <class TwistWithCovarianceStamped>
void fill_velocity_enu(const hipnuc_sample_t &s, TwistWithCovarianceStamped &m)
{
    m.twist.twist.linear.x = s.vel_enu[0];
    m.twist.twist.linear.y = s.vel_enu[1];
    m.twist.twist.linear.z = s.vel_enu[2];
    m.twist.covariance[0] = -1.0;  // unknown by convention of this driver
}

// Full-field product message (HipnucImu.msg). Fields are copied when their
// validity bit is set and left at their default (0) otherwise.
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
    m.ins_status = s.ins_status;
    for (int i = 0; i < 3; ++i) {
        m.acceleration[i] = s.acc[i];
        m.angular_velocity[i] = s.gyr[i];
        m.magnetic_field[i] = s.mag[i];
        m.velocity_enu[i] = s.vel_enu[i];
        m.acceleration_enu[i] = s.acc_enu[i];
        m.heave_surge_sway[i] = s.heave_m[i];
    }
    m.roll = s.roll;
    m.pitch = s.pitch;
    m.yaw = s.yaw;
    m.heading = s.heading;
    for (int i = 0; i < 4; ++i) m.quaternion_wxyz[i] = s.quat[i];
    m.inclination[0] = s.inclination[0];
    m.inclination[1] = s.inclination[1];
    m.device_time_us = s.device_time_us;
    m.utc_valid = (s.valid & HIPNUC_VALID_UTC) != 0;
    m.utc_year = s.utc.year;
    m.utc_month = s.utc.month;
    m.utc_day = s.utc.day;
    m.utc_hour = s.utc.hour;
    m.utc_minute = s.utc.minute;
    m.utc_second = s.utc.second;
    m.utc_millisecond = s.utc.millisecond;
    m.temperature = s.temperature;
    m.pressure = s.pressure;
    m.longitude = s.longitude;
    m.latitude = s.latitude;
    m.altitude_msl = s.altitude_msl;
    m.geoid_separation = s.undulation;
    m.position_quality = s.position_quality;
    m.position_satellites = s.position_satellites;
    m.heading_quality = s.heading_quality;
    m.heading_satellites = s.heading_satellites;
    m.pdop = s.pdop;
    m.hdop = s.hdop;
    m.differential_age = s.diff_age;
    m.odometer_speed = s.odometer_speed;
}

}  // namespace hipnuc_ros

#endif  // HIPNUC_ROS_CONVERT_HPP
