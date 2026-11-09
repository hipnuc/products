# One decoded HiPNUC measurement in SI units. `valid` is the bit mask from
# hipnuc_data.h (HIPNUC_VALID_*): read a field only when its bit is set.
# Bits indicate field presence, not solution convergence or a valid GNSS fix.
# Fields without a set bit are placeholders. INS and GNSS origins may differ.
uint64 VALID_STATUS=1
uint64 VALID_INS_STATUS=2
uint64 VALID_NODE_ID=4
uint64 VALID_SEQUENCE=8
uint64 VALID_ACC=16
uint64 VALID_GYR=32
uint64 VALID_MAG=64
uint64 VALID_ROLL_PITCH=128
uint64 VALID_YAW=256
uint64 VALID_HEADING=512
uint64 VALID_QUAT=1024
uint64 VALID_INCLINATION=2048
uint64 VALID_INCLINATION_YAW=4096
uint64 VALID_ATT_STD=8192
uint64 VALID_SYSTEM_TIME=16384
uint64 VALID_DEVICE_TIME=32768
uint64 VALID_UTC=65536
uint64 VALID_UTC_TIME_OF_DAY=131072
uint64 VALID_GPS_TIME=262144
uint64 VALID_PRESSURE=524288
uint64 VALID_TEMPERATURE=1048576
uint64 VALID_HEAVE=2097152
uint64 VALID_HEAVE_FREQUENCY=4194304
uint64 VALID_POSITION=8388608
uint64 VALID_ALTITUDE=16777216
uint64 VALID_VELOCITY_ENU=33554432
uint64 VALID_ACC_ENU=67108864
uint64 VALID_SOG=134217728
uint64 VALID_COG=268435456
uint64 VALID_GNSS_POSITION=536870912
uint64 VALID_GNSS_ALTITUDE=1073741824
uint64 VALID_GNSS_VELOCITY=2147483648
uint64 VALID_POSITION_QUALITY=4294967296
uint64 VALID_POSITION_SATELLITES=8589934592
uint64 VALID_HEADING_QUALITY=17179869184
uint64 VALID_HEADING_SATELLITES=34359738368
uint64 VALID_NMEA_STATUS=68719476736
uint64 VALID_NMEA_MODE=137438953472
uint64 VALID_PDOP=274877906944
uint64 VALID_HDOP=549755813888
uint64 VALID_DIFF_AGE=1099511627776
uint64 VALID_UNDULATION=2199023255552

std_msgs/Header header         # time the device began sending the frame; body-axis frame_id
uint8 source                 # 1 HI91, 2 HI81, 3 HI83, 4 GGA, 5 RMC, 6 J1939, 7 CANFD83
uint64 valid
uint8 node_id                # HI83 node id
uint8 sequence               # CANFD83 frame counter, VALID_SEQUENCE

uint16 main_status           # raw status bits (a set WB_CONV/ATT_CONV bit means NOT converged)
bool gyro_bias_converged
bool attitude_converged
bool magnetic_disturbance
bool device_static
bool magnetometer_aiding
uint8 ins_status             # 0 invalid, 1 aligning, 3 navigating, 6 dead reckoning

float32[3] acceleration      # m/s^2, body axes, specific force
float32[3] angular_velocity  # rad/s, body axes
float32[3] magnetic_field    # T
float32 roll                 # rad, device Euler convention
float32 pitch
float32 yaw
float32 heading              # rad, clockwise from north (INS)
float32[4] quaternion_wxyz   # body to navigation
float32[2] inclination       # rad
float32 inclination_yaw      # rad, HI83 inclinometer yaw, VALID_INCLINATION_YAW
float32[3] attitude_std      # rad, 1-sigma about the navigation axes; [2] heading, VALID_ATT_STD

uint32 system_time_ms        # HI91: UTC ms of day once synchronized, else uptime ms; wraps at 24 h, VALID_SYSTEM_TIME
uint64 device_time_us        # HI83: device time since power-up, VALID_DEVICE_TIME
bool utc_valid               # VALID_UTC, date and time; VALID_UTC_TIME_OF_DAY alone leaves the date zero
uint16 utc_year
uint8 utc_month
uint8 utc_day
uint8 utc_hour
uint8 utc_minute
uint8 utc_second
uint16 utc_millisecond
uint16 gps_week
uint32 gps_tow_ms

float32 temperature          # degC
float32 pressure             # Pa; raw product field, validity/freshness not guaranteed
float32[3] heave_surge_sway  # m, VALID_HEAVE
float32[3] heave_surge_sway_frequency # Hz, VALID_HEAVE_FREQUENCY

float64 longitude            # INS origin, deg WGS84, VALID_POSITION
float64 latitude
float64 altitude_msl         # INS origin, m MSL, VALID_ALTITUDE
float64 gnss_longitude        # GNSS antenna, deg WGS84, VALID_GNSS_POSITION
float64 gnss_latitude
float64 gnss_altitude_msl     # GNSS antenna, m MSL, VALID_GNSS_ALTITUDE
float32[3] gnss_velocity_enu  # m/s, VALID_GNSS_VELOCITY
float32 geoid_separation     # GNSS geoid separation, m
float32[3] velocity_enu      # m/s
float32[3] acceleration_enu  # m/s^2
uint8 position_quality       # GGA quality: 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float
uint8 position_satellites
uint8 heading_quality        # 4 = fixed; only then is a dual-antenna heading valid
uint8 heading_satellites
float32 pdop
float32 hdop
float32 differential_age     # s
float32 speed_over_ground    # m/s, VALID_SOG
float32 course_over_ground   # rad clockwise from north, VALID_COG
uint8 nmea_status             # ASCII A/V, VALID_NMEA_STATUS
uint8 nmea_mode               # raw ASCII RMC mode, VALID_NMEA_MODE
