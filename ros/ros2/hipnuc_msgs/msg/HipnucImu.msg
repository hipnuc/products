# One decoded HiPNUC measurement in SI units. `valid` is the bit mask from
# hipnuc_sample.h (HIPNUC_VALID_*): read a field only when its bit is set.
std_msgs/Header header
uint8 source                 # 1 HI91, 2 HI81, 3 HI83, 4 GGA, 5 RMC, 6 J1939, 7 CANFD83
uint32 valid
uint8 node_id                # CAN source address or HI83 node id

uint16 main_status           # raw status bits (a set WB_CONV/ATT_CONV bit means NOT converged)
bool gyro_bias_converged
bool attitude_converged
bool magnetic_disturbance
bool device_static
uint8 ins_status             # 0 invalid, 1 aligning, 3 navigating, 6 dead reckoning

float32[3] acceleration      # m/s^2, body axes, specific force
float32[3] angular_velocity  # rad/s, body axes
float32[3] magnetic_field    # T
float32 roll                 # rad, device Euler convention
float32 pitch
float32 yaw
float32 heading              # rad, clockwise from north (INS / J1939)
float32[4] quaternion_wxyz   # body to navigation
float32[2] inclination       # rad

uint64 device_time_us
bool utc_valid
uint16 utc_year
uint8 utc_month
uint8 utc_day
uint8 utc_hour
uint8 utc_minute
uint8 utc_second
uint16 utc_millisecond

float32 temperature          # degC
float32 pressure             # Pa
float32[3] heave_surge_sway  # m

float64 longitude            # deg, WGS84
float64 latitude
float64 altitude_msl         # m above mean sea level
float32 geoid_separation     # m; ellipsoid height = altitude_msl + geoid_separation
float32[3] velocity_enu      # m/s
float32[3] acceleration_enu  # m/s^2
uint8 position_quality       # GGA quality: 0 none, 1 GNSS, 2 DGNSS, 4 RTK fixed, 5 RTK float
uint8 position_satellites
uint8 heading_quality
uint8 heading_satellites
float32 pdop
float32 hdop
float32 differential_age     # s
float32 odometer_speed       # m/s
