# Whole command for one active session. CAN-native steering sign.
uint8 GEAR_PARK=0
uint8 GEAR_DRIVE=1
uint8 GEAR_NEUTRAL=2
uint8 GEAR_REVERSE=3
std_msgs/Header header
uint64 session_id
uint8 gear
# Nonnegative speed magnitude. Reverse is selected with gear.
float64 speed_mps
float64 steering_angle_rad
float64 brake_percent
