uint8 GEAR_P=0
uint8 GEAR_D=1
uint8 GEAR_N=2
uint8 GEAR_R=3
uint8 GEAR_UNKNOWN=255

uint8 DRIVE_MODE_REMOTE=0
uint8 DRIVE_MODE_AD=1
uint8 DRIVE_MODE_PARALLEL=2
uint8 DRIVE_MODE_SEMI_AUTONOMOUS=3
uint8 DRIVE_MODE_UNKNOWN=255
uint8 COUNTER_UNKNOWN=255

# Message assembly time, not a CAN hardware timestamp.
std_msgs/Header header

# 0x304: measured speed (m/s), datasheet steering sign (rad), pressure (Pa).
float64 speed_mps
float64 steering_angle_rad
float64 brake_pressure_pa

# Existing fields: aliases of status_304_received/valid/age_sec.
bool speed_received
bool speed_valid
float64 speed_age_sec

# 0x303: mode values 4..15 are preserved raw and have no named meaning here.
uint8 gear
uint8 drive_mode
bool brake_hold_active
float64 requested_speed_mps
# State of charge as a fraction in [0, 1].
float64 battery_soc

# 0x301: false before first reception means unknown, not confirmed normal.
# Always check status_301_valid before interpreting these fields.
float64 battery_voltage_v
bool emergency_stop_pressed
bool motor_fault
bool remote_fault
bool ad_signal_received
bool dbs_fault
bool parallel_driving_active
bool bms_fault
bool eps_fault
bool front_bumper_triggered
bool rear_bumper_triggered
bool headlight_on
bool left_indicator_on
bool brake_light_on
bool oil_level_abnormal
bool horn_on
bool right_indicator_on

# Per-ID freshness. Stamps use the node ROS clock after a socket read.
# Ages use steady_clock and exclude time queued before the socket read.
# valid means a decoded sample is fresh, not that the robot is fault-free.
# Before first accepted frame: received=false, valid=false, stamp=0,
# age=NaN, counter=255. Counter discontinuities do not change freshness.
bool status_301_received
bool status_301_valid
builtin_interfaces/Time status_301_stamp
float64 status_301_age_sec
uint8 status_301_counter
uint64 status_301_counter_discontinuities
uint64 status_301_rejected_frames

bool status_303_received
bool status_303_valid
builtin_interfaces/Time status_303_stamp
float64 status_303_age_sec
uint8 status_303_counter
uint64 status_303_counter_discontinuities
uint64 status_303_rejected_frames

bool status_304_received
bool status_304_valid
builtin_interfaces/Time status_304_stamp
float64 status_304_age_sec
uint8 status_304_counter
uint64 status_304_counter_discontinuities
uint64 status_304_rejected_frames

# 0x10A: raw remote A switch value, initialized to 255 before reception.
# Other raw values are preserved. selected means raw == 1, not CAN ownership.
# Values remain cached when stale; always check status_10a_valid.
uint8 remote_a_raw
bool remote_a_selected

# 0x10A defines no message counter. Freshness uses feedback_timeout_ms.
bool status_10a_received
bool status_10a_valid
builtin_interfaces/Time status_10a_stamp
float64 status_10a_age_sec
uint64 status_10a_rejected_frames
