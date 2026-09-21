# ROS Header for time stamp
std_msgs/Header header

# speed (m/s)
float64 speed_mps

# check if the data has been received
bool speed_received

# the time elapsed after the final speed received
float64 speed_age_sec