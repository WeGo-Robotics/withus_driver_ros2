# Stationary brake response experiment, not a drive or emergency-stop command.
# Only hardware_brake_test accepts this message.
std_msgs/Header header
# Current DriverStatus.session_id; changes when a test ends or is interrupted.
uint64 session_id
# Pedal request, 0..100 percent. Not a pressure or verified stopping value.
float64 brake_percent
