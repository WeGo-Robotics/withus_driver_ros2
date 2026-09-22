uint8 DISCONNECTED=0
uint8 MONITORING=1
uint8 REQUESTING=2
uint8 ACTIVE=3
uint8 STOPPING=4
uint8 FAULT=5
std_msgs/Header header
uint8 state
string reason
bool can_connected
bool enable_tx
bool feedback_valid
uint64 session_id
float64 command_age_sec
uint64 rejected_commands
uint64 tx_sent_frames
uint64 tx_failed_frames
string last_rejection
# In hardware_mode_only, ACTIVE means AD mode was observed.
# hardware_remote_mode normally uses MONITORING and control_requested;
# actual vehicle mode is reported independently by VehicleStatus.drive_mode.
# STOPPING in either mode-only profile sends only a bounded request=0 burst.
string control_profile
# Current 0x501 request intent, not proof of CAN delivery or vehicle mode.
# hardware_remote_mode keeps MONITORING while broadcasting request=1.
bool control_requested
# Stationary brake test only; other profiles report false/empty/NaN.
bool brake_test_ready
bool brake_test_active
string brake_test_reason
float64 brake_test_command_age_sec
