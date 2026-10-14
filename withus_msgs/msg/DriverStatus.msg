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
# hardware_drive keeps MONITORING while broadcasting request=1.
# Its state describes the 0x501 request; use drive_* for actuator state.
# vcan_reference uses state for the virtual vehicle control state machine.
string control_profile
# Current 0x501 request intent, not proof of CAN delivery or vehicle mode.
bool control_requested
# hardware_drive only. These describe driver intent and observed feedback,
# not a physical stopping guarantee. state/control_requested still describe 0x501.
bool drive_ready
bool drive_active
bool drive_mode_confirmed
bool drive_gear_pending
string drive_reason
