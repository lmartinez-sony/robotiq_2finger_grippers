# See Robotiq documentation for fields documentation.
std_msgs/Header header
bool is_ready
bool is_reset
bool is_moving
bool obj_detected
uint8 fault_status
float32 position     #[m] Distance in between fingers 
float32 requested_position
float32 current 