# NavigationStatus.msg
# State constants
uint8 STATE_IDLE = 0
uint8 STATE_NAVIGATING = 1
uint8 STATE_PAUSED = 2
uint8 STATE_COMPLETED = 3
uint8 STATE_CANCELED = 4
uint8 STATE_ABORTED = 5
uint8 STATE_REJECTED = 6

# Mission mode constants (mirror NavigationMission)
uint8 MODE_GO_TO_POSE = 0
uint8 MODE_WAYPOINTS = 1

std_msgs/Header header
string mission_id
uint8 state
uint8 mode

# Telemetry snapshot
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
builtin_interfaces/Duration estimated_time_remaining
uint32 current_waypoint
uint32 total_waypoints
string message