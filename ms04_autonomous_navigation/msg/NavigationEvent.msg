# NavigationEvent.msg
# Event types
uint8 EVENT_GOAL_SUBMITTED = 0
uint8 EVENT_GOAL_ACCEPTED = 1
uint8 EVENT_GOAL_REJECTED = 2
uint8 EVENT_FEEDBACK = 3
uint8 EVENT_GOAL_COMPLETED = 4
uint8 EVENT_GOAL_CANCELED = 5
uint8 EVENT_GOAL_ABORTED = 6
uint8 EVENT_GOAL_PAUSED = 7
uint8 EVENT_GOAL_RESUMED = 8
uint8 EVENT_GOAL_REPLACED = 9

std_msgs/Header header
string mission_id
uint8 event_type
string message

# Telemetry data
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
uint32 current_waypoint
uint32 total_waypoints
