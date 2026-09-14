# NavigationMission.msg
# Mode constants
uint8 MODE_GO_TO_POSE = 0
uint8 MODE_WAYPOINTS = 1

# Command constants
uint8 COMMAND_START = 0
uint8 COMMAND_CANCEL = 1
uint8 COMMAND_PAUSE = 2
uint8 COMMAND_RESUME = 3
uint8 COMMAND_REPLACE = 4

# Header and identification
std_msgs/Header header
string mission_id
uint8 mode
uint8 command

# Target pose for single-goal navigation (MODE_GO_TO_POSE)
geometry_msgs/PoseStamped target_pose

# Waypoint list for multi-waypoint navigation (MODE_WAYPOINTS)
geometry_msgs/PoseStamped[] waypoints
