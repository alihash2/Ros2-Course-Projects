# One QR tag observation registered into the shared database.
#
# Stored by qr_tag_database_node.py (ms05_auto_mapping) and queried by
# Robot 2's navigator. Both poses are captured at record time: the tag's
# pose in Map 1 (what Robot 2 localises against) and the recording
# robot's own pose (what makes the record auditable). header.frame_id
# MUST be "map" - the database rejects anything else.

int32 marker_id                         # <index> from payload ms05:<room>:<index>
std_msgs/Header header                  # frame_id: map
geometry_msgs/PoseStamped tag_map_pose      # tag pose in Map 1
geometry_msgs/PoseStamped robot_map_pose    # recording robot's pose in Map 1
builtin_interfaces/Time recorded_at     # when the record was written
string source_robot                     # e.g. "robot1"
float32 range_at_capture                # detector range when recorded [m]
string room                             # <room> from the payload
