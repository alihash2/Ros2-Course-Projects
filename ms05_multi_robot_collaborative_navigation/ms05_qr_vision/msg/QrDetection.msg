# One observed QR tag from a robot camera.
#
# Topic convention (one detector instance per robot, run in that
# namespace):  /robot1/qr/detections  and  /robot2/qr/detections
#
# Payload contract written by ms05_gazebo_worlds (markers/qr_markers.yaml):
#   ms05:<room>:<index>      e.g. "ms05:01:00"
# where <index> is the zero-padded marker index "00".."07" and equals
# marker_id; <room> is the room tag the marker belongs to.

std_msgs/Header header            # stamp = detection time; frame_id = camera optical frame
int32 marker_id                   # <index> from the payload parsed as int (0..7)
string payload                    # full decoded payload, e.g. "ms05:01:00"
string room                       # <room> from the payload
geometry_msgs/Point camera_point  # tag centre in the camera frame (x right, y down, z forward)
float32 range                     # euclidean distance to the tag centre [m]
float32 bearing                   # yaw of the tag centre vs the optical axis [rad], + = right
float32 yaw_error                 # rotation of the tag normal vs the optical axis [rad], 0 = facing camera
float32 confidence                # detector confidence proxy (sharpness of the tag region), 0..1
bool centered                     # range, bearing and yaw within tolerances for
                                  # centered_frames consecutive observations
