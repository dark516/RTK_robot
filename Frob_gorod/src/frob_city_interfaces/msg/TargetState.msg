uint8 PROVISIONAL=0
uint8 CONFIRMED=1
uint8 CONFLICT=2

# Target kind uses the canonical sign-class numbers.
uint8 BUS_STOP=6
uint8 PARKING=7

string target_id
uint64 version
uint8 kind
uint8 state
string site_id
string face_id
string directed_road_id
string service_anchor_id
geometry_msgs/PoseStamped sign_pose
bool geometry_valid
geometry_msgs/PoseStamped service_pose
geometry_msgs/PoseStamped terminal_limit
float32 xy_tolerance_m
float32 yaw_tolerance_rad
float32 front_clearance_min_m
builtin_interfaces/Time last_seen
