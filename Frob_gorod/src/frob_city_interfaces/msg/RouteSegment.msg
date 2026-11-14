uint8 CORRIDOR=0
uint8 JUNCTION=1
uint8 SERVICE_APPROACH=2
uint8 OBSERVATION_ADJUST=3

uint8 HOLD=0
uint8 OBSERVE=1
uint8 BUS_SERVICE=2
uint8 PARK=3

# selected_movement uses the same movement-mask bits as ApproachState.
uint8 MOVE_STRAIGHT=1
uint8 MOVE_LEFT=2
uint8 MOVE_RIGHT=4

string segment_id
uint8 kind
string from_anchor_id
string to_anchor_id
string approach_id
uint8 selected_movement
nav_msgs/Path path
geometry_msgs/PolygonStamped[] allowed_regions
string[] dependency_ids
uint64[] dependency_versions
string[] traversed_edge_ids
uint8 terminal_kind
string terminal_id
geometry_msgs/PoseStamped terminal_limit
bool commitment_gate_valid
geometry_msgs/PoseStamped commitment_gate
float32 max_speed_mps
float32 max_yaw_rate_radps
float32 xy_tolerance_m
float32 yaw_tolerance_rad
float32 estimated_duration_s
