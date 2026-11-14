# Mission phase values.
uint8 PHASE_IDLE=0
uint8 PHASE_BOARDING=1
uint8 PHASE_PARKING_PRIOR=2
uint8 PHASE_PARKING_SEARCH=3
uint8 PHASE_PARKING_APPROACH=4
uint8 PHASE_DONE=5
uint8 PHASE_PAUSED=6
uint8 PHASE_ABORTED=7

# Segment-execution state values.
uint8 EXEC_HOLDING=0
uint8 EXEC_REQUESTING_PLAN=1
uint8 EXEC_VALIDATING_SEGMENT=2
uint8 EXEC_SENDING_GOAL=3
uint8 EXEC_MOVING=4
uint8 EXEC_CANCELING=5
uint8 EXEC_OBSERVING=6
uint8 EXEC_VERIFYING_STOP=7
uint8 EXEC_DWELLING=8

# Parking-prior state values.
uint8 PARKING_PRIOR_NOT_CHECKED=0
uint8 PARKING_PRIOR_FOUND=1
uint8 PARKING_PRIOR_CLEAR_ABSENT=2
uint8 PARKING_PRIOR_INCONCLUSIVE=3
uint8 PARKING_PRIOR_UNREACHABLE_NOW=4
uint8 PARKING_PRIOR_INVALID=5

std_msgs/Header header
string run_id
string navigator_instance_id
uint8 phase
uint8 execution_state
string objective_id
string route_id
string active_segment_id
uint64 goal_generation
uint32 required_unique_bus_stops
string[] serviced_stop_ids
float32 current_dwell_s
uint8 parking_prior_state
bool attempt_tainted
string reason
