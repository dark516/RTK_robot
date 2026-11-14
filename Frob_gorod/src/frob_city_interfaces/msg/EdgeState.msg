uint8 OPEN=0
uint8 TEMP_BLOCKED=1
uint8 HAZARD_BLOCKED=2

string edge_id
uint64 version
uint8 state
uint32 completed_traversals
builtin_interfaces/Time next_recheck_after
string reason
