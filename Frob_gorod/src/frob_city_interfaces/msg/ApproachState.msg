uint8 UNKNOWN=0
uint8 OBSERVING=1
uint8 RESOLVED_CLEAR=2
uint8 RESOLVED_SIGNED=3
uint8 CONFLICT=4
uint8 BLOCKED=5

# Movement-mask bits from the frozen architecture contract.
uint8 MOVE_STRAIGHT=1
uint8 MOVE_LEFT=2
uint8 MOVE_RIGHT=4

string approach_id
uint64 version
uint8 state
uint8[] confirmed_sign_classes
uint8 allowed_movement_mask
builtin_interfaces/Time last_qualified_observation
bool precautionary_hold
string reason
