uint8 UNKNOWN=0
uint8 COMPLETE=1
uint8 INCONCLUSIVE=2
uint8 UNREACHABLE_NOW=3

string coverage_id
uint64 version
uint8 state
uint32 failed_attempts
builtin_interfaces/Time last_qualified_observation
string reason
