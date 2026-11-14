uint8 UNKNOWN=0
uint8 STRAIGHT_ONLY=1
uint8 LEFT_ONLY=2
uint8 RIGHT_ONLY=3
uint8 NO_LEFT=4
uint8 NO_RIGHT=5
uint8 BUS_STOP=6
uint8 PARKING=7
uint8 HAZARD_MARKER=8

uint8 RANGE_UNKNOWN=0
uint8 OPTICAL_Z=1
uint8 RAY_RANGE=2

uint8 sign_class
float32 confidence
uint32 x_min_px
uint32 y_min_px
uint32 width_px
uint32 height_px
bool range_valid
uint8 range_convention
float32 range_m
float32 range_stddev_m
string detector_track_id
