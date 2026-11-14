# One traffic-sign detection from a video frame.

int32 class_id
string class_name
float32 confidence

# distance_cm is valid only when has_distance is true.
bool has_distance
float32 distance_cm

# Bounding box and centre in source-image pixels.
int32 x_min
int32 y_min
int32 x_max
int32 y_max
int32 center_x
int32 center_y
