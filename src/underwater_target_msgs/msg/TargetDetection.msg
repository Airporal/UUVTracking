# TargetDetection.msg
# Single detection result from the monocular camera detector.

std_msgs/Header header

# Class information
int32 class_id
string class_name

# Detection confidence [0.0, 1.0]
float32 confidence

# Bounding box center in pixel coordinates
float32 bbox_center_x
float32 bbox_center_y

# Bounding box size in pixels
float32 bbox_width
float32 bbox_height

# Normalised image coordinates of the bbox centre [-1.0, 1.0]
float32 image_x
float32 image_y

# Optional: estimated relative distance from the camera (metres, -1 if unknown)
float32 estimated_distance

# True when this detection is considered valid / above threshold
bool is_valid
