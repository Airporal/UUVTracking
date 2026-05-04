# TargetError.msg
# Control error published by the control node for diagnostics / outer-loop control.

std_msgs/Header header

# Image-plane error (normalised coordinates, target minus centre)
float32 error_x      # horizontal error [-1, 1]
float32 error_y      # vertical  error [-1, 1]

# Distance error (positive = too far, negative = too close, metres)
float32 error_distance

# Magnitude of the combined image-plane error
float32 error_magnitude

# True when the error is within the acceptable dead-band
bool is_centred

# Current controller state label (e.g. "FOLLOWING", "LOST", "SEARCHING")
string controller_state
