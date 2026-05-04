# TargetTrackingState.msg
# High-level tracking state published by the detection node.

std_msgs/Header header

# Current tracking status
bool is_tracking

# Number of consecutive frames without a detection (0 when tracking)
int32 lost_frame_count

# Most recent valid detection
underwater_target_msgs/TargetDetection detection

# Tracker-internal state identifier (e.g. "TRACKING", "LOST", "SEARCHING")
string state_label
