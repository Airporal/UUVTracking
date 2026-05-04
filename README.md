# UUVTracking — Underwater Robot Target-Following ROS 2 Workspace

A modular ROS 2 (Humble) workspace for autonomous underwater target detection and following using a monocular camera.

## Repository Layout

```
UUVTracking/
└── src/
    ├── underwater_target_msgs/          # Custom message definitions
    ├── underwater_target_detection/     # Monocular vision / detection node
    ├── underwater_target_control/       # Motion-control node
    └── underwater_target_bringup/       # Launch files & system configuration
```

---

## Packages

### `underwater_target_msgs`

Custom ROS 2 interface package (CMake / `rosidl`).

| Message | Description |
|---|---|
| `TargetDetection` | Single detection result (class, confidence, bbox, normalised pose) |
| `TargetTrackingState` | High-level tracker state (tracking flag, lost count, latest detection) |
| `TargetError` | Control error for diagnostics (image error, distance error, state label) |

### `underwater_target_detection`

Python lifecycle node that runs a pluggable detector backend.

**Key files**

| File | Role |
|---|---|
| `detection_node.py` | ROS 2 lifecycle node |
| `detector_base.py` | Abstract `DetectorBase` interface |
| `yolo_detector.py` | Ultralytics YOLOv8 implementation |
| `config/detection_params.yaml` | Default parameters |

**Topic graph**

| Direction | Topic | Type |
|---|---|---|
| Sub | `/camera/image_raw` | `sensor_msgs/Image` |
| Sub | `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| Pub | `/target/detection` | `TargetDetection` |
| Pub | `/target/bbox` | `vision_msgs/Detection2DArray` |
| Pub | `/target/pose_image` | `geometry_msgs/Point` |
| Pub | `/target/tracking_status` | `std_msgs/Bool` |
| Pub | `/target/tracking_state` | `TargetTrackingState` |
| Pub | `/target/debug_image` | `sensor_msgs/Image` |

### `underwater_target_control`

Python lifecycle node that converts image-plane target error into UUV velocity commands.

**Pluggable controllers**

| Backend | Class | Description |
|---|---|---|
| IBVS (default) | `IBVSController` | Proportional image-based visual servoing |
| PID | `PIDController` | Independent PID loops for yaw, heave, surge |

**Topic graph**

| Direction | Topic | Type |
|---|---|---|
| Sub | `/target/pose_image` | `geometry_msgs/Point` |
| Sub | `/target/tracking_status` | `std_msgs/Bool` |
| Sub | `/odometry` | `nav_msgs/Odometry` (optional) |
| Sub | `/imu` | `sensor_msgs/Imu` (optional) |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` |
| Pub | `/target_following/error` | `TargetError` |
| Pub | `/target_following/state` | `std_msgs/String` |

### `underwater_target_bringup`

Launch files and system-wide configuration.

| Launch file | Purpose |
|---|---|
| `robot.launch.py` | Full stack (detection + control) for real-robot deployment |
| `detection_only.launch.py` | Detection pipeline only (calibration / testing) |
| `tracking_demo.launch.py` | Full stack + optional RViz2 visualisation |

---

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04 recommended)
- Python ≥ 3.10
- `cv_bridge`, `vision_msgs`, `lifecycle_msgs`
- (Optional) [Ultralytics](https://github.com/ultralytics/ultralytics) for YOLO inference:
  ```bash
  pip install ultralytics
  ```

---

## Build

```bash
# From the workspace root (this repository)
cd /path/to/UUVTracking

colcon build --symlink-install
source install/setup.bash
```

---

## Running

### Full stack (real robot)

```bash
ros2 launch underwater_target_bringup robot.launch.py \
    camera_topic:=/camera/image_raw \
    use_sim_time:=false
```

### Detection only

```bash
ros2 launch underwater_target_bringup detection_only.launch.py \
    camera_topic:=/camera/image_raw
```

### Tracking demo with RViz

```bash
ros2 launch underwater_target_bringup tracking_demo.launch.py \
    launch_rviz:=true
```

### Select controller backend

```bash
ros2 launch underwater_target_bringup robot.launch.py \
    control_params:=/path/to/custom_control.yaml
```

---

## Configuration

All parameters are fully configurable via YAML.  Default files live in the
`config/` directory of each package.

**Detection parameters** (`underwater_target_detection/config/detection_params.yaml`)

```yaml
detection_node:
  ros__parameters:
    model_path: "yolov8n.pt"
    confidence_threshold: 0.5
    target_classes: [0]   # COCO class IDs
    device: "cpu"
    publish_debug_image: true
```

**Control parameters** (`underwater_target_control/config/control_params.yaml`)

```yaml
control_node:
  ros__parameters:
    controller_type: "ibvs"   # or "pid"
    max_linear_vel: 0.5
    max_angular_vel: 0.5
    ibvs:
      lambda_gain: 0.5
      desired_distance: 2.0
```

---

## Testing

```bash
# Unit tests (no ROS 2 runtime required)
cd src/underwater_target_detection
python -m pytest test/ -v

cd ../underwater_target_control
python -m pytest test/ -v
```

---

## Architecture

```
Camera ──► detection_node ──► /target/pose_image ──► control_node ──► /cmd_vel ──► UUV
                          └──► /target/tracking_status ─┘
```

The system uses the **ROS 2 managed-node (lifecycle)** pattern for both the
detection and control nodes, enabling safe configuration, activation, and
deactivation by an external orchestrator.

The controller and detector are both **abstract-base-class driven**, making it
straightforward to swap in alternative backends (e.g. an SSD detector, an MPC
controller) without modifying any node or launch code.

---

## License

Apache-2.0
