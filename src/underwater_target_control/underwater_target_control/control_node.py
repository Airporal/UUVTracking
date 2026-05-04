"""Control ROS 2 lifecycle node.

Subscribes to detection outputs and drives the UUV to follow the target.

Topic graph
-----------
Subscriptions
  /target/pose_image         geometry_msgs/msg/Point
  /target/tracking_status    std_msgs/msg/Bool
  /odometry                  nav_msgs/msg/Odometry      [optional]
  /imu                       sensor_msgs/msg/Imu        [optional]

Publications
  /cmd_vel                   geometry_msgs/msg/Twist
  /target_following/error    underwater_target_msgs/msg/TargetError
  /target_following/state    std_msgs/msg/String
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from underwater_target_msgs.msg import TargetError

from underwater_target_control.controller_base import ControllerBase, TargetObservation
from underwater_target_control.ibvs_controller import IBVSController
from underwater_target_control.pid_controller import PIDController

try:
    from geometry_msgs.msg import Point
except ImportError:
    Point = None  # type: ignore[assignment,misc]


class ControlNode(LifecycleNode):
    """ROS 2 Lifecycle node for underwater target-following control.

    Supports pluggable controller backends (IBVS, PID).  The backend is
    selected via the ``controller_type`` parameter.

    Safety features
    ---------------
    * Command saturation (enforced inside each controller backend).
    * Watchdog timer: if no tracking update is received within
      ``watchdog_timeout`` seconds the node publishes a zero command.
    * Lost-target fallback: configurable behaviour (stop / search).
    """

    # Controller type registry
    _CONTROLLER_REGISTRY = {
        "ibvs": IBVSController,
        "pid": PIDController,
    }

    def __init__(self) -> None:
        super().__init__("control_node")
        self._controller: Optional[ControllerBase] = None
        self._is_tracking: bool = False
        self._last_image_x: float = 0.0
        self._last_image_y: float = 0.0
        self._last_distance: float = -1.0
        self._last_update_time: float = 0.0
        self._prev_time: float = 0.0

        self._declare_parameters()

    # ------------------------------------------------------------------
    # Parameter declarations
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        # Controller selection
        self.declare_parameter("controller_type", "ibvs")

        # IBVS params
        self.declare_parameter("ibvs.lambda_gain", 0.5)
        self.declare_parameter("ibvs.desired_image_x", 0.0)
        self.declare_parameter("ibvs.desired_image_y", 0.0)
        self.declare_parameter("ibvs.desired_distance", 2.0)
        self.declare_parameter("ibvs.distance_gain", 0.3)
        self.declare_parameter("ibvs.deadband", 0.05)

        # PID params
        self.declare_parameter("pid.yaw_kp", 0.8)
        self.declare_parameter("pid.yaw_ki", 0.0)
        self.declare_parameter("pid.yaw_kd", 0.05)
        self.declare_parameter("pid.heave_kp", 0.5)
        self.declare_parameter("pid.heave_ki", 0.0)
        self.declare_parameter("pid.heave_kd", 0.02)
        self.declare_parameter("pid.surge_kp", 0.3)
        self.declare_parameter("pid.surge_ki", 0.0)
        self.declare_parameter("pid.surge_kd", 0.01)
        self.declare_parameter("pid.desired_distance", 2.0)
        self.declare_parameter("pid.deadband", 0.05)

        # Velocity limits
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 0.5)

        # Watchdog
        self.declare_parameter("watchdog_timeout", 1.0)  # seconds

        # Control loop rate (Hz)
        self.declare_parameter("control_rate", 20.0)

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlNode: configuring …")

        ctrl_type = str(self.get_parameter("controller_type").value).lower()
        max_lv = float(self.get_parameter("max_linear_vel").value)
        max_av = float(self.get_parameter("max_angular_vel").value)

        if ctrl_type == "ibvs":
            self._controller = IBVSController(
                lambda_gain=float(self.get_parameter("ibvs.lambda_gain").value),
                desired_image_x=float(self.get_parameter("ibvs.desired_image_x").value),
                desired_image_y=float(self.get_parameter("ibvs.desired_image_y").value),
                desired_distance=float(self.get_parameter("ibvs.desired_distance").value),
                distance_gain=float(self.get_parameter("ibvs.distance_gain").value),
                max_linear_vel=max_lv,
                max_angular_vel=max_av,
                deadband=float(self.get_parameter("ibvs.deadband").value),
            )
        elif ctrl_type == "pid":
            self._controller = PIDController(
                yaw_kp=float(self.get_parameter("pid.yaw_kp").value),
                yaw_ki=float(self.get_parameter("pid.yaw_ki").value),
                yaw_kd=float(self.get_parameter("pid.yaw_kd").value),
                heave_kp=float(self.get_parameter("pid.heave_kp").value),
                heave_ki=float(self.get_parameter("pid.heave_ki").value),
                heave_kd=float(self.get_parameter("pid.heave_kd").value),
                surge_kp=float(self.get_parameter("pid.surge_kp").value),
                surge_ki=float(self.get_parameter("pid.surge_ki").value),
                surge_kd=float(self.get_parameter("pid.surge_kd").value),
                desired_distance=float(self.get_parameter("pid.desired_distance").value),
                max_linear_vel=max_lv,
                max_angular_vel=max_av,
                deadband=float(self.get_parameter("pid.deadband").value),
            )
        else:
            self.get_logger().error(f"Unknown controller_type: '{ctrl_type}'. Defaulting to IBVS.")
            self._controller = IBVSController(
                max_linear_vel=max_lv,
                max_angular_vel=max_av,
            )

        self._watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        control_rate = float(self.get_parameter("control_rate").value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        rel_qos = QoSProfile(depth=5)

        # Subscribers
        self._pose_sub = self.create_subscription(
            Point,
            "/target/pose_image",
            self._pose_callback,
            qos,
        )
        self._status_sub = self.create_subscription(
            Bool,
            "/target/tracking_status",
            self._status_callback,
            qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odometry",
            self._odom_callback,
            qos,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self._imu_callback,
            qos,
        )

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", rel_qos)
        self._error_pub = self.create_publisher(
            TargetError, "/target_following/error", rel_qos
        )
        self._state_pub = self.create_publisher(
            String, "/target_following/state", rel_qos
        )

        # Control-loop timer
        period = 1.0 / control_rate
        self._control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f"ControlNode: configured with '{ctrl_type}' controller at {control_rate} Hz."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlNode: activating …")
        self._prev_time = time.monotonic()
        self._last_update_time = time.monotonic()
        if self._controller:
            self._controller.reset()
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlNode: deactivating …")
        self._publish_zero_cmd()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlNode: cleaning up …")
        self._controller = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("ControlNode: shutting down.")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------

    def _pose_callback(self, msg: Point) -> None:
        self._last_image_x = msg.x
        self._last_image_y = msg.y
        self._last_distance = msg.z
        self._last_update_time = time.monotonic()

    def _status_callback(self, msg: Bool) -> None:
        self._is_tracking = msg.data

    def _odom_callback(self, msg: Odometry) -> None:
        # Reserved for advanced controllers (e.g. MPC)
        pass

    def _imu_callback(self, msg: Imu) -> None:
        # Reserved for advanced controllers (e.g. disturbance compensation)
        pass

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        now = time.monotonic()
        dt = max(now - self._prev_time, 1e-4)
        self._prev_time = now

        # Watchdog check
        time_since_update = now - self._last_update_time
        if time_since_update > self._watchdog_timeout:
            if self._is_tracking:
                self.get_logger().warning(
                    f"Watchdog triggered: no update for {time_since_update:.2f}s"
                )
                self._is_tracking = False
            self._publish_zero_cmd()
            self._publish_state("WATCHDOG")
            return

        if not self._controller:
            return

        obs = TargetObservation(
            image_x=self._last_image_x,
            image_y=self._last_image_y,
            estimated_distance=self._last_distance,
            is_valid=self._is_tracking,
        )

        cmd = self._controller.compute_command(obs, dt)

        # Publish velocity command
        twist = Twist()
        twist.linear.x = cmd.vx
        twist.linear.y = cmd.vy
        twist.linear.z = cmd.vz
        twist.angular.x = cmd.wx
        twist.angular.y = cmd.wy
        twist.angular.z = cmd.wz
        self._cmd_vel_pub.publish(twist)

        # Publish error diagnostics
        error_msg = TargetError()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.error_x = self._last_image_x
        error_msg.error_y = self._last_image_y
        error_msg.error_distance = (
            self._last_distance if self._last_distance > 0 else 0.0
        )
        error_msg.error_magnitude = (
            (self._last_image_x**2 + self._last_image_y**2) ** 0.5
        )
        error_msg.is_centred = error_msg.error_magnitude < 0.05
        error_msg.controller_state = "FOLLOWING" if self._is_tracking else "LOST"
        self._error_pub.publish(error_msg)

        state_label = "FOLLOWING" if self._is_tracking else "LOST"
        self._publish_state(state_label)

    def _publish_zero_cmd(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_state(self, label: str) -> None:
        msg = String()
        msg.data = label
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
