"""Detection ROS 2 lifecycle node.

This node acts as the bridge between the ROS camera topic and the underlying
detector backend.  It follows the ROS 2 *managed-node* (lifecycle) pattern so
that external orchestration tools can configure, activate, and deactivate it
without restarting the process.

Topic graph
-----------
Subscriptions
  /camera/image_raw          sensor_msgs/msg/Image
  /camera/camera_info        sensor_msgs/msg/CameraInfo   [optional]

Publications
  /target/detection          underwater_target_msgs/msg/TargetDetection
  /target/bbox               vision_msgs/msg/Detection2DArray
  /target/pose_image         geometry_msgs/msg/Point
  /target/tracking_status    std_msgs/msg/Bool
  /target/debug_image        sensor_msgs/msg/Image
"""

from __future__ import annotations

import threading
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from underwater_target_msgs.msg import TargetDetection, TargetTrackingState

from underwater_target_detection.detector_base import DetectorBase
from underwater_target_detection.yolo_detector import YOLODetector, is_available as yolo_available


class DetectionNode(LifecycleNode):
    """ROS 2 Lifecycle node for underwater target detection.

    Loads a configurable detector backend, subscribes to camera images, and
    publishes detection results for the downstream control node.
    """

    def __init__(self) -> None:
        super().__init__("detection_node")
        self._bridge = CvBridge()
        self._detector: Optional[DetectorBase] = None
        self._lock = threading.Lock()

        # Tracking state
        self._is_tracking: bool = False
        self._lost_frame_count: int = 0
        self._max_lost_frames: int = 30

        self._declare_parameters()

    # ------------------------------------------------------------------
    # Parameter declarations
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("target_classes", [0])  # class 0 = person by default
        self.declare_parameter("device", "cpu")
        self.declare_parameter("max_lost_frames", 30)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DetectionNode: configuring …")

        model_path = self.get_parameter("model_path").value
        conf_thresh = float(self.get_parameter("confidence_threshold").value)
        target_classes = list(self.get_parameter("target_classes").value)
        device = str(self.get_parameter("device").value)
        self._max_lost_frames = int(self.get_parameter("max_lost_frames").value)

        # Build detector
        self._detector = YOLODetector(
            confidence_threshold=conf_thresh,
            target_classes=target_classes,
            device=device,
        )
        try:
            self._detector.load_model(model_path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load model: {exc}")
            if not yolo_available():
                self.get_logger().warning(
                    "ultralytics not installed — detector will return empty results."
                )

        # QoS
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        best_effort_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        # Subscribers (inactive until activate)
        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            sensor_qos,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            sensor_qos,
        )

        # Publishers
        self._detection_pub = self.create_publisher(
            TargetDetection, "/target/detection", best_effort_qos
        )
        self._bbox_pub = self.create_publisher(
            Detection2DArray, "/target/bbox", best_effort_qos
        )
        self._pose_image_pub = self.create_publisher(
            Point, "/target/pose_image", best_effort_qos
        )
        self._tracking_status_pub = self.create_publisher(
            Bool, "/target/tracking_status", best_effort_qos
        )
        self._tracking_state_pub = self.create_publisher(
            TargetTrackingState, "/target/tracking_state", best_effort_qos
        )
        self._debug_image_pub = self.create_publisher(
            Image, "/target/debug_image", best_effort_qos
        )

        self.get_logger().info("DetectionNode: configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DetectionNode: activating …")
        self._is_tracking = False
        self._lost_frame_count = 0
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DetectionNode: deactivating …")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DetectionNode: cleaning up …")
        self._detector = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("DetectionNode: shutting down.")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        # Reserved for future use (e.g. undistortion, distance estimation)
        pass

    def _image_callback(self, msg: Image) -> None:
        if not self._detector:
            return

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"CvBridge conversion error: {exc}")
            return

        with self._lock:
            detections = self._detector.detect(cv_image)

        publish_debug = bool(self.get_parameter("publish_debug_image").value)

        if detections:
            # Pick the highest-confidence detection as the primary target
            best = max(detections, key=lambda d: d.confidence)

            self._is_tracking = True
            self._lost_frame_count = 0

            stamp = msg.header.stamp

            # TargetDetection
            det_msg = self._build_target_detection_msg(
                best, stamp, msg.header.frame_id
            )
            self._detection_pub.publish(det_msg)

            # Detection2DArray (vision_msgs)
            d2d_array = Detection2DArray()
            d2d_array.header = det_msg.header
            for det in detections:
                d2d = Detection2D()
                d2d.header = det_msg.header
                bbox = BoundingBox2D()
                bbox.center.position.x = det.bbox_center_x
                bbox.center.position.y = det.bbox_center_y
                bbox.size_x = det.bbox_width
                bbox.size_y = det.bbox_height
                d2d.bbox = bbox
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(det.class_id)
                hyp.hypothesis.score = det.confidence
                d2d.results.append(hyp)
                d2d_array.detections.append(d2d)
            self._bbox_pub.publish(d2d_array)

            # Normalised pose
            pose_msg = Point()
            pose_msg.x = best.image_x
            pose_msg.y = best.image_y
            pose_msg.z = best.estimated_distance
            self._pose_image_pub.publish(pose_msg)

            if publish_debug:
                debug_img = self._annotate(cv_image, detections)
                self._debug_image_pub.publish(
                    self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                )
        else:
            self._lost_frame_count += 1
            if self._lost_frame_count >= self._max_lost_frames:
                self._is_tracking = False

        # Tracking status
        status_msg = Bool()
        status_msg.data = self._is_tracking
        self._tracking_status_pub.publish(status_msg)

        # TargetTrackingState
        state_msg = TargetTrackingState()
        state_msg.header.stamp = msg.header.stamp
        state_msg.header.frame_id = msg.header.frame_id
        state_msg.is_tracking = self._is_tracking
        state_msg.lost_frame_count = self._lost_frame_count
        state_msg.state_label = "TRACKING" if self._is_tracking else "LOST"
        if detections:
            best = max(detections, key=lambda d: d.confidence)
            state_msg.detection = self._build_target_detection_msg(
                best, state_msg.header.stamp, state_msg.header.frame_id
            )
        self._tracking_state_pub.publish(state_msg)

    # ------------------------------------------------------------------
    # Visualisation helpers
    # ------------------------------------------------------------------

    def _build_target_detection_msg(
        self,
        det: Detection,
        stamp,
        frame_id: str,
    ) -> TargetDetection:
        """Populate and return a TargetDetection message from a Detection object."""
        msg = TargetDetection()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.class_id = det.class_id
        msg.class_name = det.class_name
        msg.confidence = det.confidence
        msg.bbox_center_x = det.bbox_center_x
        msg.bbox_center_y = det.bbox_center_y
        msg.bbox_width = det.bbox_width
        msg.bbox_height = det.bbox_height
        msg.image_x = det.image_x
        msg.image_y = det.image_y
        msg.estimated_distance = det.estimated_distance
        msg.is_valid = True
        return msg

    def _annotate(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """Draw bounding boxes and labels on a copy of *image*."""
        annotated = image.copy()
        h, w = annotated.shape[:2]
        for det in detections:
            x1 = int(det.bbox_center_x - det.bbox_width / 2)
            y1 = int(det.bbox_center_y - det.bbox_height / 2)
            x2 = int(det.bbox_center_x + det.bbox_width / 2)
            y2 = int(det.bbox_center_y + det.bbox_height / 2)
            color = (0, 255, 0) if det.is_valid else (0, 0, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            cv2.putText(
                annotated,
                label,
                (x1, max(y1 - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )
        # Draw crosshair at image centre
        cv2.drawMarker(
            annotated,
            (w // 2, h // 2),
            (255, 255, 0),
            cv2.MARKER_CROSS,
            20,
            2,
        )
        return annotated


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionNode()
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
