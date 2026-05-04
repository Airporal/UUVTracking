"""detection_only.launch.py — Launch only the detection node.

Useful for testing and calibration of the detection pipeline without
the full control stack running.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    detection_dir = get_package_share_directory("underwater_target_detection")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock."
    )
    declare_detection_params = DeclareLaunchArgument(
        "detection_params",
        default_value=os.path.join(detection_dir, "config", "detection_params.yaml"),
        description="Path to detection parameters.",
    )
    declare_camera_topic = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Input camera topic.",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically activate detection node.",
    )

    detection_node = LifecycleNode(
        package="underwater_target_detection",
        executable="detection_node",
        name="detection_node",
        namespace="",
        output="screen",
        parameters=[
            LaunchConfiguration("detection_params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/camera/image_raw", LaunchConfiguration("camera_topic")),
        ],
    )

    configure_detection = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=detection_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_detection = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=detection_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    on_detection_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=detection_node,
            goal_state="inactive",
            entities=[activate_detection],
        ),
        condition=IfCondition(LaunchConfiguration("autostart")),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_detection_params,
            declare_camera_topic,
            declare_autostart,
            detection_node,
            on_detection_configured,
            configure_detection,
        ]
    )
