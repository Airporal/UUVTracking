"""robot.launch.py — Full target-following stack for real robot deployment.

Launches:
  * detection_node  (lifecycle)
  * control_node    (lifecycle)

Parameters are loaded from the bundled YAML config files.
Topic remappings can be customised via launch arguments.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("underwater_target_bringup")
    detection_dir = get_package_share_directory("underwater_target_detection")
    control_dir = get_package_share_directory("underwater_target_control")

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock."
    )
    declare_detection_params = DeclareLaunchArgument(
        "detection_params",
        default_value=os.path.join(detection_dir, "config", "detection_params.yaml"),
        description="Path to the detection node parameter file.",
    )
    declare_control_params = DeclareLaunchArgument(
        "control_params",
        default_value=os.path.join(control_dir, "config", "control_params.yaml"),
        description="Path to the control node parameter file.",
    )
    declare_camera_topic = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Camera image topic.",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically transition lifecycle nodes to active.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    detection_params = LaunchConfiguration("detection_params")
    control_params = LaunchConfiguration("control_params")

    # ---------------------------------------------------------------------------
    # Nodes
    # ---------------------------------------------------------------------------
    detection_node = LifecycleNode(
        package="underwater_target_detection",
        executable="detection_node",
        name="detection_node",
        namespace="",
        output="screen",
        parameters=[
            detection_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/camera/image_raw", LaunchConfiguration("camera_topic")),
        ],
    )

    control_node = LifecycleNode(
        package="underwater_target_control",
        executable="control_node",
        name="control_node",
        namespace="",
        output="screen",
        parameters=[
            control_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---------------------------------------------------------------------------
    # Lifecycle auto-configure & activate
    # ---------------------------------------------------------------------------
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
    configure_control = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=control_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_control = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=control_node,
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # Chain: detection configured → activate detection
    on_detection_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=detection_node,
            goal_state="inactive",
            entities=[activate_detection],
        ),
        condition=IfCondition(LaunchConfiguration("autostart")),
    )
    # Chain: control configured → activate control
    on_control_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=control_node,
            goal_state="inactive",
            entities=[activate_control],
        ),
        condition=IfCondition(LaunchConfiguration("autostart")),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_detection_params,
            declare_control_params,
            declare_camera_topic,
            declare_autostart,
            # Nodes
            detection_node,
            control_node,
            # Lifecycle management
            on_detection_configured,
            on_control_configured,
            configure_detection,
            configure_control,
        ]
    )
