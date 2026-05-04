"""tracking_demo.launch.py — Full stack with optional RViz visualisation.

Launches the complete target-following system together with an optional
RViz2 window for monitoring detection outputs and robot state.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("underwater_target_bringup")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock."
    )
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz2 for visualisation.",
    )
    declare_camera_topic = DeclareLaunchArgument(
        "camera_topic",
        default_value="/camera/image_raw",
        description="Input camera topic.",
    )

    # Include the full robot launch (detection + control)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "robot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "autostart": "true",
        }.items(),
    )

    # Optional RViz2
    rviz_config = os.path.join(bringup_dir, "rviz", "tracking_demo.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_launch_rviz,
            declare_camera_topic,
            robot_launch,
            rviz_node,
        ]
    )
