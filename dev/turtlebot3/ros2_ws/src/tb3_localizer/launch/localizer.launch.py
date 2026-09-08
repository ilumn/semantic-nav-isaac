"""
localizer.launch.py — Launch the Stage-2 planar localizer node.

Usage:
    ros2 launch tb3_localizer localizer.launch.py
    ros2 launch tb3_localizer localizer.launch.py use_sim_time:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_localizer")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="RGB image topic used for detection bearing geometry.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan topic used to range semantic detections.",
        ),

        Node(
            package="tb3_localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "localizer.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                },
            ],
        ),
    ])
