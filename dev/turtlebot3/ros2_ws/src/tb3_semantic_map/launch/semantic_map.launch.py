"""
semantic_map.launch.py — Launch the live semantic map node.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_semantic_map")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("state_topic", default_value="/semantic_map/state"),
        DeclareLaunchArgument("compat_topic", default_value="/semantic_map/compat_objects"),
        DeclareLaunchArgument("markers_topic", default_value="/semantic_map/markers"),
        DeclareLaunchArgument("status_topic", default_value="/semantic_map/status"),
        DeclareLaunchArgument("debug_image_topic", default_value="/semantic_map/debug_image"),
        DeclareLaunchArgument("device", default_value="cpu"),
        Node(
            package="tb3_semantic_map",
            executable="semantic_map_node",
            name="semantic_map_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "semantic_map.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "map_topic": LaunchConfiguration("map_topic"),
                    "state_topic": LaunchConfiguration("state_topic"),
                    "compat_topic": LaunchConfiguration("compat_topic"),
                    "markers_topic": LaunchConfiguration("markers_topic"),
                    "status_topic": LaunchConfiguration("status_topic"),
                    "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                    "device": LaunchConfiguration("device"),
                },
            ],
        ),
    ])
