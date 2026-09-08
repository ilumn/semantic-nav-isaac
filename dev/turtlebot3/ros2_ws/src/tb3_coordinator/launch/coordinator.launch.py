"""
coordinator.launch.py — Launch the semantic navigation coordinator.

Usage:
    ros2 launch tb3_coordinator coordinator.launch.py
    ros2 launch tb3_coordinator coordinator.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_coordinator")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument(
            "motion_initially_armed",
            default_value="true",
            description="If false, block autonomous motion until set_motion_armed is called.",
        ),
        DeclareLaunchArgument(
            "readiness_required",
            default_value="false",
            description="If true, refuse arming until sensor topics and TF are ready.",
        ),
        DeclareLaunchArgument("readiness_scan_topic", default_value="/scan"),
        DeclareLaunchArgument("readiness_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("readiness_image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("readiness_camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("readiness_map_frame", default_value="map"),
        DeclareLaunchArgument("readiness_odom_frame", default_value="odom"),
        DeclareLaunchArgument("readiness_base_frame", default_value="base_link"),
        DeclareLaunchArgument("readiness_camera_frame", default_value=""),

        Node(
            package="tb3_coordinator",
            executable="coordinator_node",
            name="coordinator_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "coordinator.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "motion_initially_armed": LaunchConfiguration("motion_initially_armed"),
                    "readiness_required": LaunchConfiguration("readiness_required"),
                    "readiness_scan_topic": LaunchConfiguration("readiness_scan_topic"),
                    "readiness_odom_topic": LaunchConfiguration("readiness_odom_topic"),
                    "readiness_image_topic": LaunchConfiguration("readiness_image_topic"),
                    "readiness_camera_info_topic": LaunchConfiguration("readiness_camera_info_topic"),
                    "readiness_map_frame": LaunchConfiguration("readiness_map_frame"),
                    "readiness_odom_frame": LaunchConfiguration("readiness_odom_frame"),
                    "readiness_base_frame": LaunchConfiguration("readiness_base_frame"),
                    "readiness_camera_frame": LaunchConfiguration("readiness_camera_frame"),
                },
            ],
        ),
    ])
