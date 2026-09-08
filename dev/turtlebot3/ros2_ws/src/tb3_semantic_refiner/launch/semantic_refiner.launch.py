"""
semantic_refiner.launch.py — Launch the asynchronous semantic refiner node.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_semantic_refiner")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument(
            "worker_enabled",
            default_value="true",
            description="Enable the background semantic-nav-memory worker.",
        ),
        DeclareLaunchArgument(
            "auto_run_when_buffer_ready",
            default_value="true",
            description="Automatically run a refinement job once the frame buffer is ready.",
        ),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        Node(
            package="tb3_semantic_refiner",
            executable="semantic_refiner_node",
            name="semantic_refiner_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "semantic_refiner.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "worker_enabled": LaunchConfiguration("worker_enabled"),
                    "auto_run_when_buffer_ready": LaunchConfiguration("auto_run_when_buffer_ready"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "map_topic": LaunchConfiguration("map_topic"),
                },
            ],
        ),
    ])
