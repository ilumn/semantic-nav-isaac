"""
semantic_query.launch.py — Launch the Stage-4 semantic query node.

Usage:
    ros2 launch tb3_query semantic_query.launch.py
    ros2 launch tb3_query semantic_query.launch.py use_sim_time:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_query")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument(
            "memory_topic",
            default_value="/semantic_memory_node/objects",
            description="Optional override for the semantic memory source topic.",
        ),
        DeclareLaunchArgument(
            "memory_mode",
            default_value="semantic_map_state",
            description="Semantic memory source mode: detection3d or semantic_map_state.",
        ),
        DeclareLaunchArgument(
            "semantic_map_topic",
            default_value="/semantic_map/state",
            description="Semantic map source topic when memory_mode=semantic_map_state.",
        ),
        DeclareLaunchArgument(
            "output_frame",
            default_value="map",
            description="Override for the query result frame.",
        ),
        DeclareLaunchArgument(
            "robot_base_frame",
            default_value="base_link",
            description="Robot base frame used for robot-relative target ranking.",
        ),

        Node(
            package="tb3_query",
            executable="semantic_query_node.py",
            name="semantic_query_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "semantic_query.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "memory_topic": LaunchConfiguration("memory_topic"),
                    "memory_mode": LaunchConfiguration("memory_mode"),
                    "semantic_map_topic": LaunchConfiguration("semantic_map_topic"),
                    "output_frame": LaunchConfiguration("output_frame"),
                    "robot_base_frame": LaunchConfiguration("robot_base_frame"),
                },
            ],
        ),
    ])
