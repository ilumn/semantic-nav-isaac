import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_frontier_exploration")
    config = os.path.join(pkg_share, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_topic = LaunchConfiguration("map_topic", default="/map")
    costmap_topic = LaunchConfiguration("costmap_topic", default="/global_costmap/costmap")
    odom_topic = LaunchConfiguration("odom_topic", default="/odom")
    launch_startup_warmup = LaunchConfiguration("launch_startup_warmup", default="true")
    require_startup_warmup = LaunchConfiguration("require_startup_warmup", default="true")
    exploration_initially_enabled = LaunchConfiguration("exploration_initially_enabled", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                             description="Use simulation time"),
        DeclareLaunchArgument("map_topic", default_value="/map",
                             description="OccupancyGrid topic for the map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap",
                             description="Costmap topic for frontier cost sampling"),
        DeclareLaunchArgument("odom_topic", default_value="/odom",
                             description="Odometry topic for goal assignment"),
        DeclareLaunchArgument("launch_startup_warmup", default_value="true",
                             description="Run the startup rotation warmup node"),
        DeclareLaunchArgument("require_startup_warmup", default_value="true",
                             description="Require the warmup completion signal before frontier goals"),
        DeclareLaunchArgument("exploration_initially_enabled", default_value="true",
                             description="Initial autonomous frontier goal state"),

        # One-shot cmd_vel rotation scan; signals exploration_warmup_complete when done.
        Node(
            package="tb3_frontier_exploration",
            executable="startup_map_warmup_node.py",
            name="startup_map_warmup_node",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(launch_startup_warmup),
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="frontier_detection_node",
            name="frontier_detection_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                {"frontier_detection_node": {"ros__parameters": {
                    "map_topic": map_topic,
                    "costmap_topic": costmap_topic,
                }}},
            ],
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="goal_assignment_node",
            name="goal_assignment_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                {"goal_assignment_node": {"ros__parameters": {
                    "odom_topic": odom_topic,
                    "require_startup_warmup": require_startup_warmup,
                    "exploration_initially_enabled": exploration_initially_enabled,
                }}},
            ],
            output="screen",
        ),
    ])
