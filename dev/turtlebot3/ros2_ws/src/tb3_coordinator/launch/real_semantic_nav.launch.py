"""
real_semantic_nav.launch.py -- Real TurtleBot3 semantic navigation stack.

Precondition:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch turtlebot3_bringup robot.launch.py

Then:
    ros2 launch tb3_coordinator real_semantic_nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_coord = get_package_share_directory("tb3_coordinator")
    stack_launch = os.path.join(pkg_coord, "launch", "semantic_nav_stack.launch.py")
    nav2_params = os.path.join(pkg_coord, "config", "nav2", "waffle_pi_real_nav2.yaml")
    slam_params = os.path.join(pkg_coord, "config", "slam", "waffle_pi_real_slam.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("detector_device", default_value="cpu"),
        DeclareLaunchArgument("refiner_worker_enabled", default_value="true"),
        DeclareLaunchArgument("refiner_auto_run", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap"),
        DeclareLaunchArgument("readiness_camera_frame", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stack_launch),
            launch_arguments={
                "use_sim_time": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "nav2_params_file": nav2_params,
                "slam_params_file": slam_params,
                "detector_device": LaunchConfiguration("detector_device"),
                "refiner_worker_enabled": LaunchConfiguration("refiner_worker_enabled"),
                "refiner_auto_run": LaunchConfiguration("refiner_auto_run"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "map_topic": LaunchConfiguration("map_topic"),
                "costmap_topic": LaunchConfiguration("costmap_topic"),
                "semantic_map_state_topic": "/semantic_map/live_state",
                "semantic_map_compat_topic": "/semantic_map/live_compat_objects",
                "semantic_map_markers_topic": "/semantic_map/live_markers",
                "semantic_map_status_topic": "/semantic_map/live_status",
                "semantic_map_debug_image_topic": "/semantic_map/live_debug_image",
                "semantic_memory_state_topic": "/semantic_map/state",
                "motion_initially_armed": "false",
                "readiness_required": "true",
                "readiness_map_frame": "map",
                "readiness_odom_frame": "odom",
                "readiness_base_frame": "base_link",
                "readiness_camera_frame": LaunchConfiguration("readiness_camera_frame"),
                "exploration_initially_enabled": "false",
                "launch_startup_warmup": "false",
                "require_startup_warmup": "false",
                "query_memory_mode": "semantic_map_state",
                "query_memory_topic": "/semantic_map_memory_node/landmark_objects",
                "query_semantic_map_topic": "/semantic_map/state",
                "query_output_frame": "map",
            }.items(),
        ),
    ])
