"""
semantic_nav_stack.launch.py -- Shared ROS stack for sim and real TurtleBot3 runs.

This launch intentionally excludes simulator bridges and low-level robot
bringup. Isaac Sim and hardware wrappers provide those parts, then include this
common stack with the appropriate timing and topic arguments.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_det = get_package_share_directory("tb3_detector")
    pkg_loc = get_package_share_directory("tb3_localizer")
    pkg_mem = get_package_share_directory("tb3_memory")
    pkg_qry = get_package_share_directory("tb3_query")
    pkg_nav = get_package_share_directory("tb3_nav_adapter")
    pkg_coord = get_package_share_directory("tb3_coordinator")
    pkg_sem = get_package_share_directory("tb3_semantic_map")
    pkg_ref = get_package_share_directory("tb3_semantic_refiner")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # Nav2 ignores this map while ``slam=True``, but keep the default resolvable
    # so launch-time validation and an operator switching modes do not inherit a
    # path that does not exist in the Jazzy nav2_bringup package.
    default_map = os.path.join(pkg_nav2, "maps", "warehouse.yaml")
    default_nav2_params = os.path.join(pkg_nav2, "params", "nav2_params.yaml")
    default_slam_params = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config",
        "mapper_params_online_sync.yaml",
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": LaunchConfiguration("nav2_map"),
            "params_file": LaunchConfiguration("nav2_params_file"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "autostart": "True",
            "use_composition": "False",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_nav2")),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fe, "launch", "exploration.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map_topic": LaunchConfiguration("map_topic"),
            "costmap_topic": LaunchConfiguration("costmap_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "launch_startup_warmup": LaunchConfiguration("launch_startup_warmup"),
            "require_startup_warmup": LaunchConfiguration("require_startup_warmup"),
            "exploration_initially_enabled": LaunchConfiguration("exploration_initially_enabled"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_exploration")),
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_det, "launch", "detector.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "device": LaunchConfiguration("detector_device"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_detector")),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, "launch", "localizer.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "image_topic": LaunchConfiguration("image_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_localizer")),
    )

    memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mem, "launch", "semantic_memory.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration("launch_memory")),
    )

    semantic_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sem, "launch", "semantic_map.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "map_topic": LaunchConfiguration("map_topic"),
            "state_topic": LaunchConfiguration("semantic_map_state_topic"),
            "compat_topic": LaunchConfiguration("semantic_map_compat_topic"),
            "markers_topic": LaunchConfiguration("semantic_map_markers_topic"),
            "status_topic": LaunchConfiguration("semantic_map_status_topic"),
            "debug_image_topic": LaunchConfiguration("semantic_map_debug_image_topic"),
            "device": LaunchConfiguration("detector_device"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_semantic_map")),
    )

    semantic_refiner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ref, "launch", "semantic_refiner.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "worker_enabled": LaunchConfiguration("refiner_worker_enabled"),
            "auto_run_when_buffer_ready": LaunchConfiguration("refiner_auto_run"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "map_topic": LaunchConfiguration("map_topic"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_semantic_refiner")),
    )

    query = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_qry, "launch", "semantic_query.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "memory_mode": LaunchConfiguration("query_memory_mode"),
            "memory_topic": LaunchConfiguration("query_memory_topic"),
            "semantic_map_topic": LaunchConfiguration("query_semantic_map_topic"),
            "output_frame": LaunchConfiguration("query_output_frame"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_query")),
    )

    nav_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "nav_goal_adapter.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration("launch_nav_adapter")),
    )

    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_coord, "launch", "coordinator.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "motion_initially_armed": LaunchConfiguration("motion_initially_armed"),
            "readiness_required": LaunchConfiguration("readiness_required"),
            "readiness_scan_topic": LaunchConfiguration("scan_topic"),
            "readiness_odom_topic": LaunchConfiguration("odom_topic"),
            "readiness_image_topic": LaunchConfiguration("image_topic"),
            "readiness_camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "readiness_map_frame": LaunchConfiguration("readiness_map_frame"),
            "readiness_odom_frame": LaunchConfiguration("readiness_odom_frame"),
            "readiness_base_frame": LaunchConfiguration("readiness_base_frame"),
            "readiness_camera_frame": LaunchConfiguration("readiness_camera_frame"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_coordinator")),
    )

    semantic_map_memory = Node(
        package="tb3_coordinator",
        executable="semantic_map_memory_node",
        name="semantic_map_memory_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("tb3_coordinator"), "config", "coordinator.yaml"
            ]),
            {
                "use_sim_time": use_sim_time,
                "state_topic": LaunchConfiguration("semantic_memory_state_topic"),
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_semantic_map_memory")),
    )

    runtime_debug = Node(
        package="tb3_coordinator",
        executable="semantic_runtime_debug_node",
        name="semantic_runtime_debug_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_runtime_debug")),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("tb3_coordinator"), "rviz", "semantic_nav.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("nav2_map", default_value=default_map),
        DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
        DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
        DeclareLaunchArgument("semantic_map_state_topic", default_value="/semantic_map/state"),
        DeclareLaunchArgument("semantic_map_compat_topic", default_value="/semantic_map/compat_objects"),
        DeclareLaunchArgument("semantic_map_markers_topic", default_value="/semantic_map/markers"),
        DeclareLaunchArgument("semantic_map_status_topic", default_value="/semantic_map/status"),
        DeclareLaunchArgument("semantic_map_debug_image_topic", default_value="/semantic_map/debug_image"),
        DeclareLaunchArgument("semantic_memory_state_topic", default_value=""),
        DeclareLaunchArgument("detector_device", default_value="cpu"),
        DeclareLaunchArgument("motion_initially_armed", default_value="true"),
        DeclareLaunchArgument("readiness_required", default_value="false"),
        DeclareLaunchArgument("readiness_map_frame", default_value="map"),
        DeclareLaunchArgument("readiness_odom_frame", default_value="odom"),
        DeclareLaunchArgument("readiness_base_frame", default_value="base_link"),
        DeclareLaunchArgument("readiness_camera_frame", default_value=""),
        DeclareLaunchArgument("exploration_initially_enabled", default_value="true"),
        DeclareLaunchArgument("launch_startup_warmup", default_value="true"),
        DeclareLaunchArgument("require_startup_warmup", default_value="true"),
        DeclareLaunchArgument("launch_nav2", default_value="true"),
        DeclareLaunchArgument("launch_exploration", default_value="true"),
        DeclareLaunchArgument("launch_detector", default_value="true"),
        DeclareLaunchArgument("launch_localizer", default_value="true"),
        DeclareLaunchArgument("launch_memory", default_value="true"),
        DeclareLaunchArgument("launch_semantic_map", default_value="true"),
        DeclareLaunchArgument("launch_semantic_refiner", default_value="true"),
        DeclareLaunchArgument("launch_query", default_value="true"),
        DeclareLaunchArgument("launch_nav_adapter", default_value="true"),
        DeclareLaunchArgument("launch_coordinator", default_value="true"),
        DeclareLaunchArgument("launch_semantic_map_memory", default_value="true"),
        DeclareLaunchArgument("use_runtime_debug", default_value="false"),
        DeclareLaunchArgument("refiner_worker_enabled", default_value="true"),
        DeclareLaunchArgument("refiner_auto_run", default_value="true"),
        DeclareLaunchArgument("query_memory_mode", default_value="semantic_map_state"),
        DeclareLaunchArgument("query_memory_topic", default_value="/semantic_map/compat_objects"),
        DeclareLaunchArgument("query_semantic_map_topic", default_value="/semantic_map/state"),
        DeclareLaunchArgument("query_output_frame", default_value="map"),
        TimerAction(period=5.0, actions=[nav2]),
        TimerAction(period=8.0, actions=[coordinator]),
        TimerAction(period=15.0, actions=[exploration]),
        TimerAction(period=10.0, actions=[detector]),
        TimerAction(period=10.0, actions=[localizer]),
        TimerAction(period=10.0, actions=[memory]),
        TimerAction(period=10.0, actions=[semantic_map]),
        TimerAction(period=10.0, actions=[semantic_refiner]),
        TimerAction(period=10.0, actions=[query]),
        TimerAction(period=10.0, actions=[nav_adapter]),
        TimerAction(period=12.0, actions=[semantic_map_memory]),
        TimerAction(period=12.0, actions=[runtime_debug]),
        rviz_node,
    ])
