"""Bring up the ROS side of semantic navigation for NVIDIA Isaac Sim.

Isaac Sim owns physics, simulation time, robot motion, odometry, and sensors.
This launch owns only ROS-side robot description, diagnostics, Nav2/SLAM, and
the existing semantic-navigation stack.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _argument(name: str, default_value, description: str = ""):
    return DeclareLaunchArgument(
        name,
        default_value=default_value,
        description=description,
    )


def generate_launch_description():
    """Build the Isaac Sim ROS-side launch description."""

    use_sim_time = LaunchConfiguration("use_sim_time")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    package_share = FindPackageShare("isaac_semantic_nav")
    coordinator_share = FindPackageShare("tb3_coordinator")
    stack_launch = PathJoinSubstitution(
        [coordinator_share, "launch", "semantic_nav_stack.launch.py"]
    )
    default_nav2_params = PathJoinSubstitution(
        [package_share, "config", "nav2_isaac_waffle_pi.yaml"]
    )
    default_nav2_map = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "maps", "warehouse.yaml"]
    )
    default_slam_params = PathJoinSubstitution(
        [coordinator_share, "config", "slam", "waffle_pi_real_slam.yaml"]
    )

    robot_urdf = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("robot_description_package")),
            "urdf",
            LaunchConfiguration("robot_description_file"),
        ]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_urdf]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_robot_state_publisher")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                "frame_prefix": LaunchConfiguration("frame_prefix"),
            }
        ],
    )

    semantic_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stack_launch),
        condition=IfCondition(LaunchConfiguration("launch_stack")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": LaunchConfiguration("use_rviz"),
            "nav2_map": LaunchConfiguration("nav2_map"),
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "map_topic": LaunchConfiguration("map_topic"),
            "costmap_topic": LaunchConfiguration("costmap_topic"),
            "odom_topic": odom_topic,
            "scan_topic": scan_topic,
            "image_topic": image_topic,
            "camera_info_topic": camera_info_topic,
            "semantic_map_state_topic": LaunchConfiguration(
                "semantic_map_state_topic"
            ),
            "semantic_map_compat_topic": LaunchConfiguration(
                "semantic_map_compat_topic"
            ),
            "semantic_map_markers_topic": LaunchConfiguration(
                "semantic_map_markers_topic"
            ),
            "semantic_map_status_topic": LaunchConfiguration(
                "semantic_map_status_topic"
            ),
            "semantic_map_debug_image_topic": LaunchConfiguration(
                "semantic_map_debug_image_topic"
            ),
            "semantic_memory_state_topic": LaunchConfiguration(
                "semantic_memory_state_topic"
            ),
            "detector_device": LaunchConfiguration("detector_device"),
            "motion_initially_armed": LaunchConfiguration(
                "motion_initially_armed"
            ),
            "readiness_required": LaunchConfiguration("readiness_required"),
            "readiness_map_frame": LaunchConfiguration("map_frame"),
            "readiness_odom_frame": LaunchConfiguration("odom_frame"),
            "readiness_base_frame": LaunchConfiguration("base_frame"),
            "readiness_camera_frame": LaunchConfiguration("camera_frame"),
            "exploration_initially_enabled": LaunchConfiguration(
                "exploration_initially_enabled"
            ),
            "launch_startup_warmup": LaunchConfiguration(
                "launch_startup_warmup"
            ),
            "require_startup_warmup": LaunchConfiguration(
                "require_startup_warmup"
            ),
            "launch_nav2": LaunchConfiguration("launch_nav2"),
            "launch_exploration": LaunchConfiguration("launch_exploration"),
            "launch_detector": LaunchConfiguration("launch_detector"),
            "launch_localizer": LaunchConfiguration("launch_localizer"),
            "launch_memory": LaunchConfiguration("launch_memory"),
            "launch_semantic_map": LaunchConfiguration("launch_semantic_map"),
            "launch_semantic_refiner": LaunchConfiguration(
                "launch_semantic_refiner"
            ),
            "launch_query": LaunchConfiguration("launch_query"),
            "launch_nav_adapter": LaunchConfiguration("launch_nav_adapter"),
            "launch_coordinator": LaunchConfiguration("launch_coordinator"),
            "launch_semantic_map_memory": LaunchConfiguration(
                "launch_semantic_map_memory"
            ),
            "use_runtime_debug": LaunchConfiguration("use_runtime_debug"),
            "refiner_worker_enabled": LaunchConfiguration(
                "refiner_worker_enabled"
            ),
            "refiner_auto_run": LaunchConfiguration("refiner_auto_run"),
            "query_memory_mode": LaunchConfiguration("query_memory_mode"),
            "query_memory_topic": LaunchConfiguration("query_memory_topic"),
            "query_semantic_map_topic": LaunchConfiguration(
                "query_semantic_map_topic"
            ),
            "query_output_frame": LaunchConfiguration("query_output_frame"),
        }.items(),
    )

    contract_checker = Node(
        package="isaac_semantic_nav",
        executable="contract_checker",
        name="isaac_contract_checker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_contract_checker")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "clock_topic": LaunchConfiguration("clock_topic"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "odom_topic": odom_topic,
                "tf_topic": LaunchConfiguration("tf_topic"),
                "tf_static_topic": LaunchConfiguration("tf_static_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": scan_topic,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "status_topic": LaunchConfiguration("contract_status_topic"),
                "startup_grace_sec": LaunchConfiguration(
                    "contract_startup_grace_sec"
                ),
                "report_period_sec": LaunchConfiguration(
                    "contract_report_period_sec"
                ),
                "rate_window_sec": LaunchConfiguration(
                    "contract_rate_window_sec"
                ),
                "max_message_age_sec": LaunchConfiguration(
                    "contract_max_message_age_sec"
                ),
                "shutdown_on_failure": LaunchConfiguration(
                    "contract_shutdown_on_failure"
                ),
                "failure_reports_before_shutdown": LaunchConfiguration(
                    "contract_failure_reports_before_shutdown"
                ),
                "require_joint_states": LaunchConfiguration(
                    "contract_require_joint_states"
                ),
                "require_imu": LaunchConfiguration("contract_require_imu"),
                "require_tf_static_topic": LaunchConfiguration(
                    "contract_require_tf_static_topic"
                ),
                "clock_min_rate_hz": LaunchConfiguration("clock_min_rate_hz"),
                "joint_states_min_rate_hz": LaunchConfiguration(
                    "joint_states_min_rate_hz"
                ),
                "odom_min_rate_hz": LaunchConfiguration("odom_min_rate_hz"),
                "tf_min_rate_hz": LaunchConfiguration("tf_min_rate_hz"),
                "imu_min_rate_hz": LaunchConfiguration("imu_min_rate_hz"),
                "scan_min_rate_hz": LaunchConfiguration("scan_min_rate_hz"),
                "image_min_rate_hz": LaunchConfiguration("image_min_rate_hz"),
                "camera_info_min_rate_hz": LaunchConfiguration(
                    "camera_info_min_rate_hz"
                ),
                "map_frame": LaunchConfiguration("map_frame"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_footprint_frame": LaunchConfiguration(
                    "base_footprint_frame"
                ),
                "base_frame": LaunchConfiguration("base_frame"),
                "scan_frame": LaunchConfiguration("scan_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "camera_optical_frame": LaunchConfiguration(
                    "camera_optical_frame"
                ),
                "require_map_frame": LaunchConfiguration(
                    "contract_require_map_frame"
                ),
                "require_camera_optical_frame": LaunchConfiguration(
                    "contract_require_camera_optical_frame"
                ),
                "tf_timeout_sec": LaunchConfiguration("contract_tf_timeout_sec"),
            }
        ],
    )

    arguments = [
        _argument("use_sim_time", "true", "Use the Isaac Sim /clock source."),
        _argument("launch_stack", "true"),
        _argument("stack_start_delay_sec", "5.0"),
        _argument("use_rviz", "true"),
        _argument("launch_robot_state_publisher", "true"),
        _argument("robot_description_package", "turtlebot3_description"),
        _argument("robot_description_file", "turtlebot3_waffle_pi.urdf"),
        _argument("frame_prefix", ""),
        _argument("nav2_map", default_nav2_map),
        _argument("nav2_params_file", default_nav2_params),
        _argument("slam_params_file", default_slam_params),
        _argument("map_topic", "/map"),
        _argument("costmap_topic", "/global_costmap/costmap"),
        _argument("odom_topic", "/odom"),
        _argument("scan_topic", "/scan"),
        _argument("image_topic", "/camera/image_raw"),
        _argument("camera_info_topic", "/camera/camera_info"),
        _argument("clock_topic", "/clock"),
        _argument("joint_states_topic", "/joint_states"),
        _argument("tf_topic", "/tf"),
        _argument("tf_static_topic", "/tf_static"),
        _argument("imu_topic", "/imu"),
        _argument(
            "cmd_vel_topic",
            "/cmd_vel",
            "Unstamped geometry_msgs/msg/Twist consumed by Isaac Sim.",
        ),
        _argument("map_frame", "map"),
        _argument("odom_frame", "odom"),
        _argument("base_footprint_frame", "base_footprint"),
        _argument("base_frame", "base_link"),
        _argument("scan_frame", "base_scan"),
        _argument("camera_frame", "camera_rgb_frame"),
        _argument("camera_optical_frame", "camera_rgb_optical_frame"),
        _argument("semantic_map_state_topic", "/semantic_map/state"),
        _argument("semantic_map_compat_topic", "/semantic_map/compat_objects"),
        _argument("semantic_map_markers_topic", "/semantic_map/markers"),
        _argument("semantic_map_status_topic", "/semantic_map/status"),
        _argument("semantic_map_debug_image_topic", "/semantic_map/debug_image"),
        _argument("semantic_memory_state_topic", ""),
        _argument("detector_device", "cuda:0"),
        _argument("motion_initially_armed", "true"),
        _argument("readiness_required", "true"),
        _argument("exploration_initially_enabled", "true"),
        _argument("launch_startup_warmup", "true"),
        _argument("require_startup_warmup", "true"),
        _argument("launch_nav2", "true"),
        _argument("launch_exploration", "true"),
        _argument("launch_detector", "true"),
        _argument("launch_localizer", "true"),
        _argument("launch_memory", "true"),
        _argument("launch_semantic_map", "true"),
        _argument("launch_semantic_refiner", "true"),
        _argument("launch_query", "true"),
        _argument("launch_nav_adapter", "true"),
        _argument("launch_coordinator", "true"),
        _argument("launch_semantic_map_memory", "true"),
        _argument("use_runtime_debug", "false"),
        _argument("refiner_worker_enabled", "true"),
        _argument("refiner_auto_run", "true"),
        _argument("query_memory_mode", "semantic_map_state"),
        _argument("query_memory_topic", "/semantic_map/compat_objects"),
        _argument("query_semantic_map_topic", "/semantic_map/state"),
        _argument("query_output_frame", "map"),
        _argument("launch_contract_checker", "true"),
        _argument("contract_status_topic", "/isaac_semantic_nav/contract_status"),
        _argument("contract_startup_grace_sec", "45.0"),
        _argument("contract_report_period_sec", "2.0"),
        _argument("contract_rate_window_sec", "5.0"),
        _argument("contract_max_message_age_sec", "2.5"),
        _argument("contract_shutdown_on_failure", "false"),
        _argument("contract_failure_reports_before_shutdown", "3"),
        _argument("contract_require_joint_states", "true"),
        _argument("contract_require_imu", "true"),
        _argument("contract_require_tf_static_topic", "true"),
        _argument("contract_require_map_frame", "true"),
        _argument("contract_require_camera_optical_frame", "true"),
        _argument("contract_tf_timeout_sec", "0.05"),
        _argument("clock_min_rate_hz", "10.0"),
        _argument("joint_states_min_rate_hz", "5.0"),
        _argument("odom_min_rate_hz", "10.0"),
        _argument("tf_min_rate_hz", "10.0"),
        _argument("imu_min_rate_hz", "20.0"),
        _argument("scan_min_rate_hz", "5.0"),
        _argument("image_min_rate_hz", "10.0"),
        _argument("camera_info_min_rate_hz", "1.0"),
    ]

    return LaunchDescription(
        [
            *arguments,
            robot_state_publisher,
            contract_checker,
            TimerAction(
                period=LaunchConfiguration("stack_start_delay_sec"),
                actions=[semantic_stack],
            ),
        ]
    )
