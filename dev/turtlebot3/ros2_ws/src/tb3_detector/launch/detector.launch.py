from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_detector")

    # ── Launch arguments ────────────────────────────────────────────────
    # Only hardware/runtime parameters are controlled from launch:
    #
    #   model_path   — resolved to an absolute install path here so the node
    #                  does not have to guess where pkg_share lives at runtime.
    #   device       — useful to flip to "cuda:0" without editing the yaml.
    #   use_sim_time — must be set at launch time (clock source is external).
    #   image/camera_info topics — differ between sim and real robot bringup.
    #
    # ALL other parameters (conf_threshold, class_filter, publish_debug_image,
    # enable_tracking) come from detector.yaml.
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([pkg_share, "models", "yolov8n.pt"]),
            description="Absolute path to YOLOv8 .pt weights.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Torch inference device: 'cpu' or 'cuda:0'.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when running against a simulator clock.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="RGB image topic consumed by the detector.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera_info",
            description="CameraInfo topic paired with image_topic.",
        ),

        # ── detector_node ────────────────────────────────────────────────
        # Parameter resolution order (later entries win):
        #   1. detector.yaml  — provides all defaults
        #   2. inline dict    — overrides only the three launch-controlled keys
        Node(
            package="tb3_detector",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "detector.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "model_path":   LaunchConfiguration("model_path"),
                    "device":       LaunchConfiguration("device"),
                    "image_topic":  LaunchConfiguration("image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                },
            ],
        ),
    ])
