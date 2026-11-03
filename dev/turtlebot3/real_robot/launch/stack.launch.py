"""Real-robot semantic navigation stack (SLAM, Nav2, YOLOE semantic map, memory, query,
coordinator), fail-fast for systemd.

Same arguments as the hand-run setup: GPU detector inside semantic_map_node only (no
standalone detector_node), normalized lidar on /scan_slam, refiner off, no RViz. Motion
starts unarmed (real_semantic_nav.launch.py forces that).

If any node exits, the whole stack shuts down and semnav-stack.service restarts it, which
gives a fresh map and memory rather than a silently degraded pipeline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _fail_fast(event, _context):
    return [
        LogInfo(msg=f"[semnav] {event.action.name} exited with {event.returncode}; restarting the stack"),
        EmitEvent(event=Shutdown(reason=f"{event.action.name} exited")),
    ]


def generate_launch_description():
    real = os.path.join(get_package_share_directory("tb3_coordinator"), "launch", "real_semantic_nav.launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real),
            launch_arguments={
                "detector_device": "cuda:0",
                "launch_detector": "false",
                # Superseded by semantic_map_node and semantic_map_memory_node on this robot.
                "launch_localizer": "false",
                "launch_memory": "false",
                "scan_topic": "/scan_slam",
                "refiner_worker_enabled": "false",
                "refiner_auto_run": "false",
                "use_rviz": "false",
            }.items(),
        ),
        RegisterEventHandler(OnProcessExit(on_exit=_fail_fast)),
    ])
