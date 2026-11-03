"""TurtleBot3 base + LDS-02 lidar (official turtlebot3_bringup), fail-fast for systemd.

The base listens on /cmd_vel_safe, so the only way to the motors is through semnav_safety.py.

If any process in the group exits (e.g. the lidar driver after a USB hiccup), the whole
launch shuts down and semnav-base.service restarts it, instead of running half-alive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

OPENCR = "/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00"


def _fail_fast(event, _context):
    return [
        LogInfo(msg=f"[semnav] {event.action.name} exited with {event.returncode}; restarting the base group"),
        EmitEvent(event=Shutdown(reason=f"{event.action.name} exited")),
    ]


def generate_launch_description():
    bringup = os.path.join(get_package_share_directory("turtlebot3_bringup"), "launch", "robot.launch.py")
    return LaunchDescription([
        GroupAction([
            SetRemap(src="cmd_vel", dst="cmd_vel_safe"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup),
                launch_arguments={"usb_port": OPENCR}.items(),
            ),
        ]),
        RegisterEventHandler(OnProcessExit(on_exit=_fail_fast)),
    ])
