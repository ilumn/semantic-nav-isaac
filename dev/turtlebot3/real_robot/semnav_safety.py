#!/usr/bin/env python3
"""semnav safety gate: the only path from velocity commands to the motors.

The base listens on /cmd_vel_safe (remapped in base.launch.py). Everything that wants to move
the robot (Nav2, the coordinator, the control node, teleop) still publishes /cmd_vel, and this
node decides what gets through, at a fixed 20 Hz:

  disabled       starts disabled; semnav-control enables it on arm/look and disables on disarm
  dry run        /mnt/data/semnav/dry_run exists: everything runs, the wheels get zero
  stale inputs   /cmd_vel older than 0.5 s, or /scan or /odom older than 0.5 s, gives zero
  obstacle       a lidar return in the swept corridor ahead/behind zeroes that direction; a
                 return the chassis would hit when turning zeroes that turn direction only,
                 so the robot can always turn away from what is next to it
  stall          commanded motion with no odometry response for 1.5 s holds for 2 s; three
                 stalls within 60 s disable the gate until the next enable

Status is published on /semnav/safety (JSON, 2 Hz and on every enable).
"""

import json
import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

DRY_RUN_FILE = "/mnt/data/semnav/dry_run"

RATE_HZ = 20.0
CMD_STALE_S = 0.5
SENSOR_STALE_S = 0.5

# Footprint in base_link (TurtleBot3 Waffle: the axle sits well forward of the chassis centre).
FRONT_X = 0.077
REAR_X = -0.205
HALF_WIDTH = 0.155
TURN_RADIUS = 0.28       # returns further than this cannot be reached by turning in place
TURN_LOOKAHEAD = (0.15, 0.3)  # rad of rotation checked ahead of the current heading
LIDAR_X = -0.064         # base_scan in base_link
SIDE_MARGIN = 0.03
STOP_MARGIN = 0.06       # m of clearance kept at zero speed
STOP_TIME = 0.5          # s of travel added to the margin, covers scan age and braking

STALL_V = 0.04           # m/s commanded before a stall can be declared
STALL_W = 0.25           # rad/s
STALL_V_ODOM = 0.01      # m/s
STALL_W_ODOM = 0.03      # rad/s
STALL_TIME_S = 1.5
STALL_HOLD_S = 2.0
STALL_WINDOW_S = 60.0
STALL_LIMIT = 3


class SafetyGate:
    def __init__(self) -> None:
        self.node = rclpy.create_node("semnav_safety")
        self.log = self.node.get_logger()
        n = self.node

        self.enabled = False
        self.disabled_reason = "not enabled yet"
        self.enabled_at: Optional[float] = None
        self.first_cmd_latency: Optional[float] = None
        self.dry_run = os.path.exists(DRY_RUN_FILE)

        self.cmd = (0.0, 0.0)
        self.cmd_time: Optional[float] = None
        self.scan_time: Optional[float] = None
        self.odom_time: Optional[float] = None
        self.odom = (0.0, 0.0)

        # A return must show up in two consecutive scans to count, which filters single-ray noise.
        self.blocked_prev = (False, False, False, False)
        self.blocked = (False, False, False, False)   # forward, reverse, turn left, turn right
        self.nearest: Optional[float] = None

        self.stall_since: Optional[float] = None
        self.hold_until = 0.0
        self.stalls: list = []
        self.stall_total = 0
        self.last_block = ""
        self.out = (0.0, 0.0)

        sensor = qos_profile_sensor_data
        n.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        n.create_subscription(LaserScan, "/scan", self._on_scan, sensor)
        n.create_subscription(Odometry, "/odom", self._on_odom, sensor)
        self.out_pub = n.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.status_pub = n.create_publisher(String, "/semnav/safety", 10)
        n.create_service(SetBool, "/semnav/safety/enable", self._on_enable)
        n.create_timer(1.0 / RATE_HZ, self._tick)
        n.create_timer(0.5, self._publish_status)
        n.create_timer(1.0, self._check_dry_run)
        self.log.info(f"safety gate up, disabled, dry_run={self.dry_run}")

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = time.monotonic()
        if (self.enabled and self.first_cmd_latency is None and self.enabled_at is not None
                and (abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3)):
            self.first_cmd_latency = self.cmd_time - self.enabled_at
            self.log.info(f"first motion command {self.first_cmd_latency:.2f} s after enable")

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.odom_time = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_time = time.monotonic()
        v = abs(self.cmd[0])
        margin = STOP_MARGIN + v * STOP_TIME
        half = HALF_WIDTH + SIDE_MARGIN
        forward = reverse = left = right = False
        nearest = math.inf
        for i, r in enumerate(msg.ranges):
            if not (0.05 < r < 3.0):
                continue
            a = msg.angle_min + i * msg.angle_increment
            x = LIDAR_X + r * math.cos(a)
            y = r * math.sin(a)
            if REAR_X <= x <= FRONT_X and abs(y) <= HALF_WIDTH:
                continue  # inside the chassis: the robot's own hardware
            nearest = min(nearest, r)
            if abs(y) <= half:
                if FRONT_X < x <= FRONT_X + margin:
                    forward = True
                elif REAR_X - margin <= x < REAR_X:
                    reverse = True
            if math.hypot(x, y) <= TURN_RADIUS:
                left = left or self._hit_when_turning(x, y, 1.0)
                right = right or self._hit_when_turning(x, y, -1.0)
        self.nearest = None if math.isinf(nearest) else nearest
        now = (forward, reverse, left, right)
        self.blocked = tuple(a and b for a, b in zip(now, self.blocked_prev))
        self.blocked_prev = now

    @staticmethod
    def _hit_when_turning(x: float, y: float, direction: float) -> bool:
        """Would the footprint reach this point after turning a little in this direction?"""
        for angle in TURN_LOOKAHEAD:
            c, s = math.cos(direction * angle), math.sin(direction * angle)
            # The point as seen from the chassis once it has turned.
            px, py = c * x + s * y, -s * x + c * y
            if REAR_X - SIDE_MARGIN <= px <= FRONT_X + SIDE_MARGIN and abs(py) <= HALF_WIDTH + SIDE_MARGIN:
                return True
        return False

    def _on_enable(self, request, response):
        self._set_enabled(bool(request.data), "disabled by control")
        response.success = True
        response.message = f"enabled={self.enabled} dry_run={self.dry_run}"
        return response

    def _set_enabled(self, value: bool, reason: str) -> None:
        if value and not self.enabled:
            self.enabled_at = time.monotonic()
            self.first_cmd_latency = None
            self.stalls.clear()
            self.stall_since = None
            self.hold_until = 0.0
        self.enabled = value
        self.disabled_reason = "" if value else reason
        self.log.info(f"enabled={value}" + ("" if value else f" ({reason})"))
        self._publish_status()

    def _check_dry_run(self) -> None:
        dry = os.path.exists(DRY_RUN_FILE)
        if dry != self.dry_run:
            self.dry_run = dry
            self.log.warning(f"dry_run={dry}")

    @staticmethod
    def _stale(stamp: Optional[float], limit: float, now: float) -> bool:
        return stamp is None or now - stamp > limit

    def _block_reason(self, now: float, v: float, w: float) -> tuple:
        """Return (v, w, reason) after applying the sensor and obstacle rules."""
        if self._stale(self.scan_time, SENSOR_STALE_S, now):
            return 0.0, 0.0, "stale /scan"
        if self._stale(self.odom_time, SENSOR_STALE_S, now):
            return 0.0, 0.0, "stale /odom"
        reasons = []
        if v > 0.0 and self.blocked[0]:
            v = 0.0
            reasons.append("obstacle ahead")
        elif v < 0.0 and self.blocked[1]:
            v = 0.0
            reasons.append("obstacle behind")
        if (w > 0.0 and self.blocked[2]) or (w < 0.0 and self.blocked[3]):
            w = 0.0
            reasons.append("obstacle when turning")
        return v, w, ", ".join(reasons)

    def _update_stall(self, now: float, v: float, w: float) -> None:
        moving_cmd = abs(v) >= STALL_V or abs(w) >= STALL_W
        linear_ok = abs(v) < STALL_V or abs(self.odom[0]) >= STALL_V_ODOM
        angular_ok = abs(w) < STALL_W or abs(self.odom[1]) >= STALL_W_ODOM
        if not moving_cmd or (linear_ok and angular_ok):
            self.stall_since = None
            return
        if self.stall_since is None:
            self.stall_since = now
        elif now - self.stall_since >= STALL_TIME_S:
            self.stall_since = None
            self.hold_until = now + STALL_HOLD_S
            self.stall_total += 1
            self.stalls = [t for t in self.stalls if now - t < STALL_WINDOW_S] + [now]
            self.log.error(f"STALL: cmd v={v:.3f} w={w:.3f}, odom v={self.odom[0]:.3f} w={self.odom[1]:.3f}; "
                           f"holding {STALL_HOLD_S:.0f} s ({len(self.stalls)}/{STALL_LIMIT})")
            if len(self.stalls) >= STALL_LIMIT:
                self._set_enabled(False, f"{STALL_LIMIT} stalls in {STALL_WINDOW_S:.0f} s")

    def _tick(self) -> None:
        now = time.monotonic()
        v, w = (0.0, 0.0) if self._stale(self.cmd_time, CMD_STALE_S, now) else self.cmd
        reason = ""
        if not self.enabled:
            v, w = 0.0, 0.0
        elif now < self.hold_until:
            v, w, reason = 0.0, 0.0, "stall hold"
        else:
            v, w, reason = self._block_reason(now, v, w)
            if not self.dry_run:
                self._update_stall(now, v, w)
        if reason != self.last_block:
            if reason:
                self.log.warning(f"blocking: {reason} (cmd v={self.cmd[0]:.3f} w={self.cmd[1]:.3f}, nearest {self.nearest})")
            self.last_block = reason
        if self.dry_run:
            v, w = 0.0, 0.0
        self.out = (v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.out_pub.publish(msg)

    def _publish_status(self) -> None:
        self.status_pub.publish(String(data=json.dumps({
            "enabled": self.enabled,
            "disabled_reason": self.disabled_reason,
            "dry_run": self.dry_run,
            "blocking": self.last_block,
            "nearest_m": None if self.nearest is None else round(self.nearest, 2),
            "stalls": self.stall_total,
            "cmd": [round(c, 3) for c in self.cmd],
            "out": [round(c, 3) for c in self.out],
            "first_cmd_latency_s": None if self.first_cmd_latency is None else round(self.first_cmd_latency, 2),
        })))


def main() -> None:
    rclpy.init()
    gate = SafetyGate()
    try:
        rclpy.spin(gate.node)
    finally:
        # Leave the wheels stopped if the gate itself goes away.
        if rclpy.ok():
            gate.out_pub.publish(Twist())
            rclpy.shutdown()


if __name__ == "__main__":
    main()
