#!/usr/bin/env python3
"""semnav control node: the one place that commands robot motion, plus health and a watchdog.

Runs as semnav-control.service. The `semnav` CLI talks to it over a UNIX socket, so a command
takes ~0.1 s instead of starting ROS command-line tools (5-10 s on this Jetson).

Commands (one line per connection; replies stream back; the connection closes when done):
  status [--json]        inputs, localization, detector, memory, coordinator, watchdog
  arm                    motor power on, arm the coordinator, frontier exploration OFF
  explore                arm with frontier exploration ON
  disarm | stop          disarm, cancel goals, zero velocity (stop also aborts look/find/go)
  go <text>              arm (exploration off), send the semantic command, wait, disarm
  look <name>            tether-safe stop-and-look until <name> is in memory (stays unarmed)
  find <sentence>        work out the object from the sentence ("the trash can", "the sofa",
                         "somewhere to throw this away", "the chair near the door"); go if
                         it is in memory, otherwise explore until it is, then go
  understand <sentence>  say which object find would look for, without moving

Watchdog: while the coordinator is armed or a look/find is turning the robot, stale /scan_slam
or /odom (> 2.5 s) or map -> base_link (> 3.0 s) disarms, cancels goals and sends zero velocity. The safety gate (semnav_safety.py) sits between /cmd_vel and the motors: this node
enables it while armed or turning and disables it otherwise, and treats the gate going away or
disabling itself as a reason to disarm.
"""

import json
import math
import os
import re
import socket
import threading
import time
from collections import Counter, deque
from typing import Optional

import rclpy
import tf2_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tb3_query.query_core import parse_command
from tb3_semantic_map_msgs.msg import SemanticMapState

SOCKET_PATH = os.path.join(os.environ.get("XDG_RUNTIME_DIR", "/tmp"), "semnav.sock")

SAFETY_STALE_S = 3.0     # the gate stops the wheels by itself; this only catches it having died
# The gate stops the wheels within 0.5 s of stale data and carries on when it recovers, so these
# only end a mission for a real outage. NTP clock steps alone cause gaps of about a second.
SCAN_STALE_S = 2.5
ODOM_STALE_S = 2.5
TF_STALE_S = 3.0

TURN_GAIN = 1.5          # rad/s per rad of heading error
TURN_MAX = 0.45          # rad/s
TURN_MIN = 0.12          # rad/s, enough to overcome static friction on carpet
TURN_TOL = math.radians(3.0)
LOOK_STEP_DEG = 30.0
LOOK_DWELL_S = 2.0
GO_TIMEOUT_S = 180.0
FIND_TIMEOUT_S = 300.0
FIND_NO_FRONTIERS_S = 20.0   # this long with nothing left to explore ends the search
NOT_REMEMBERED = {"person"}  # excluded_classes in coordinator.yaml: never reaches memory

# Sentence understanding: exact words first, then meaning via the MobileCLIP text encoder that
# YOLOE already uses. A meaning match needs a clear winner, so "a plant" or "the fridge" are
# refused rather than guessed as a chair or a door.
MEANING_MIN_SCORE = 0.70
MEANING_MIN_MARGIN = 0.02
COMMAND_PREFIX = re.compile(
    r"^(please |can you |could you )?(find|go to|go|navigate to|take me to|get to|look for|search for)?"
    r"( (the|a|an|my|some))?\b\s*")

COORD_STATUS = re.compile(r"^\[(\w+) armed=(\w+) ready=(\w+)\]\s*(.*)$")


class Rate:
    """Receipt times of one input over a sliding window."""

    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self.times: deque = deque()
        self.last: Optional[float] = None

    def tick(self) -> None:
        now = time.monotonic()
        self.last = now
        self.times.append(now)
        self._prune(now)

    def _prune(self, now: float) -> None:
        while self.times and now - self.times[0] > self.window_s:
            self.times.popleft()

    def age(self) -> Optional[float]:
        return None if self.last is None else time.monotonic() - self.last

    def hz(self) -> float:
        self._prune(time.monotonic())
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / max(self.times[-1] - self.times[0], 1e-3)


class Cancelled(Exception):
    pass


class SemnavControl:
    def __init__(self) -> None:
        self.node = rclpy.create_node("semnav_control")
        self.log = self.node.get_logger()
        n = self.node

        self.rates = {name: Rate() for name in ("scan_raw", "scan_slam", "odom", "camera_info", "semantic_status")}
        self.yaw_total = 0.0          # unwrapped odometry yaw (rad)
        self._last_yaw: Optional[float] = None
        self.yaw_rate = 0.0
        self.coord = {"mode": None, "armed": False, "ready": False, "text": "", "rate": Rate(30.0)}
        self.semantic = {"detections": None, "grounded": None, "entities": None, "map": None,
                         "sampled": None, "fps": 0.0, "_prev": None}
        self.memory: list = []
        self.tf = {"ok": False, "age": None, "x": None, "y": None, "yaw": None}
        self.watchdog_last: Optional[dict] = None
        self._watchdog_cooldown_until = 0.0

        self.mission_lock = threading.Lock()
        self.mission = {"name": None, "detail": ""}
        self.cancel = threading.Event()
        self.turning = False
        self.frontiers_seen_at = time.monotonic()
        self.safety: dict = {}
        self.safety_rate = Rate()
        self._safety_enabled_at = 0.0

        sensor = qos_profile_sensor_data
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(LaserScan, "/scan", lambda _m: self.rates["scan_raw"].tick(), sensor)
        n.create_subscription(LaserScan, "/scan_slam", lambda _m: self.rates["scan_slam"].tick(), sensor)
        n.create_subscription(Odometry, "/odom", self._on_odom, sensor)
        n.create_subscription(CameraInfo, "/camera/camera_info", lambda _m: self.rates["camera_info"].tick(), sensor)
        n.create_subscription(String, "/semantic_map/live_status", self._on_semantic_status, reliable)
        n.create_subscription(SemanticMapState, "/semantic_map/state", self._on_memory, reliable)
        n.create_subscription(String, "/coordinator_node/status", self._on_coordinator, reliable)
        n.create_subscription(String, "/semnav/safety", self._on_safety, 10)
        n.create_subscription(PoseArray, "/frontiers", self._on_frontiers, 10)

        self.cmd_vel = n.create_publisher(Twist, "/cmd_vel", 10)
        self.explore_pub = n.create_publisher(Bool, "/exploration_enabled", reliable)
        self.command_pub = n.create_publisher(String, "/user_command", reliable)
        self.health_pub = n.create_publisher(String, "/semnav/health", reliable)
        self.motor_cli = n.create_client(SetBool, "/motor_power")
        self.arm_cli = n.create_client(SetBool, "/coordinator_node/set_motion_armed")
        self.safety_cli = n.create_client(SetBool, "/semnav/safety/enable")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, n)

        self.sem2det, self.det2sem = self._load_targets()
        self.text_model = None
        self.text_model_error = ""
        self.text_model_ready = threading.Event()
        threading.Thread(target=self._load_text_model, daemon=True).start()

        n.create_timer(0.2, self._update_tf)
        n.create_timer(0.1, self._watchdog)
        n.create_timer(1.0, lambda: self.health_pub.publish(String(data=json.dumps(self.health()))))

    # ── inputs ──────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._last_yaw is not None:
            self.yaw_total += math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))
        self._last_yaw = yaw
        self.yaw_rate = msg.twist.twist.angular.z
        self.rates["odom"].tick()

    def _on_semantic_status(self, msg: String) -> None:
        self.rates["semantic_status"].tick()
        fields = dict(re.findall(r"(\w+)=([^\s|]+)", msg.data))
        now = time.monotonic()
        try:
            sampled = int(fields.get("sampled_frames", ""))
        except ValueError:
            sampled = None
        prev = self.semantic["_prev"]
        if sampled is not None and prev is not None and now > prev[0]:
            self.semantic["fps"] = max(0.0, (sampled - prev[1]) / (now - prev[0]))
        if sampled is not None:
            self.semantic["_prev"] = (now, sampled)
        self.semantic.update({
            "detections": fields.get("detections"), "grounded": fields.get("grounded"),
            "entities": fields.get("entities"), "map": fields.get("map") == "True", "sampled": sampled,
        })

    def _on_memory(self, msg: SemanticMapState) -> None:
        self.memory = [(e.semantic_name, e.detector_label, e.pose.position.x, e.pose.position.y) for e in msg.entities]

    def _on_coordinator(self, msg: String) -> None:
        match = COORD_STATUS.match(msg.data)
        if not match:
            return
        self.coord.update({"mode": match.group(1), "armed": match.group(2) == "True",
                           "ready": match.group(3) == "True", "text": match.group(4)})
        self.coord["rate"].tick()

    def _on_frontiers(self, msg: PoseArray) -> None:
        if msg.poses:
            self.frontiers_seen_at = time.monotonic()

    def _on_safety(self, msg: String) -> None:
        self.safety = json.loads(msg.data)
        self.safety_rate.tick()

    def _update_tf(self) -> None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
            self.tf.update({"ok": False})
            return
        stamp = Time.from_msg(t.header.stamp)
        age = (self.node.get_clock().now() - stamp).nanoseconds * 1e-9
        r = t.transform.rotation
        self.tf.update({
            "ok": age < TF_STALE_S, "age": age,
            "x": t.transform.translation.x, "y": t.transform.translation.y,
            "yaw": math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z)),
        })

    def _load_targets(self):
        try:
            path = os.path.join(get_package_share_directory("tb3_frontier_exploration"), "config", "semantic_targets.yaml")
            targets = [t for t in yaml.safe_load(open(path))["semantic_targets"] if t.get("enabled")]
            return ({t["semantic_name"]: t["detector_label"] for t in targets},
                    {t["detector_label"]: t["semantic_name"] for t in targets})
        except Exception as exc:  # noqa: BLE001
            self.log.warning(f"could not read semantic targets: {exc}")
            return {}, {}

    def _load_text_model(self) -> None:
        try:
            import torch
            from ultralytics.nn.text_model import build_text_model

            model = build_text_model("mobileclip:blt", device="cpu")
            labels = sorted(self.det2sem)
            with torch.no_grad():
                emb = model.encode_text(model.tokenize(labels)).float()
            self.label_embeddings = (labels, emb / emb.norm(dim=-1, keepdim=True))
            self.text_model = model
            self.log.info("text encoder ready for sentence understanding")
        except Exception as exc:  # noqa: BLE001
            self.text_model_error = str(exc)
            self.log.warning(f"text encoder unavailable, find understands exact names only: {exc}")
        finally:
            self.text_model_ready.set()

    def _closest_by_meaning(self, phrase: str) -> list:
        """(label, score) for every target, best first."""
        import torch

        labels, label_emb = self.label_embeddings
        with torch.no_grad():
            emb = self.text_model.encode_text(self.text_model.tokenize([phrase])).float()
        scores = (emb / emb.norm(dim=-1, keepdim=True) @ label_emb.T)[0].tolist()
        return sorted(zip(labels, scores), key=lambda x: -x[1])

    def understand(self, text: str) -> tuple:
        """Return (semantic name, sentence for the query node, how it was understood)."""
        norm = re.sub(r"[_\-]", " ", text.lower()).strip()
        norm = re.sub(r"[^\w\s]", " ", norm)
        norm = re.sub(r"\s+", " ", norm).strip()
        # Multi-word detector labels ("trash can") become their semantic names ("trash").
        for label in sorted(self.det2sem, key=len, reverse=True):
            norm = re.sub(rf"\b{re.escape(label)}\b", self.det2sem[label], norm)
        parsed = parse_command(norm, set(self.sem2det))
        if parsed is not None:
            target = parsed.target_semantic_name
            go_text = norm if parsed.has_relation else f"go to the {target}"
            how = "by name" + (f", {parsed.relation_phrase} the {parsed.reference_semantic_name}"
                               if parsed.has_relation else "")
            return target, go_text, how
        phrase = COMMAND_PREFIX.sub("", norm).strip() or norm
        if not self.text_model_ready.wait(timeout=60.0) or self.text_model is None:
            raise RuntimeError(f"did not recognise an object in '{text}' "
                               f"(meaning matching unavailable: {self.text_model_error or 'still loading'})")
        ranked = self._closest_by_meaning(phrase)
        (best, score), (second, second_score) = ranked[0], ranked[1]
        if score < MEANING_MIN_SCORE or score - second_score < MEANING_MIN_MARGIN:
            raise RuntimeError(f"not sure what '{text}' refers to "
                               f"(closest: {best} {score:.2f}, {second} {second_score:.2f}); "
                               f"known: {', '.join(sorted(self.sem2det))}")
        target = self.det2sem[best]
        return target, f"go to the {target}", f"by meaning, {best} {score:.2f} vs {second} {second_score:.2f}"

    def resolve(self, name: str) -> tuple:
        """Accept a semantic name ("trash") or a detector label ("trash can"); return both."""
        name = name.strip().lower()
        if name in self.sem2det:
            return name, self.sem2det[name]
        if name in self.det2sem:
            return self.det2sem[name], name
        return name, name

    # ── health and watchdog ─────────────────────────────────────────────

    def problems(self) -> list:
        out = []

        def stale(key, limit):
            age = self.rates[key].age()
            return age is None or age > limit

        if self.safety_rate.age() is None or self.safety_rate.age() > SAFETY_STALE_S:
            out.append("safety gate: not running (`semnav restart safety`)")
        if stale("scan_raw", SCAN_STALE_S):
            out.append("lidar: no /scan data (LDS cable? `semnav restart base`)")
        elif stale("scan_slam", SCAN_STALE_S):
            out.append("scan normalizer: no /scan_slam (`semnav restart scan`)")
        if stale("odom", ODOM_STALE_S):
            out.append("base: no /odom (OpenCR? `semnav restart base`)")
        if stale("camera_info", 2.0):
            out.append("camera: no frames (`semnav restart camera`)")
        if stale("semantic_status", 5.0):
            out.append("semantic stack not running (`semnav up`)")
        else:
            if not self.tf["ok"]:
                out.append("localization: no fresh map -> base_link (SLAM)")
            if self.semantic["map"] is False:
                out.append("semantic map has not received the SLAM map yet")
            if self.semantic["fps"] < 2.0 and self.semantic["sampled"] is not None:
                out.append(f"detector slow: {self.semantic['fps']:.1f} fps")
        return out

    def _watchdog_reason(self) -> Optional[str]:
        for key, limit, label in (("scan_slam", SCAN_STALE_S, "lidar /scan_slam"), ("odom", ODOM_STALE_S, "/odom")):
            age = self.rates[key].age()
            if age is None or age > limit:
                return f"{label} stale ({'never' if age is None else f'{age:.1f} s'})"
        age = self.safety_rate.age()
        if age is None or age > SAFETY_STALE_S:
            return f"safety gate not responding ({'never' if age is None else f'{age:.1f} s'})"
        if time.monotonic() - self._safety_enabled_at > 1.0 and not self.safety.get("enabled"):
            return f"safety gate disabled: {self.safety.get('disabled_reason')}"
        if self.coord["armed"] and not self.tf["ok"]:
            age = self.tf["age"]
            return f"map -> base_link stale ({'missing' if age is None else f'{age:.1f} s'})"
        return None

    def _watchdog(self) -> None:
        active = self.coord["armed"] or self.turning
        if not active or time.monotonic() < self._watchdog_cooldown_until:
            return
        reason = self._watchdog_reason()
        if reason is None:
            return
        self._watchdog_cooldown_until = time.monotonic() + 3.0
        self.watchdog_last = {"time": time.strftime("%H:%M:%S"), "reason": reason}
        self.log.error(f"WATCHDOG: {reason}; disarming and stopping")
        self.cancel.set()
        self._set_bool_async(self.arm_cli, False)
        self._set_bool_async(self.safety_cli, False)
        self._zero_velocity()

    def health(self) -> dict:
        inputs = {k: {"hz": round(r.hz(), 1), "age": None if r.age() is None else round(r.age(), 2)}
                  for k, r in self.rates.items()}
        problems = self.problems()
        coord_age = self.coord["rate"].age()
        return {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "verdict": "ok" if not problems else "degraded",
            "problems": problems,
            "coordinator": {"mode": self.coord["mode"], "armed": self.coord["armed"], "ready": self.coord["ready"],
                            "last": self.coord["text"], "age": None if coord_age is None else round(coord_age, 1)},
            "mission": dict(self.mission) if self.mission["name"] else None,
            "watchdog_last": self.watchdog_last,
            "safety": self.safety,
            "inputs": inputs,
            "localization": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in self.tf.items()},
            "detector": {"fps": round(self.semantic["fps"], 1), "detections": self.semantic["detections"],
                         "grounded": self.semantic["grounded"], "live_entities": self.semantic["entities"],
                         "has_map": self.semantic["map"]},
            "memory": dict(Counter(label for _s, label, _x, _y in self.memory)),
        }

    def status_text(self) -> str:
        h = self.health()
        c, d, loc = h["coordinator"], h["detector"], h["localization"]
        lines = [f"semnav {h['verdict'].upper()}  {h['time']}"]
        lines += [f"  ! {p}" for p in h["problems"]]
        lines.append(f"  motion:   {'ARMED' if c['armed'] else 'unarmed'}  mode={c['mode']}  ready={c['ready']}"
                     + (f"  mission={h['mission']['name']} ({h['mission']['detail']})" if h["mission"] else ""))
        for key in ("scan_raw", "scan_slam", "odom", "camera_info"):
            i = h["inputs"][key]
            last = "-" if i["age"] is None else f"{i['age']:.2f} s ago"
            lines.append(f"  {key:<10}{i['hz']:5.1f} Hz   last {last}")
        lines.append(f"  pose:     {'ok' if loc['ok'] else 'NO'}"
                     + (f" ({loc['x']:.2f}, {loc['y']:.2f}) yaw {math.degrees(loc['yaw']):.0f} deg" if loc.get("x") is not None else ""))
        lines.append(f"  detector: {d['fps']:.1f} fps  detections={d['detections']}  grounded={d['grounded']}  map={d['has_map']}")
        mem = ", ".join(f"{k} x{v}" if v > 1 else k for k, v in sorted(h["memory"].items())) or "empty"
        lines.append(f"  memory:   {mem}")
        s = h["safety"]
        if s:
            lines.append(f"  safety:   {'enabled' if s.get('enabled') else 'off'}{'  DRY RUN (wheels get zero)' if s.get('dry_run') else ''}"
                         f"  nearest {s.get('nearest_m')} m  stalls {s.get('stalls')}"
                         + (f"  BLOCKING: {s['blocking']}" if s.get("blocking") else ""))
        if h["watchdog_last"]:
            lines.append(f"  watchdog: last tripped {h['watchdog_last']['time']}: {h['watchdog_last']['reason']}")
        return "\n".join(lines)

    # ── actuation helpers (called from mission threads) ─────────────────

    def _zero_velocity(self) -> None:
        for _ in range(5):
            self.cmd_vel.publish(Twist())

    def _set_bool_async(self, client, value: bool):
        if not client.service_is_ready():
            return None
        return client.call_async(SetBool.Request(data=value))

    def _set_bool(self, client, value: bool, name: str, timeout: float = 5.0) -> tuple:
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"{name} service not available"
        future = client.call_async(SetBool.Request(data=value))
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            return False, f"{name} did not answer"
        r = future.result()
        return bool(r.success), r.message

    def _check(self) -> None:
        if self.cancel.is_set():
            raise Cancelled()

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check()
            time.sleep(0.05)

    def enable_gate(self) -> None:
        self._safety_enabled_at = time.monotonic()
        ok, msg = self._set_bool(self.safety_cli, True, "safety gate")
        if not ok:
            raise RuntimeError(f"safety gate: {msg}")

    def disarm(self, say) -> None:
        ok, msg = self._set_bool(self.arm_cli, False, "disarm")
        self._set_bool(self.safety_cli, False, "safety gate", timeout=1.0)
        self._zero_velocity()
        say(f"disarmed: {msg}" if ok else f"disarm: {msg}")

    def arm(self, say, explore: bool) -> None:
        problems = [p for p in self.problems() if not p.startswith("detector slow")]
        if problems:
            raise RuntimeError("not arming; " + "; ".join(problems))
        ok, msg = self._set_bool(self.motor_cli, True, "motor_power")
        if not ok:
            raise RuntimeError(f"motor power: {msg}")
        self.enable_gate()
        ok, msg = self._set_bool(self.arm_cli, True, "arm")
        if not ok:
            self._set_bool(self.safety_cli, False, "safety gate", timeout=1.0)
            raise RuntimeError(f"coordinator refused to arm: {msg}")
        if not explore:
            # Arming turns frontier exploration on; a frontier goal sent in the next second
            # would replace a semantic goal in Nav2, so switch it off right away.
            for _ in range(6):
                self.explore_pub.publish(Bool(data=False))
                time.sleep(0.25)
        say(f"armed (exploration {'ON' if explore else 'off'})")

    def turn_to(self, target: float) -> None:
        """Proportional turn to an unwrapped yaw (rad); slows down near the target so it does not overshoot."""
        self.turning = True
        try:
            end = time.monotonic() + 15.0
            cmd = Twist()
            while time.monotonic() < end:
                self._check()
                err = target - self.yaw_total
                if abs(err) < TURN_TOL:
                    if abs(self.yaw_rate) < 0.05:
                        break
                    cmd.angular.z = 0.0
                else:
                    w = max(-TURN_MAX, min(TURN_MAX, TURN_GAIN * err))
                    cmd.angular.z = math.copysign(max(abs(w), TURN_MIN), err)
                self.cmd_vel.publish(cmd)
                time.sleep(0.05)
        finally:
            self._zero_velocity()
            self.turning = False

    def in_memory(self, label: str) -> list:
        return [m for m in self.memory if m[1] == label]

    # ── missions ────────────────────────────────────────────────────────

    def mission_look(self, say, name: str) -> bool:
        semantic, label = self.resolve(name)
        if self.coord["armed"]:
            raise RuntimeError("disarm first; look turns the robot directly")
        if self.in_memory(label):
            m = self.in_memory(label)[0]
            say(f"{semantic} ('{label}') already in memory at ({m[2]:.2f}, {m[3]:.2f})")
            return True
        ok, msg = self._set_bool(self.motor_cli, True, "motor_power")
        if not ok:
            raise RuntimeError(f"motor power: {msg}")
        self.enable_gate()
        start = self.yaw_total
        right = [LOOK_STEP_DEG * i for i in range(1, int(180 / LOOK_STEP_DEG) + 1)]
        left = [-LOOK_STEP_DEG * i for i in range(1, int(180 / LOOK_STEP_DEG))]
        say(f"looking for {semantic} ('{label}'): right to 180, left to -150, then back (net rotation zero)")
        for heading in [0.0] + right + left:
            self.mission["detail"] = f"heading {heading:+.0f} deg"
            self.turn_to(start + math.radians(heading))
            self._sleep(LOOK_DWELL_S)
            found = self.in_memory(label)
            if found:
                say(f"FOUND {semantic} at ({found[0][2]:.2f}, {found[0][3]:.2f}) after turning to {heading:+.0f} deg")
                return True
            seen = ", ".join(sorted({m[1] for m in self.memory})) or "nothing yet"
            say(f"  {heading:+4.0f} deg: memory has {seen}")
        self.turn_to(start)
        say(f"{semantic} not found; back at the start heading")
        return False

    def check_searchable(self, name: str) -> None:
        semantic, label = self.resolve(name)
        if label not in self.det2sem:
            raise RuntimeError(f"'{name}' is not something the detector looks for; "
                               f"known: {', '.join(sorted(self.sem2det))}")
        if label in NOT_REMEMBERED:
            raise RuntimeError(f"{semantic} is never kept in memory, so it cannot be searched for")

    def mission_search(self, say, name: str) -> bool:
        """Explore until <name> is in memory. Leaves the robot armed: the caller disarms."""
        semantic, label = self.resolve(name)
        self.arm(say, explore=True)
        # Arming only turns exploration on when it was disarmed.
        self.explore_pub.publish(Bool(data=True))
        say(f"{semantic} not in memory: exploring until it is found")
        start = time.monotonic()
        self.frontiers_seen_at = start
        known = {m[1] for m in self.memory}
        while True:
            self._check()
            found = self.in_memory(label)
            if found:
                say(f"FOUND {semantic} at ({found[0][2]:.2f}, {found[0][3]:.2f}) "
                    f"after {time.monotonic() - start:.0f} s")
                return True
            now = time.monotonic()
            if now - start > FIND_TIMEOUT_S:
                say(f"{semantic} not found after {FIND_TIMEOUT_S:.0f} s")
                return False
            if now - self.frontiers_seen_at > FIND_NO_FRONTIERS_S:
                say(f"{semantic} not found: nothing left to explore")
                return False
            self.mission["detail"] = f"exploring, {now - start:.0f} s"
            labels = {m[1] for m in self.memory}
            if labels - known:
                say(f"  seen so far: {', '.join(sorted(labels))}")
                known = labels
            time.sleep(0.05)

    def mission_go(self, say, text: str) -> bool:
        if self.command_pub.get_subscription_count() == 0:
            raise RuntimeError("coordinator is not listening on /user_command (is the stack up?)")
        self.arm(say, explore=False)
        try:
            seen_text = self.coord["text"]
            self.command_pub.publish(String(data=text))
            say(f"sent: {text}")
            end = time.monotonic() + GO_TIMEOUT_S
            while time.monotonic() < end:
                self._check()
                if self.coord["text"] != seen_text:
                    seen_text = self.coord["text"]
                    say(f"  [{self.coord['mode']}] {seen_text}")
                if self.coord["mode"] in ("TARGET_REACHED", "TARGET_FAILED"):
                    say(f"result: {self.coord['mode']}")
                    return self.coord["mode"] == "TARGET_REACHED"
                time.sleep(0.05)
            say(f"result: timed out after {GO_TIMEOUT_S:.0f} s")
            return False
        finally:
            self.disarm(say)

    def run_mission(self, name: str, say, func, *args) -> None:
        if not self.mission_lock.acquire(blocking=False):
            say(f"busy with '{self.mission['name']}' ({self.mission['detail']}); use `semnav stop`")
            return
        self.cancel.clear()
        self.mission.update({"name": name, "detail": ""})
        try:
            func(say, *args)
        except Cancelled:
            say("stopped" + (f" by watchdog: {self.watchdog_last['reason']}" if self.watchdog_last else ""))
        except Exception as exc:  # noqa: BLE001
            say(f"error: {exc}")
        finally:
            self._zero_velocity()
            if name in ("look", "find") and not self.coord["armed"]:
                self._set_bool(self.safety_cli, False, "safety gate", timeout=1.0)
            self.mission.update({"name": None, "detail": ""})
            self.mission_lock.release()

    # ── command dispatch (one thread per socket connection) ─────────────

    def handle(self, line: str, say) -> None:
        cmd, _, rest = line.strip().partition(" ")
        rest = rest.strip()
        if cmd == "status":
            say(json.dumps(self.health(), indent=1) if rest == "--json" else self.status_text())
        elif cmd in ("stop", "disarm"):
            self.cancel.set()
            self.disarm(say)
        elif cmd == "arm":
            self.run_mission("arm", say, lambda s: self.arm(s, explore=False))
        elif cmd == "explore":
            self.run_mission("explore", say, lambda s: self.arm(s, explore=True))
        elif cmd == "go" and rest:
            self.run_mission("go", say, self.mission_go, rest)
        elif cmd == "look" and rest:
            self.run_mission("look", say, self.mission_look, rest)
        elif cmd == "understand" and rest:
            try:
                semantic, go_text, how = self.understand(rest)
                say(f"'{rest}' -> {semantic} ({how}); would send: {go_text}")
            except RuntimeError as exc:
                say(f"error: {exc}")
        elif cmd == "find" and rest:

            def find(s):
                semantic, text, how = self.understand(rest)
                s(f"understood '{rest}' as: {semantic} ({how})")
                if self.in_memory(self.sem2det[semantic]):
                    s(f"{semantic} is already in memory")
                else:
                    self.check_searchable(semantic)
                    try:
                        if not self.mission_search(s, semantic):
                            return
                    finally:
                        # Ends exploration and cancels its goal before the semantic goal is sent.
                        self.disarm(s)
                self.mission["name"] = "find: go"
                self.mission_go(s, text)

            self.run_mission("find", say, find)
        else:
            say("commands: status [--json] | arm | explore | disarm | stop | go <text> | look <name> | find <sentence> | understand <sentence>")


def serve(ctrl: SemnavControl) -> None:
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, 0o600)
    server.listen(8)
    ctrl.log.info(f"listening on {SOCKET_PATH}")

    def client(conn):
        with conn:
            data = b""
            while not data.endswith(b"\n") and len(data) < 4096:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                data += chunk

            def say(text: str) -> None:
                ctrl.log.info(text)
                try:
                    conn.sendall((text + "\n").encode())
                except OSError:
                    pass  # client went away; the mission keeps running and still disarms when done

            ctrl.handle(data.decode(errors="replace"), say)

    while rclpy.ok():
        conn, _ = server.accept()
        threading.Thread(target=client, args=(conn,), daemon=True).start()


def main() -> None:
    rclpy.init()
    ctrl = SemnavControl()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ctrl.node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        serve(ctrl)
    finally:
        ctrl._zero_velocity()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
