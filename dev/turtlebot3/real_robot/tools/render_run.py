#!/usr/bin/env python3
"""Polished video of one recorded `semnav find` run.

    python3 render_polished.py <bag_dir> <journal.txt> <t_command> <t_end> <out.avi>

Camera with detections fills the frame, a minimap card sits over it, and the right column shows
the robot's reasoning as it happens, taken from the control and exploration logs. Title and stats
cards open and close it.
"""

import bisect
import math
import re
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

bag_dir, journal_path, T_CMD, T_END, OUT = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
T_START = T_CMD - 2.0
TARGET = "trash can"
COMMAND = "find the trash can"

W, H, FPS = 1280, 720, 20
CAM_W = 960
COL_X = CAM_W
FONT_DIR = "/usr/share/fonts/truetype/dejavu/"


def font(size, bold=False):
    return ImageFont.truetype(FONT_DIR + ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"), size)


F_TITLE, F_SUB, F_H, F_BODY, F_SMALL, F_TINY = font(54, True), font(24), font(15, True), font(17), font(14), font(12)
F_MONO = ImageFont.truetype(FONT_DIR + "DejaVuSansMono.ttf", 14)

BG = (16, 18, 22)
PANEL = (24, 27, 33)
TEXT = (232, 236, 242)
MUTED = (140, 148, 160)
ACCENT = (90, 200, 255)
ORANGE = (255, 150, 40)
GREEN = (80, 220, 140)
YELLOW = (255, 210, 70)
RED = (255, 90, 90)
LABEL_COLOURS = {"trash can": RED, "door": (110, 160, 255), "cabinet": (200, 120, 255), "couch": GREEN,
                 "chair": (90, 210, 230), "backpack": (255, 120, 200), "table": (230, 200, 120)}

# ── recording ────────────────────────────────────────────────────────────────

TOPICS = {"/semantic_map/live_debug_image/compressed", "/map", "/tf", "/tf_static", "/semantic_map/state"}
reader = SequentialReader()
reader.open(StorageOptions(uri=bag_dir, storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
images, maps, mems, tf_static, tf_dyn = [], [], [], {}, {}
while reader.has_next():
    topic, raw, stamp = reader.read_next()
    if topic not in types:
        continue
    t = stamp * 1e-9
    if topic in ("/tf", "/tf_static"):
        for tr in deserialize_message(raw, types[topic]).transforms:
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            entry = (tr.header.frame_id, tr.transform.translation.x, tr.transform.translation.y, yaw)
            if topic == "/tf_static":
                tf_static[tr.child_frame_id] = entry
            elif tr.child_frame_id in ("odom", "base_footprint", "base_link"):
                tf_dyn.setdefault(tr.child_frame_id, []).append((t, entry))
        continue
    if t < T_START - 30 or t > T_END + 1:
        continue
    msg = deserialize_message(raw, types[topic])
    if topic.endswith("compressed"):
        images.append((t, bytes(msg.data)))
    elif topic == "/map":
        maps.append((t, msg))
    else:
        mems.append((t, [(e.detector_label, e.pose.position.x, e.pose.position.y) for e in msg.entities]))
for v in tf_dyn.values():
    v.sort(key=lambda e: e[0])
print(f"images {len(images)} maps {len(maps)} memory {len(mems)}", flush=True)


def latest(seq, t):
    if not seq:
        return None
    i = bisect.bisect_right([s[0] for s in seq], t) - 1
    return seq[i][1] if i >= 0 else None


def robot_pose(t):
    frame = "base_footprint" if "base_footprint" in tf_dyn else "base_link"
    chain = []
    while frame != "map":
        e = latest(tf_dyn[frame], t) if frame in tf_dyn else tf_static.get(frame)
        if e is None:
            return None
        chain.append(e[1:])
        frame = e[0]
    x = y = yaw = 0.0
    for bx, by, byaw in reversed(chain):
        x, y, yaw = x + math.cos(yaw) * bx - math.sin(yaw) * by, y + math.sin(yaw) * bx + math.cos(yaw) * by, yaw + byaw
    return x, y, yaw


# ── reasoning, from the logs ─────────────────────────────────────────────────

STAMP = re.compile(r"\[(\d{10}\.\d+)\]")
feed = []  # (t, text, colour)
seen_labels = set()
goal_points = []  # (t, x, y)
target_xy = None
for line in open(journal_path, errors="replace"):
    m = STAMP.search(line)
    if not m:
        continue
    t = float(m.group(1))
    if t < T_CMD - 1 or t > T_END:
        continue
    if "understood" in line:
        feed.append((t, f'Heard: "{COMMAND}"', TEXT))
    elif "not in memory: exploring" in line:
        feed.append((t, "Not in memory yet, so explore until the detector sees one", MUTED))
    elif "[goal] Selected" in line:
        g = re.search(r"\(([-\d.]+), ([-\d.]+)\) dist=([\d.]+) m gain=(\d+)", line)
        if g:
            x, y, d, gain = float(g.group(1)), float(g.group(2)), float(g.group(3)), int(g.group(4))
            goal_points.append((t, x, y))
            feed.append((t, f"Head for an opening {d:.1f} m away, about {gain * 0.0025:.0f} m² unseen beyond it", ACCENT))
    elif "[result] ABORTED" in line or "Goal stalled" in line:
        feed.append((t, "Can't get there, try another opening", ORANGE))
    elif "seen so far:" in line:
        labels = [s.strip() for s in line.split("seen so far:")[1].split(",")]
        for label in labels:
            if label and label not in seen_labels:
                seen_labels.add(label)
                feed.append((t, f"Spotted a {label}, added to memory", LABEL_COLOURS.get(label, TEXT)))
    elif "FOUND" in line:
        g = re.search(r"at \(([-\d.]+), ([-\d.]+)\) after (\d+) s", line)
        if g:
            target_xy = (float(g.group(1)), float(g.group(2)))
            feed.append((t, f"Found the {TARGET} after {g.group(3)} s", RED))
            feed.append((t + 0.05, "Stop exploring, plan a path to it", MUTED))
    elif "Nav2 goal accepted" in line:
        feed.append((t, f"Path planned, driving to the {TARGET}", YELLOW))
    elif "result: TARGET_REACHED" in line:
        feed.append((t, f"Arrived at the {TARGET}", GREEN))
feed.sort(key=lambda e: e[0])
T_FOUND = next((t for t, text, _ in feed if text.startswith("Found")), None)
T_ARRIVED = next((t for t, text, _ in feed if text.startswith("Arrived")), T_END)
print(f"{len(feed)} reasoning entries, found at {T_FOUND}, arrived at {T_ARRIVED}", flush=True)

# ── minimap ──────────────────────────────────────────────────────────────────

MM_W, MM_H = 340, 260
MM_X, MM_Y = 20, H - MM_H - 20
final = latest(maps, T_END)
grid = np.array(final.data, dtype=np.int16).reshape(final.info.height, final.info.width)
ys, xs = np.where(grid >= 0)
res = final.info.resolution
fox, foy = final.info.origin.position.x, final.info.origin.position.y
pad = 12
wx0, wx1 = fox + (xs.min() - pad) * res, fox + (xs.max() + pad) * res
wy0, wy1 = foy + (ys.min() - pad) * res, foy + (ys.max() + pad) * res
INNER_W, INNER_H = MM_W - 20, MM_H - 44
scale = min(INNER_W / (wx1 - wx0), INNER_H / (wy1 - wy0))
mm_off_x = 10 + (INNER_W - (wx1 - wx0) * scale) / 2
mm_off_y = 34 + (INNER_H - (wy1 - wy0) * scale) / 2


def mm(x, y):
    return (mm_off_x + (x - wx0) * scale, mm_off_y + (wy1 - y) * scale)


map_cache = {}


def map_layer(msg):
    key = id(msg)
    if key in map_cache:
        return map_cache[key]
    g = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    rgba = np.zeros(g.shape + (4,), np.uint8)
    rgba[g == 0] = (62, 70, 84, 255)
    rgba[g >= 50] = (225, 230, 238, 255)
    tile = Image.fromarray(np.flipud(rgba), "RGBA")
    ox, oy, r = msg.info.origin.position.x, msg.info.origin.position.y, msg.info.resolution
    x0, y0 = mm(ox, oy + msg.info.height * r)
    w, h = max(1, int(msg.info.width * r * scale)), max(1, int(msg.info.height * r * scale))
    layer = Image.new("RGBA", (MM_W, MM_H), (0, 0, 0, 0))
    layer.alpha_composite(tile.resize((w, h), Image.NEAREST), (int(round(x0)), int(round(y0))))
    map_cache.clear()
    map_cache[key] = layer
    return layer


def draw_minimap(t, trail):
    card = Image.new("RGBA", (MM_W, MM_H), (0, 0, 0, 0))
    d = ImageDraw.Draw(card)
    d.rounded_rectangle((0, 0, MM_W - 1, MM_H - 1), 14, fill=(14, 16, 20, 225), outline=(70, 78, 92, 255))
    d.text((12, 9), "MAP", font=F_H, fill=MUTED)
    msg = latest(maps, t)
    if msg is not None:
        card.alpha_composite(map_layer(msg))
    d = ImageDraw.Draw(card)
    if len(trail) > 1:
        pts = [mm(x, y) for x, y in trail]
        d.line(pts, fill=(80, 200, 255, 255), width=3, joint="curve")
    goal = None
    for gt, gx, gy in goal_points:
        if gt <= t:
            goal = (gx, gy)
    if goal is not None and (T_FOUND is None or t < T_FOUND):
        gx, gy = mm(*goal)
        d.ellipse((gx - 7, gy - 7, gx + 7, gy + 7), outline=YELLOW + (255,), width=2)
    for label, x, y in latest(mems, t) or []:
        px, py = mm(x, y)
        c = LABEL_COLOURS.get(label, (180, 180, 180))
        r = 5 if label != TARGET else 6
        d.ellipse((px - r, py - r, px + r, py + r), fill=c + (255,))
    if target_xy is not None and T_FOUND is not None and t >= T_FOUND:
        px, py = mm(*target_xy)
        pulse = 9 + 4 * (0.5 + 0.5 * math.sin((t - T_FOUND) * 6))
        d.ellipse((px - pulse, py - pulse, px + pulse, py + pulse), outline=RED + (255,), width=2)
    pose = robot_pose(t)
    if pose is not None:
        px, py = mm(pose[0], pose[1])
        a = -pose[2]
        tri = [(px + 11 * math.cos(a), py + 11 * math.sin(a)),
               (px + 7 * math.cos(a + 2.5), py + 7 * math.sin(a + 2.5)),
               (px + 7 * math.cos(a - 2.5), py + 7 * math.sin(a - 2.5))]
        d.polygon(tri, fill=ORANGE + (255,), outline=(20, 20, 20, 255))
    return card


# ── reasoning column ─────────────────────────────────────────────────────────


def wrap(text, fnt, width):
    words, lines, cur = text.split(), [], ""
    for w in words:
        trial = (cur + " " + w).strip()
        if fnt.getlength(trial) <= width:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    return lines + [cur] if cur else lines


def draw_column(img, t):
    d = ImageDraw.Draw(img)
    d.rectangle((COL_X, 0, W, H), fill=PANEL)
    d.text((COL_X + 22, 22), "ROBOT REASONING", font=F_H, fill=MUTED)
    d.line((COL_X + 22, 48, W - 22, 48), fill=(52, 58, 68), width=1)
    visible = [e for e in feed if e[0] <= t]
    blocks = []
    for et, text, colour in visible:
        lines = wrap(text, F_BODY, W - COL_X - 70)
        blocks.append((et, lines, colour, 24 + 22 * len(lines)))
    y = H - 24
    placed = []
    for block in reversed(blocks):
        y -= block[3]
        if y < 62:
            break
        placed.append((y, block))
    for i, (by, (et, lines, colour, h)) in enumerate(placed):
        age = t - et
        alpha = min(1.0, age / 0.4)  # fade in
        fade_old = max(0.35, 1.0 - 0.12 * i)
        a = alpha * fade_old
        c = tuple(int(PANEL[k] + (colour[k] - PANEL[k]) * a) for k in range(3))
        m = tuple(int(PANEL[k] + (MUTED[k] - PANEL[k]) * a) for k in range(3))
        stamp = f"{int(et - T_CMD) // 60}:{int(et - T_CMD) % 60:02d}"
        d.text((COL_X + 22, by), stamp, font=F_MONO, fill=m)
        d.ellipse((COL_X + 64, by + 6, COL_X + 70, by + 12), fill=c)
        for k, line in enumerate(lines):
            d.text((COL_X + 80 - 2, by - 2 + 22 * k), line, font=F_BODY, fill=c)


def phase(t):
    if t >= T_ARRIVED:
        return "ARRIVED", GREEN
    if T_FOUND is not None and t >= T_FOUND:
        return "DRIVING TO TARGET", YELLOW
    if t >= T_CMD:
        return "EXPLORING", ACCENT
    return "READY", MUTED


def chip(d, xy, text, fnt, fg, bg):
    x, y = xy
    w = fnt.getlength(text)
    d.rounded_rectangle((x, y, x + w + 28, y + 36), 18, fill=bg)
    d.text((x + 14, y + 8), text, font=fnt, fill=fg)
    return x + w + 28


# ── cards ────────────────────────────────────────────────────────────────────


def title_card():
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((90, 230), "Semantic navigation", font=F_TITLE, fill=TEXT)
    d.text((92, 305), "TurtleBot3 · open-vocabulary detection · lidar SLAM · frontier exploration", font=F_SUB, fill=MUTED)
    d.rounded_rectangle((90, 380, 90 + F_SUB.getlength(f"$ semnav {COMMAND}") + 40, 432), 12, fill=PANEL)
    d.text((110, 390), f"$ semnav {COMMAND}", font=F_SUB, fill=ACCENT)
    return img


def stats_card(trail):
    dist = sum(math.dist(trail[i], trail[i + 1]) for i in range(len(trail) - 1))
    objects = len(latest(mems, T_END) or [])
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((90, 150), f"Arrived at the {TARGET}", font=F_TITLE, fill=TEXT)
    stats = [(f"{T_FOUND - T_CMD:.0f} s", "to find it"), (f"{T_ARRIVED - T_CMD:.0f} s", "command to arrival"),
             (f"{dist:.1f} m", "driven"), (f"{objects}", "objects mapped")]
    for i, (big, small) in enumerate(stats):
        x = 90 + i * 270
        d.rounded_rectangle((x, 280, x + 240, 420), 16, fill=PANEL)
        d.text((x + 24, 300), big, font=font(46, True), fill=ACCENT)
        d.text((x + 26, 370), small, font=F_SUB, fill=MUTED)
    d.text((92, 480), "Map built from scratch during the run; nothing was known in advance.", font=F_SUB, fill=MUTED)
    return img


# ── frames ───────────────────────────────────────────────────────────────────

writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))


def emit(img, n=1):
    bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    for _ in range(n):
        writer.write(bgr)


def crossfade(a, b, seconds):
    n = int(seconds * FPS)
    for k in range(n):
        emit(Image.blend(a, b, (k + 1) / n))


trail = []
first = None
last = None
n = int((T_END - T_START) * FPS)
for k in range(n):
    t = T_START + k / FPS
    img = Image.new("RGB", (W, H), BG)
    jpeg = latest(images, t)
    if jpeg is not None:
        cam = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        cam = cv2.resize(cam, (CAM_W, H), interpolation=cv2.INTER_CUBIC)
        img.paste(Image.fromarray(cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)), (0, 0))
    pose = robot_pose(t)
    if pose is not None and t >= T_CMD:
        if not trail or math.dist(trail[-1], pose[:2]) > 0.03:
            trail.append(pose[:2])
    img = img.convert("RGBA")
    # top gradient for legibility
    grad = Image.new("RGBA", (CAM_W, 120), (0, 0, 0, 0))
    gd = ImageDraw.Draw(grad)
    for yy in range(120):
        gd.line((0, yy, CAM_W, yy), fill=(0, 0, 0, int(150 * (1 - yy / 120))))
    img.alpha_composite(grad, (0, 0))
    img.alpha_composite(draw_minimap(t, trail), (MM_X, MM_Y))
    img = img.convert("RGB")
    d = ImageDraw.Draw(img)
    x = chip(d, (20, 18), f"▶  semnav {COMMAND}", F_BODY, TEXT, (30, 34, 42))
    label, colour = phase(t)
    chip(d, (x + 10, 18), label, F_H, (16, 18, 22), colour)
    elapsed = max(0.0, t - T_CMD)
    timer = f"{elapsed:5.1f} s"
    d.rounded_rectangle((CAM_W - 130, 18, CAM_W - 20, 54), 18, fill=(30, 34, 42))
    d.text((CAM_W - 118, 26), timer, font=F_MONO, fill=TEXT)
    draw_column(img, t)
    if first is None:
        first = img
        crossfade(title_card(), img, 0.8) if False else None
    last = img
    if k == 0:
        title = title_card()
        emit(title, int(2.5 * FPS))
        crossfade(title, img, 0.8)
    emit(img)
    if k % 200 == 0:
        print(f"frame {k}/{n}", flush=True)

emit(last, int(1.0 * FPS))
stats = stats_card(trail)
crossfade(last, stats, 0.8)
emit(stats, int(4 * FPS))
writer.release()
print("done", flush=True)
