#!/usr/bin/env python3
"""Bounded tunnel passage probe for the ds_nav physics-test gate.

Gazebo truth feedback is explicit diagnostics, not localization acceptance.
Commands go to /sim/manual_cmd_vel and the diagnostic gate is the only owner of
/sim/actuator_cmd_vel.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SHARE = ROOT / "src/simulation_integration/huananhu_sim_resources"

parser = argparse.ArgumentParser()
parser.add_argument("--speed", type=float, default=.25)
parser.add_argument("--escape", action="store_true",
                    help="Bounded recovery: permit translation while posture is obstructed")
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--posture", choices=["stand", "tunnel"], default="tunnel")
parser.add_argument("--target-x", type=float, default=-.7)
parser.add_argument("--roof-bottom", type=float)
parser.add_argument("--target-y", type=float, default=None,
                    help="default: tunnel profile lane body centre")
parser.add_argument("--timeout", type=float, default=100)
parser.add_argument("--share", type=Path, default=DEFAULT_SHARE)
args = parser.parse_args()

if args.output.exists():
    parser.error("Output already exists; use a new result path")
if not .02 <= args.speed <= .5:
    parser.error("speed must be in [.02, .5] m/s")
if args.timeout <= 6:
    parser.error("timeout must exceed 6 simulation seconds")

share = args.share
sys.path.insert(0, str(share / "scripts"))
from kinematic_math import forward_kinematics  # noqa: E402

cfg = json.loads((share / "config/model.json").read_text())
profile = json.loads((share / "config/tunnel_profile.json").read_text())
support = {k: np.asarray(v) for k, v in json.loads(
    (share / "config/cad_support_vertices.json").read_text()).items()}
if args.target_y is None:
    args.target_y = float(profile["lane_body_center_y_m"])

rclpy.init()
node = Node("sim_tunnel_probe")
data = {}
contacts = {}
events = []
last_contact_time = -1e9
gate_status = {"value": ""}

for key, topic, message_type in (
    ("truth", "/sim/ground_truth_odometry", Odometry),
    ("joints", "/joint_states", JointState),
    ("clock", "/clock", Clock),
):
    node.create_subscription(
        message_type, topic, lambda msg, k=key: data.update({k: msg}),
        qos_profile_sensor_data)


def on_contact(msg):
    global last_contact_time
    if msg.contacts and "clock" in data:
        last_contact_time = data["clock"].clock.sec + data["clock"].clock.nanosec * 1e-9
    for contact in msg.contacts:
        key = contact.collision1.name + " | " + contact.collision2.name
        contacts[key] = contacts.get(key, 0) + 1


node.create_subscription(Contacts, "/assessment/body_contacts", on_contact,
                         qos_profile_sensor_data)
node.create_subscription(String, "/sim/physics_test_status",
                         lambda msg: gate_status.update({"value": msg.data}), 10)
cmd = node.create_publisher(Twist, "/sim/manual_cmd_vel", 10)
mode = node.create_publisher(String, "/sim/control_mode", 10)
post = node.create_publisher(String, "/huananhu/posture", 10)
yaw_pub = node.create_publisher(Float64MultiArray, "/huananhu/yaw", 10)
aim_pub = node.create_publisher(Float64, "/huananhu/aim", 10)


def sim_time():
    return data["clock"].clock.sec + data["clock"].clock.nanosec * 1e-9


def snap(stage):
    msg = data["truth"]
    q = msg.pose.pose.orientation
    pose = msg.pose.pose.position
    joints = dict(zip(data["joints"].name, data["joints"].position))
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    # Truth is the chassis link; CAD mount is -0.038 m below it.
    base = np.array([pose.x, pose.y, pose.z]) + rotation @ np.array([0, 0, -.038])
    fk = forward_kinematics(cfg, joints)
    cloud = np.concatenate([
        (points @ fk[link][:3, :3].T + fk[link][:3, 3]) @ rotation.T + base
        for link, points in support.items()
    ])
    low = cloud.min(0)
    high = cloud.max(0)
    return dict(
        cad_min_world=low.tolist(),
        cad_max_world=high.tolist(),
        cad_height_m=float(high[2] - low[2]),
        roof_margin_m=None if args.roof_bottom is None else float(args.roof_bottom - high[2]),
        head_pitch=joints.get("pitch2_joint"),
        turret_yaw1=joints.get("yaw1_joint"),
        turret_yaw2=joints.get("yaw2_joint"),
        stage=stage,
        sim=sim_time(),
        xyz=[pose.x, pose.y, pose.z],
        yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        roll=roll,
        pitch=pitch,
        fold=joints.get("pitch1_drive_joint"),
        body_contacts=sum(contacts.values()),
        gate_status=gate_status["value"],
    )


def posture():
    post.publish(String(data=args.posture))
    yaw_pub.publish(Float64MultiArray(data=[0., 0.]))
    aim_pub.publish(Float64(data=profile["aim_rad"] if args.posture == "tunnel" else 0.))


end = time.monotonic() + 30
while len(data) < 3 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.05)
assert len(data) == 3
if args.escape:
    initial = snap("initial")["xyz"]
    assert math.dist(initial[:2], [args.target_x, args.target_y]) < 1.2, \
        "Recovery target must be within 1.2 m"

status = "not_started"
rows = []
mode.publish(String(data="manual"))
wall = time.monotonic()
start = sim_time()
last = -1
progress = []
lane_ready = False
iy = 0.
previous_xy = None
previous_sim = None
vy_filtered = 0.
previous_yaw = None
w_filtered = 0.
iw = 0.
try:
    while sim_time() - start < args.timeout and time.monotonic() - wall < 240:
        rclpy.spin_once(node, timeout_sec=.005)
        if sim_time() - last < .05:
            continue
        last = sim_time()
        elapsed = sim_time() - start
        state = snap("settle" if elapsed < 6 else "drive")
        rows.append(state)
        posture()
        motion = Twist()
        if abs(state["roll"]) > .15 or abs(state["pitch"]) > .15:
            status = "tilt_abort"
            break
        if elapsed >= 6:
            for key in ("truth", "joints"):
                stamp = data[key].header.stamp.sec + data[key].header.stamp.nanosec * 1e-9
                if sim_time() - stamp > 1:
                    raise RuntimeError("Stale " + key + " feedback")
            if contacts and not args.escape:
                status = "body_contact_stop"
                break
            if gate_status["value"].endswith("body_contact_stop"):
                status = "body_contact_stop"
                break
            target = 0 if args.posture == "tunnel" else math.radians(81)
            posture_ok = (
                abs(state["fold"] - target) > .01
                or abs(state["turret_yaw1"]) > .01
                or abs(state["turret_yaw2"]) > .01
                or (args.posture == "tunnel"
                    and abs(state["head_pitch"] - (profile["aim_rad"] - target)) > .01)
            )
            if posture_ok and not args.escape:
                status = "posture_blocked"
                break
            dx = args.target_x - state["xyz"][0]
            dy = args.target_y - state["xyz"][1]
            err = math.remainder(-state["yaw"], 2 * math.pi)
            if math.hypot(dx, dy) < .025 and abs(err) < .02:
                status = "reached"
                break
            step = .05 if previous_sim is None else max(.001, sim_time() - previous_sim)
            if previous_xy is not None:
                vy_filtered = .8 * vy_filtered + .2 * (state["xyz"][1] - previous_xy[1]) / step
            if previous_yaw is not None:
                w_filtered = (.8 * w_filtered
                              + .2 * math.remainder(state["yaw"] - previous_yaw, 2 * math.pi) / step)
            previous_yaw = state["yaw"]
            iw = max(-.1, min(.1, iw + err * step))
            previous_xy = state["xyz"][:2]
            previous_sim = sim_time()
            iy = max(-.04, min(.04, iy + dy * step))
            vx = max(-args.speed, min(args.speed, .8 * dx))
            vy = max(-min(args.speed, .08), min(min(args.speed, .08),
                                                2. * dy + .4 * iy - .2 * vy_filtered))
            # Align the lane before forward travel; avoid a cold diagonal start.
            if not lane_ready:
                if abs(dy) < .01 and abs(err) < .02:
                    lane_ready = True
                else:
                    vx = 0.
            # Once primed, keep lateral feedback active instead of alternating axes.
            motion.linear.x = math.cos(state["yaw"]) * vx + math.sin(state["yaw"]) * vy
            motion.linear.y = -math.sin(state["yaw"]) * vx + math.cos(state["yaw"]) * vy
            motion.angular.z = max(-.25, min(.25, 3. * err + .4 * iw - .2 * w_filtered))
            if args.escape and sim_time() - last_contact_time < .5:
                # Retreat along the dominant requested axis first. Tangential
                # corrections can point toward the contact and defeat the guard.
                if abs(dx) >= abs(dy):
                    vy = 0.
                else:
                    vx = 0.
                motion.linear.x = math.cos(state["yaw"]) * vx + math.sin(state["yaw"]) * vy
                motion.linear.y = -math.sin(state["yaw"]) * vx + math.cos(state["yaw"]) * vy
                motion.angular.z = 0.
            progress.append((sim_time(), state["xyz"][:2]))
            if len(progress) > 1:
                old = min(progress, key=lambda v: abs(v[0] - (sim_time() - 5)))
                if sim_time() - old[0] >= 4.9 and math.dist(state["xyz"][:2], old[1]) < .005:
                    status = "collision_or_stall"
                    break
        cmd.publish(motion)
    else:
        status = "timeout"
    events.append({"status": status, "sim": sim_time()})
    cmd.publish(Twist())
    stop = sim_time()
    while sim_time() - stop < 2:
        rclpy.spin_once(node, timeout_sec=.01)
        if sim_time() - last > .05:
            rows.append(snap("stopped"))
            cmd.publish(Twist())
            last = sim_time()
except Exception as error:
    status = "error"
    events.append({"error": str(error)})
    raise
except KeyboardInterrupt:
    status = "cancelled"
finally:
    if rclpy.ok():
        cmd.publish(Twist())
        mode.publish(String(data="disabled"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({
        "status": status,
        "command_speed_mps": args.speed,
        "posture": args.posture,
        "target": [args.target_x, args.target_y],
        "roof_bottom_world_m": args.roof_bottom,
        "profile": profile,
        "feedback": "Gazebo chassis truth and diagnostic gate only; not localization",
        "trace": rows,
        "body_contact_pairs": contacts,
        "events": events,
    }, indent=2))
    print(status, rows[-1] if rows else None, flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
