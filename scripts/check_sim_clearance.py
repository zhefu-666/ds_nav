#!/usr/bin/env python3
"""Check ds_nav CAD-derived collision resources and linkage closure.

Reads the resources package directly from the source tree (or --share), so it
can run before a build. It validates the 501-pose linkage closure, the 14 CAD
convex hulls and the drawing-dimension profile height/width.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as E

import numpy as np
import trimesh

PARSER = argparse.ArgumentParser()
PARSER.add_argument(
    "--share", type=Path,
    default=Path(__file__).resolve().parents[1]
    / "src/simulation_integration/huananhu_sim_resources",
)
PARSER.add_argument(
    "--output", type=Path,
    default=Path(__file__).resolve().parents[1] / "artifacts/sim_tunnel/offline_validation.json",
)
ARGS = PARSER.parse_args()

share = ARGS.share
sys.path.insert(0, str(share / "scripts"))
from kinematic_math import forward_kinematics, joint_positions  # noqa: E402

cfg = json.loads((share / "config/model.json").read_text())
source = json.loads((share / "config/linkage_geometry.json").read_text())
profile = json.loads((share / "config/tunnel_profile.json").read_text())
support = {k: np.asarray(v) for k, v in json.loads(
    (share / "config/cad_support_vertices.json").read_text()).items()}
zero = forward_kinematics(cfg, joint_positions(cfg, 0, 0, 0, 0))
cad_to_model = zero["cad_base"]
points = {k: np.asarray(v) for k, v in source["axis_line_reference_points"].items()}


def point_on_link(world, link, cad_point):
    return (world[link] @ np.linalg.inv(zero[link]) @ cad_to_model
            @ np.r_[cad_point, 1])[:3]


limits = {
    j.get("name"): (float(j.find("limit").get("lower")),
                    float(j.find("limit").get("upper")))
    for j in E.parse(share / "urdf/huananhu.urdf").getroot().findall("joint")
    if j.find("limit") is not None and j.find("limit").get("lower") is not None
}
max_loop = max_gas = max_level = 0.0
rng = np.random.default_rng(20261029)
for i in range(501):
    fold = math.radians(81 * i / 500)
    q = joint_positions(
        cfg, fold, float(rng.uniform(-math.pi, math.pi)),
        float(rng.uniform(-math.pi / 2, math.pi / 2)),
        float(rng.uniform(math.radians(-20), math.radians(36)))
    )
    world = forward_kinematics(cfg, q)
    normal = world["yaw2"][:3, :3] @ np.asarray(cfg["mechanism"]["axis"])
    projection = np.eye(3) - np.outer(normal, normal)
    for key, l1, l2 in (("D", "pitch1_coupler", "pitch1_rocker"),
                        ("F", "uwb_platform", "pitch1_rocker")):
        max_loop = max(max_loop, float(np.linalg.norm(
            projection @ (point_on_link(world, l1, points[key])
                          - point_on_link(world, l2, points[key])))))
    max_level = max(max_level, float(np.linalg.norm(
        world["uwb_platform"][:3, :3] - world["yaw2"][:3, :3])))
    for spring in cfg["gas_springs"]:
        max_gas = max(max_gas, float(np.linalg.norm(
            point_on_link(world, spring["name"] + "_rod", np.asarray(spring["moving"]))
            - point_on_link(world, "pitch1_rocker", np.asarray(spring["moving"])))))
    for name, value in q.items():
        if name in limits:
            assert limits[name][0] - 1e-8 <= value <= limits[name][1] + 1e-8, (name, value)
assert max_loop < 1e-8 and max_gas < 1e-8 and max_level < 1e-10

hulls = {}
for link, bounds in support.items():
    if link.startswith("wheel_"):
        continue
    path = share / "models/huananhu_demo/collision" / f"{link}_hull.obj"
    text = path.read_text()
    assert sum(line.startswith("vn ") for line in text.splitlines()) == sum(
        line.startswith("v ") for line in text.splitlines()
    ), "Gazebo requires per-vertex normals: " + str(path)
    mesh = trimesh.load(path, force="mesh")
    assert mesh.is_convex and mesh.is_watertight, link
    bound_error = float(np.max(abs(mesh.bounds - np.array([bounds.min(0), bounds.max(0)]))));
    assert bound_error < 1e-9
    hulls[link] = {"vertices": len(mesh.vertices), "faces": len(mesh.faces),
                   "convex": True, "watertight": True,
                   "local_bound_error_m": bound_error}

q = joint_positions(cfg, profile["fold_rad"], profile["yaw1_rad"],
                    profile["yaw2_rad"], profile["aim_rad"])
fk = forward_kinematics(cfg, q)
cloud = np.concatenate([p @ fk[k][:3, :3].T + fk[k][:3, 3] for k, p in support.items()])
height = float(np.ptp(cloud[:, 2]))
width = float(np.ptp(cloud[:, 1]))
assert height < 0.25 and width < 0.8
assert len(hulls) == 14

AH = np.array(source["projected_points"]["H"]) - source["projected_points"]["A"]
out = {
    "passed": True,
    "resources": str(share),
    "sampled_linkage_poses": 501,
    "max_projected_linkage_closure_error_m": max_loop,
    "max_gas_spring_endpoint_error_m": max_gas,
    "max_uwb_orientation_error": max_level,
    "main_link_AH_m": float(np.linalg.norm(AH)),
    "main_link_capture_angle_to_ground_deg": math.degrees(
        math.asin(AH[1] / np.linalg.norm(AH))),
    "profile_height_m": height,
    "profile_width_y_m": width,
    "nominal_roof_margin_m": .25 - height,
    "nominal_total_side_margin_m": .8 - width,
    "hulls": hulls,
    "self_collision_and_hardware_calibration_validated": False,
}
ARGS.output.parent.mkdir(parents=True, exist_ok=True)
ARGS.output.write_text(json.dumps(out, indent=2) + "\n")
print(json.dumps({k: v for k, v in out.items() if k != "hulls"}, indent=2))
