#!/usr/bin/env python3
"""Generate visual-mesh GICP prior and folded-posture occupancy for RMUL 2027.

Default PCD samples the same STL triangles rendered by GPU lidar, plus explicit
floor visual surfaces. This is surface geometry, not ray visibility simulation.
The 2D map clips visual triangles to the robot's world-height band and projects
filled polygons, so raised platforms remain occupied rather than hollow outlines.
Legacy collision sampling remains available as --geometry collision for diagnosis.
"""

import argparse
import hashlib
import json
import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np

GENERATOR_VERSION = "2.0.0"


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _pose_matrix(el):
    """<pose>x y z roll pitch yaw</pose> -> 4x4, identity when missing."""
    t = np.eye(4)
    if el is None:
        return t
    vals = [float(v) for v in (el.text or "").split()]
    if len(vals) != 6:
        raise ValueError("pose needs 6 values, got %r" % (el.text,))
    t[:3, :3] = _rpy_to_matrix(*vals[3:])
    t[:3, 3] = vals[:3]
    return t


def _box_triples(xml_text, model_name=None):
    """Return [(pose_matrix, size[3], name)] for every collision box in an SDF text.

    The model pose is folded in, so a world file yields boxes already in world
    coordinates. `model_name` restricts the scan to a single model.
    """
    root = ET.fromstring(xml_text)
    models = list(root.iter("model"))
    if model_name is not None:
        models = [m for m in models if m.get("name") == model_name]
        if not models:
            raise ValueError("model %r not found" % model_name)
    if not models:
        models = [root]

    out = []
    for model in models:
        model_tf = _pose_matrix(model.find("pose"))
        for link in model.findall("link"):
            link_pose = _pose_matrix(link.find("pose"))
            for col in link.findall("collision"):
                box = col.find("geometry/box/size")
                if box is None:
                    continue
                size = np.array([float(v) for v in box.text.split()])
                out.append(
                    (
                        model_tf @ link_pose @ _pose_matrix(col.find("pose")),
                        size,
                        col.get("name") or link.get("name") or "box",
                    )
                )
    return out


def _model_pose_from_world(world_text, model_name):
    root = ET.fromstring(world_text)
    for model in root.iter("model"):
        if model.get("name") != model_name:
            continue
        inc = model.find("include/uri")
        uri = (inc.text or "").strip() if inc is not None else ""
        return _pose_matrix(model.find("pose")), uri
    raise ValueError("model %r not found in world" % model_name)


def _sample_box(tf, size, spacing):
    """Sample the 6 faces of a box at `spacing`, in the box's parent frame."""
    half = size / 2.0
    axes = []
    for axis in range(3):
        n = max(2, int(math.ceil(size[axis] / spacing)) + 1)
        axes.append(np.linspace(-half[axis], half[axis], n))
    gx, gy, gz = np.meshgrid(axes[0], axes[1], axes[2], indexing="ij")
    grid = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
    # keep a point only when it lies on at least one face plane
    tol = 1e-6
    on_face = (
        (np.abs(np.abs(grid[:, 0]) - half[0]) < tol)
        | (np.abs(np.abs(grid[:, 1]) - half[1]) < tol)
        | (np.abs(np.abs(grid[:, 2]) - half[2]) < tol)
    )
    pts = grid[on_face]
    return pts @ tf[:3, :3].T + tf[:3, 3]


def _sample_plane(tf, size, spacing, z_offset):
    """Sample the horizontal plane (top/bottom face) of a box."""
    nx = max(2, int(math.ceil(size[0] / spacing)) + 1)
    ny = max(2, int(math.ceil(size[1] / spacing)) + 1)
    xs = np.linspace(-size[0] / 2.0, size[0] / 2.0, nx)
    ys = np.linspace(-size[1] / 2.0, size[1] / 2.0, ny)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    grid = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z_offset)], axis=1)
    return grid @ tf[:3, :3].T + tf[:3, 3]


def _write_pcd_binary(path, pts):
    pts = np.asarray(pts, dtype=np.float32)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n" % (len(pts), len(pts))
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes(order="C"))


def _world_box_corners(tf, size):
    """The 8 world-frame corners of an oriented box."""
    half = size / 2.0
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    local = signs * half
    return local @ tf[:3, :3].T + tf[:3, 3]


def _write_pgm(path, occupied, resolution, origin_x, origin_y):
    """Write a P5 trinary map: 0 = occupied, 254 = free (P4/P5 convention)."""
    height, width = occupied.shape
    img = np.where(occupied, 0, 254).astype(np.uint8)
    xs = origin_x + (np.arange(width) + .5) * resolution
    ys = origin_y + (np.arange(height) + .5) * resolution
    known = (xs[None, :] >= -6.) & (xs[None, :] <= 6.) & (ys[:, None] >= -3.) & (ys[:, None] <= 5.)
    img[~known & ~occupied] = 205
    header = "P5\n%d %d\n255\n" % (width, height)
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        # image rows are top-down; world y grows upwards, so flip
        f.write(np.flipud(img).tobytes(order="C"))


def _project_occupancy(boxes_world, z_band, resolution, origin_x, origin_y, width, height):
    """Rasterise box footprints whose z range intersects `z_band`.

    Footprints (not surfaces) are used so a 0.3 m wall becomes a filled band and
    the planner cannot slip through a single-cell line.
    """
    occupied = np.zeros((height, width), dtype=bool)
    used = []
    for tf, size, name in boxes_world:
        corners = _world_box_corners(tf, size)
        zmin, zmax = corners[:, 2].min(), corners[:, 2].max()
        if zmax < z_band[0] or zmin > z_band[1]:
            used.append({"name": name, "z": [round(float(zmin), 4), round(float(zmax), 4)],
                         "projected": False})
            continue
        # oriented boxes in this model are axis-aligned in world (only 180° x flip)
        xmin, xmax = corners[:, 0].min(), corners[:, 0].max()
        ymin, ymax = corners[:, 1].min(), corners[:, 1].max()
        c0 = int(np.floor((xmin - origin_x) / resolution))
        c1 = int(np.ceil((xmax - origin_x) / resolution))
        r0 = int(np.floor((ymin - origin_y) / resolution))
        r1 = int(np.ceil((ymax - origin_y) / resolution))
        c0, c1 = max(c0, 0), min(c1, width)
        r0, r1 = max(r0, 0), min(r1, height)
        if c1 > c0 and r1 > r0:
            # rows are y-up in this array; _write_pgm flips for the file
            occupied[r0:r1, c0:c1] = True
        used.append({"name": name, "z": [round(float(zmin), 4), round(float(zmax), 4)],
                     "projected": True})
    return occupied, used


def _read_stl_triangles(path):
    """Read binary or ASCII STL without a new runtime dependency."""
    raw = open(path, "rb").read()
    count = struct.unpack_from("<I", raw, 80)[0] if len(raw) >= 84 else 0
    if len(raw) == 84 + 50 * count:
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                          ("attribute", "<u2")])
        triangles = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"].astype(float)
    else:
        vertices = []
        for line in raw.decode("ascii").splitlines():
            values = line.strip().split()
            if values and values[0] == "vertex":
                vertices.append([float(x) for x in values[1:]])
        triangles = np.asarray(vertices, dtype=float).reshape(-1, 3, 3)
    if not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError("invalid STL triangles: " + str(path))
    return triangles


def _visual_triangles(model_text, model_world_tf, model_dir):
    triangles, sources = [], []
    root = ET.fromstring(model_text)
    for model in root.findall("model"):
        for link in model.findall("link"):
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                uri = mesh.findtext("uri", "")
                if not uri.startswith("model://"):
                    raise ValueError("unsupported mesh URI " + uri)
                path = os.path.join(model_dir, uri[len("model://"):])
                scale = np.asarray([float(x) for x in mesh.findtext("scale", "1 1 1").split()])
                tf = model_world_tf @ _pose_matrix(model.find("pose")) @ _pose_matrix(link.find("pose")) @ _pose_matrix(visual.find("pose"))
                tri = _read_stl_triangles(path) * scale
                triangles.append(tri @ tf[:3, :3].T + tf[:3, 3])
                sources.append({"uri": uri, "sha256": _sha256(path), "triangles": len(tri)})
    if not triangles:
        raise ValueError("no visual STL mesh found")
    return np.concatenate(triangles), sources


def _sample_triangles(triangles, spacing):
    """Deterministic barycentric surface sampling, maximum edge interval spacing."""
    chunks = []
    for tri in triangles:
        edge = max(np.linalg.norm(tri[i] - tri[j]) for i, j in ((0, 1), (1, 2), (2, 0)))
        n = max(1, int(math.ceil(edge / spacing)))
        for i in range(n + 1):
            j = np.arange(n + 1 - i)
            chunks.append(tri[0] + (i / n) * (tri[1] - tri[0]) +
                          (j[:, None] / n) * (tri[2] - tri[0]))
    points = np.concatenate(chunks)
    # Stable voxel deduplication limits coplanar triangle and floor duplicates.
    _, indices = np.unique(np.floor(points / (spacing * .5)).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(indices)]


def _clip_z(polygon, height, lower):
    result = []
    if not len(polygon):
        return result
    for a, b in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        ia = a[2] >= height if lower else a[2] <= height
        ib = b[2] >= height if lower else b[2] <= height
        if ia:
            result.append(a)
        if ia != ib:
            result.append(a + (height - a[2]) / (b[2] - a[2]) * (b - a))
    return result


def _project_visual_occupancy(triangles, z_band, resolution, origin_x, origin_y, width, height):
    """Conservative convex polygon/cell overlap via separating axes; fills tops."""
    occupied = np.zeros((height, width), dtype=bool)
    projected = 0
    for tri in triangles:
        polygon = _clip_z(list(tri), z_band[0], True)
        polygon = _clip_z(polygon, z_band[1], False)
        if not polygon:
            continue
        xy = np.asarray(polygon)[:, :2]
        lo = np.floor((xy.min(0) - [origin_x, origin_y]) / resolution).astype(int)
        hi = np.floor((xy.max(0) - [origin_x, origin_y]) / resolution).astype(int)
        x0, y0 = max(0, lo[0]), max(0, lo[1])
        x1, y1 = min(width - 1, hi[0]), min(height - 1, hi[1])
        if x1 < x0 or y1 < y0:
            continue
        xx, yy = np.meshgrid(origin_x + (np.arange(x0, x1 + 1) + .5) * resolution,
                             origin_y + (np.arange(y0, y1 + 1) + .5) * resolution)
        centers = np.stack([xx, yy], axis=-1)
        hit = np.ones(xx.shape, dtype=bool)
        for a, b in zip(xy, np.roll(xy, -1, axis=0)):
            normal = np.array([-(b - a)[1], (b - a)[0]])
            if np.linalg.norm(normal) < 1e-12:
                continue
            projection = xy @ normal
            radius = resolution * .5 * np.abs(normal).sum()
            cell = centers @ normal
            hit &= (cell + radius >= projection.min() - 1e-10) & (cell - radius <= projection.max() + 1e-10)
        occupied[y0:y1 + 1, x0:x1 + 1] |= hit
        projected += 1
    return occupied, [{"visual_triangles_projected": projected}]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    default_res = os.path.abspath(os.path.join(pkg, "..", "huananhu_sim_resources"))

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--world", default=os.path.join(default_res, "worlds", "rmul_2027_world.sdf")
    )
    ap.add_argument("--model-dir", default=os.path.join(default_res, "models"))
    ap.add_argument("--model-name", default="rmul_2027")
    ap.add_argument("--out", default=os.path.join(pkg, "maps", "prior_map.pcd"))
    ap.add_argument("--geometry", choices=["visual", "collision"], default="visual")
    ap.add_argument("--spacing", type=float, default=0.05, help="structure sampling step [m]")
    ap.add_argument("--floor-spacing", type=float, default=0.10, help="floor sampling step [m]")
    ap.add_argument(
        "--no-interior-filter",
        action="store_true",
        help="keep outer wall faces and the floor strip hidden under the walls",
    )
    ap.add_argument("--pgm-out", default=os.path.join(pkg, "maps", "rmul_2027.pgm"))
    ap.add_argument(
        "--pgm-z-min",
        type=float,
        default=0.085,
        help="lower edge (exclude audited floor panels at 0.080..0.083m with 2mm margin) of the posture band projected into the 2D map [m]",
    )
    ap.add_argument(
        "--pgm-z-max",
        type=float,
        default=0.327,
        help="folded CAD height .24447 + floor .0805 + clearance; excludes .330m tunnel roof",
    )
    ap.add_argument("--pgm-resolution", type=float, default=0.01)
    ap.add_argument("--pgm-origin", type=float, nargs=2, default=[-6.55, -3.55])
    ap.add_argument("--pgm-size", type=int, nargs=2, default=[1310, 910], metavar=("W", "H"))
    args = ap.parse_args()
    if min(args.spacing, args.floor_spacing, args.pgm_resolution) <= 0:
        ap.error("sampling and grid spacing must be positive")
    if args.pgm_z_max <= args.pgm_z_min:
        ap.error("posture band requires z-max > z-min")

    world_text = open(args.world, encoding="utf-8").read()
    model_world_tf, model_uri = _model_pose_from_world(world_text, args.model_name)
    model_path = os.path.join(args.model_dir, args.model_name, "model.sdf")
    model_text = open(model_path, encoding="utf-8").read()

    boxes = _box_triples(model_text)
    floor_boxes = _box_triples(world_text)

    triangles, visual_sources = _visual_triangles(model_text, model_world_tf, args.model_dir)
    chunks = []
    manifest_boxes = []
    boxes_world = []
    for tf, size, name in boxes:
        world_tf = model_world_tf @ tf
        boxes_world.append((world_tf, size, name))
        pts = _sample_box(world_tf, size, args.spacing)
        if args.geometry == "collision":
            chunks.append(pts)
        manifest_boxes.append(
            {
                "name": name,
                "world_center": [round(float(v), 4) for v in world_tf[:3, 3]],
                "size": [round(float(v), 4) for v in size],
                "samples": int(len(pts)),
            }
        )

    if args.geometry == "visual":
        chunks.append(_sample_triangles(triangles, args.spacing))

    for tf, size, name in floor_boxes:
        pts = _sample_plane(tf, size, args.floor_spacing, z_offset=size[2] / 2.0)
        chunks.append(pts)
        manifest_boxes.append(
            {
                "name": "floor:%s" % name,
                "world_top_z": round(float(pts[:, 2].max()), 4),
                "size": [round(float(v), 4) for v in size],
                "samples": int(len(pts)),
            }
        )

    pts = np.concatenate(chunks, axis=0)
    bounds = None
    if not args.no_interior_filter:
        eps = 0.02
        keep = (
            (pts[:, 0] >= -6.0 - eps)
            & (pts[:, 0] <= 6.0 + eps)
            & (pts[:, 1] >= -3.0 - eps)
            & (pts[:, 1] <= 5.0 + eps)
            & (pts[:, 2] >= 0.06)
            & (pts[:, 2] <= 0.60)
        )
        bounds = {
            "x": [-6.0 - eps, 6.0 + eps],
            "y": [-3.0 - eps, 5.0 + eps],
            "z": [0.06, 0.60],
            "note": "interior faces only: outer wall faces and floor under the walls dropped",
        }
        pts = pts[keep]

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    _write_pcd_binary(args.out, pts)

    width, height = args.pgm_size
    project = _project_visual_occupancy if args.geometry == "visual" else _project_occupancy
    occupied, pgm_boxes = project(
        triangles if args.geometry == "visual" else boxes_world,
        (args.pgm_z_min, args.pgm_z_max),
        args.pgm_resolution,
        args.pgm_origin[0],
        args.pgm_origin[1],
        width,
        height,
    )
    if args.geometry == "visual":
        # Physics boxes represent solid volumes even if their top lies above the
        # posture slab; union prevents hollow wall/platform interiors.
        solid, solid_details = _project_occupancy(
            boxes_world, (args.pgm_z_min, args.pgm_z_max), args.pgm_resolution,
            args.pgm_origin[0], args.pgm_origin[1], width, height)
        occupied |= solid
        pgm_boxes.extend(solid_details)
    os.makedirs(os.path.dirname(os.path.abspath(args.pgm_out)), exist_ok=True)
    _write_pgm(args.pgm_out, occupied, args.pgm_resolution, args.pgm_origin[0], args.pgm_origin[1])

    manifest = {
        "generator": os.path.basename(__file__),
        "generator_version": GENERATOR_VERSION,
        "frame": "map",
        "frame_note": "identical to the Gazebo world frame; no static map->world offset exists",
        "units": "metres (m), radians",
        "representation": args.geometry + "_surface_prior; visibility is not simulated",
        "visual_sources": visual_sources,
        "model_world_transform": [
            [round(float(v), 6) for v in row] for row in model_world_tf
        ],
        "sources": {
            "world": {"path": os.path.relpath(args.world, pkg), "sha256": _sha256(args.world)},
            "model_sdf": {
                "path": os.path.relpath(model_path, pkg),
                "sha256": _sha256(model_path),
                "uri": model_uri,
            },
        },
        "sampling": {
            "structure_spacing_m": args.spacing,
            "floor_spacing_m": args.floor_spacing,
            "interior_filter": bounds,
        },
        "geometry": manifest_boxes,
        "result": {
            "file": os.path.basename(args.out),
            "sha256": _sha256(args.out),
            "points": int(len(pts)),
            "bbox_min": [round(float(v), 4) for v in pts.min(axis=0)],
            "bbox_max": [round(float(v), 4) for v in pts.max(axis=0)],
        },
        "occupancy_grid": {
            "file": os.path.basename(args.pgm_out),
            "sha256": _sha256(args.pgm_out),
            "yaml": {
                "image": os.path.basename(args.pgm_out),
                "resolution": args.pgm_resolution,
                "origin": [args.pgm_origin[0], args.pgm_origin[1], 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            "size": {"width": width, "height": height},
            "posture_band_z": [args.pgm_z_min, args.pgm_z_max],
            "band_note": "folded posture: visual triangles clipped to world-height slab and projected as filled polygons, union solid collision footprints to avoid hollow volumes; .280m platforms occupied, .330m roof excluded; posture gate still required",
            "occupied_cells": int(occupied.sum()),
            "boxes": pgm_boxes,
        },
    }
    import yaml
    with open(os.path.splitext(args.pgm_out)[0] + ".yaml", "w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest["occupancy_grid"]["yaml"], stream, sort_keys=False)

    manifest_path = os.path.splitext(args.out)[0] + "_manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
        f.write("\n")

    print("wrote %s (%d points)" % (args.out, len(pts)))
    print("wrote %s" % manifest_path)


if __name__ == "__main__":
    main()
