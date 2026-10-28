#!/usr/bin/env python3
"""Build a 2D occupancy grid (PGM+YAML) and 3D point clouds (PCD) from the
RMUL 2026 arena STL.

The mesh is transformed through the same chain Gazebo applies to it:

    world_frame = world_pose ∘ link_pose ∘ mesh_scale

so that the produced map lives in the same coordinates the robot's odometry
does at runtime.
"""

import argparse
from pathlib import Path

import numpy as np
import trimesh
import yaml


# From src/rmu_gazebo_simulator/.../models/rmul_2026/model.sdf
MESH_SCALE = 0.001                      # <scale>0.001 0.001 0.001</scale>
LINK_TRANSLATION = (5.0, 2.0, 0.0)      # <pose>5 2 0 0 0 3.14</pose>
LINK_YAW = 3.14

# From src/rmu_gazebo_simulator/.../worlds/rmul_2026_world.sdf
WORLD_TRANSLATION = (0.5, 2.0, -1.3)    # <pose>0.5 2.0 -1.3 1.57 0 0</pose>
WORLD_ROLL = 1.57


def apply_full_transform(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    mesh.apply_scale(MESH_SCALE)
    r_link = trimesh.transformations.euler_matrix(0.0, 0.0, LINK_YAW, axes="sxyz")
    t_link = trimesh.transformations.translation_matrix(LINK_TRANSLATION)
    mesh.apply_transform(t_link @ r_link)
    r_world = trimesh.transformations.euler_matrix(WORLD_ROLL, 0.0, 0.0, axes="sxyz")
    t_world = trimesh.transformations.translation_matrix(WORLD_TRANSLATION)
    mesh.apply_transform(t_world @ r_world)
    return mesh


def write_pcd_binary(points: np.ndarray, path: Path) -> None:
    n = points.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(points.astype(np.float32, copy=False).tobytes(order="C"))


def write_pgm_p5(image: np.ndarray, path: Path) -> None:
    if image.dtype != np.uint8:
        raise ValueError("PGM writer expects uint8 image")
    h, w = image.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(image.tobytes(order="C"))


def rasterize(points_xy: np.ndarray, resolution: float,
              x_min: float, y_min: float,
              width: int, height: int) -> np.ndarray:
    grid = np.zeros((height, width), dtype=np.uint8)
    gx = np.floor((points_xy[:, 0] - x_min) / resolution).astype(np.int64)
    gy = np.floor((points_xy[:, 1] - y_min) / resolution).astype(np.int64)
    mask = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[mask], gx[mask]] = 255
    return grid


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--stl", required=True, help="Path to rmul_2026.stl")
    ap.add_argument("--map-out-dir", required=True)
    ap.add_argument("--pcd-out-dir", required=True)
    ap.add_argument("--map-name", default="rmul_2026")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--sample-count", type=int, default=1_500_000)
    ap.add_argument("--slice-z-min", type=float, default=0.10,
                    help="Lower z for the 2D map slice (m).")
    ap.add_argument("--slice-z-max", type=float, default=0.45,
                    help="Upper z for the 2D map slice (m).")
    ap.add_argument("--loc-z-min", type=float, default=0.02,
                    help="Lower z for the localisation point cloud (m).")
    ap.add_argument("--loc-z-max", type=float, default=1.20,
                    help="Upper z for the localisation point cloud (m).")
    ap.add_argument("--padding", type=float, default=1.0,
                    help="Free-space padding around the arena (m).")
    args = ap.parse_args()

    stl_path = Path(args.stl).expanduser().resolve()
    map_dir = Path(args.map_out_dir).expanduser().resolve()
    pcd_dir = Path(args.pcd_out_dir).expanduser().resolve()
    map_dir.mkdir(parents=True, exist_ok=True)
    pcd_dir.mkdir(parents=True, exist_ok=True)

    print(f"[1/6] Loading {stl_path}")
    mesh = trimesh.load(stl_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        raise SystemExit(f"Expected Trimesh, got {type(mesh).__name__}")
    print(f"      faces={len(mesh.faces)}  watertight={mesh.is_watertight}")

    print("[2/6] Applying scale + link pose + world pose")
    apply_full_transform(mesh)
    b = mesh.bounds
    print(f"      world bounds  x[{b[0,0]:.2f}, {b[1,0]:.2f}]  "
          f"y[{b[0,1]:.2f}, {b[1,1]:.2f}]  z[{b[0,2]:.2f}, {b[1,2]:.2f}]")

    print(f"[3/6] Sampling {args.sample_count} points on surface")
    points, _ = trimesh.sample.sample_surface_even(mesh, count=args.sample_count)
    points = np.asarray(points, dtype=np.float64)
    print(f"      got {points.shape[0]} points")

    full_pcd = pcd_dir / f"{args.map_name}.pcd"
    print(f"[4/6] Writing full point cloud -> {full_pcd}")
    write_pcd_binary(points, full_pcd)

    loc_mask = (points[:, 2] >= args.loc_z_min) & (points[:, 2] <= args.loc_z_max)
    loc_pcd = pcd_dir / f"{args.map_name}_localization.pcd"
    print(f"      Writing localisation PCD (z in [{args.loc_z_min}, {args.loc_z_max}]) "
          f"-> {loc_pcd}  ({int(loc_mask.sum())} pts)")
    write_pcd_binary(points[loc_mask], loc_pcd)

    print(f"[5/6] Slicing z in [{args.slice_z_min}, {args.slice_z_max}] for the 2D map")
    slab_mask = (points[:, 2] >= args.slice_z_min) & (points[:, 2] <= args.slice_z_max)
    slab = points[slab_mask, :2]
    if slab.shape[0] < 1000:
        raise SystemExit(
            f"Only {slab.shape[0]} points in slice band, adjust --slice-z-*")

    x_min = float(slab[:, 0].min() - args.padding)
    y_min = float(slab[:, 1].min() - args.padding)
    x_max = float(slab[:, 0].max() + args.padding)
    y_max = float(slab[:, 1].max() + args.padding)
    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))
    print(f"      grid {width} x {height} cells @ {args.resolution} m, "
          f"origin=({x_min:.3f}, {y_min:.3f})")

    obstacle = rasterize(slab, args.resolution, x_min, y_min, width, height)

    # nav_msgs convention with negate=0:
    #   probability = (255 - pixel) / 255
    #   occupied when probability > occupied_thresh (default 0.65)
    # So obstacles are BLACK (0) and free space is WHITE (255).
    pgm = np.full_like(obstacle, 255)
    pgm[obstacle > 0] = 0

    # PGM stores row 0 at the top; ROS map_server treats the first row as the
    # highest-y row of the OccupancyGrid, so we flip along y.
    pgm_flipped = np.flipud(pgm)

    pgm_path = map_dir / f"{args.map_name}.pgm"
    yaml_path = map_dir / f"{args.map_name}.yaml"
    print(f"[6/6] Writing PGM  -> {pgm_path}")
    write_pgm_p5(pgm_flipped, pgm_path)

    yaml_data = {
        "image": f"{args.map_name}.pgm",
        "mode": "trinary",
        "resolution": float(args.resolution),
        "origin": [x_min, y_min, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w") as f:
        yaml.safe_dump(yaml_data, f, default_flow_style=False, sort_keys=False)
    print(f"      Writing YAML -> {yaml_path}")

    occ_cells = int((pgm_flipped == 0).sum())
    print(f"Done. occupied cells: {occ_cells}, total cells: {width * height}, "
          f"occ ratio: {occ_cells / (width * height):.3%}")


if __name__ == "__main__":
    main()
