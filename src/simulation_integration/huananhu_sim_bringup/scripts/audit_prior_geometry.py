#!/usr/bin/env python3
"""Read-only geometry/map audit; does not run Gazebo or use estimator truth."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('prior_generator', HERE / 'generate_prior_map.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)


def audit(resources, maps):
    world = (resources / 'worlds/rmul_2027_world.sdf').read_text()
    model = (resources / 'models/rmul_2027/model.sdf').read_text()
    tf, _ = g._model_pose_from_world(world, 'rmul_2027')
    triangles, sources = g._visual_triangles(model, tf, str(resources / 'models'))
    boxes = [(tf @ pose, size, name) for pose, size, name in g._box_triples(model)]
    meta = yaml.safe_load((maps / 'rmul_2027.yaml').read_text())
    with open(maps / meta['image'], 'rb') as stream:
        assert stream.readline().strip() == b'P5'
        width, height = map(int, stream.readline().split())
        assert stream.readline().strip() == b'255'
        pixels = np.flipud(np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width))
    def cell(x, y):
        column = int(np.floor((x - meta['origin'][0]) / meta['resolution']))
        row = int(np.floor((y - meta['origin'][1]) / meta['resolution']))
        return int(pixels[row, column])
    checks = {}
    for name, cx, cy in [('west', -2.5, 4.46), ('east', 2.5, -2.46)]:
        xylo, xyhi = np.array([cx - .175, cy - .4]), np.array([cx + .175, cy + .4])
        bottom = triangles[np.all(abs(triangles[:, :, 2] - .330) < 1e-6, axis=1)]
        selected = bottom[(bottom[:, :, 0].min(1) >= xylo[0] - 1e-6) &
                          (bottom[:, :, 0].max(1) <= xyhi[0] + 1e-6) &
                          (bottom[:, :, 1].min(1) >= xylo[1] - 1e-6) &
                          (bottom[:, :, 1].max(1) <= xyhi[1] + 1e-6)]
        assert len(selected), name + ' roof underside missing'
        roof_name = name + '_tunnel_metal_roof'
        roof = next((pose, size) for pose, size, boxname in boxes if boxname == roof_name)
        roof_corners = g._world_box_corners(*roof)
        assert abs(roof_corners[:, 2].min() - .330) < 1e-8
        assert np.allclose(roof_corners.max(0) - roof_corners.min(0), [.350, .960, .020])
        intrusions = []
        for pose, size, boxname in boxes:
            corners = g._world_box_corners(pose, size)
            low, high = corners.min(0), corners.max(0)
            overlap = np.minimum(high, [*xyhi, .327]) - np.maximum(low, [*xylo, .081])
            if np.all(overlap > 1e-7):
                intrusions.append({'name': boxname, 'overlap_m': overlap.tolist()})
        assert not intrusions, (name, intrusions)
        assert cell(cx, cy) == 254
        rows = pixels[:, int(np.floor((cx - meta['origin'][0]) / meta['resolution']))]
        centerrow = int(np.floor((cy - meta['origin'][1]) / meta['resolution']))
        lo = hi = centerrow
        while lo > 0 and rows[lo - 1] == 254:
            lo -= 1
        while hi + 1 < height and rows[hi + 1] == 254:
            hi += 1
        free_width = (hi - lo + 1) * meta['resolution']
        assert free_width >= .752575283 + .01, (name, free_width)
        checks[name] = {'roof_bottom_world_m': .330, 'floor_surface_world_m': .0805,
                        'geometric_clearance_m': .2495, 'design_clearance_m': .250,
                        'roof_under_triangles': len(selected), 'collision_intrusions': intrusions,
                        'center_cell': cell(cx, cy), 'grid_free_width_m': free_width}
    assert cell(-2., 3.) == 0, 'raised platform must be occupied'
    for x,y in [(-4.2,4.38),(-4.2,3.3),(4.2,-2.54),(4.2,-1.3)]:
        assert cell(x,y)==254, 'audited 0.083m floor panel must not block portal approach'
    assert np.any(pixels == 205), 'unknown exterior padding missing'
    return {'result': 'PASS_STATIC_ONLY', 'visual_sources': sources, 'tunnels': checks,
            'raised_platform_cell': cell(-2., 3.), 'unknown_cells': int((pixels == 205).sum()),
            'map_resolution_m': meta['resolution'],
            'limits': ['No physical simulation performed',
                       'CAD height .244469m leaves only .005031m geometric vertical margin',
                       'MINCO circumscribed-circle footprint remains a narrow-tunnel limitation']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resources', type=Path, default=HERE.parent.parent / 'huananhu_sim_resources')
    parser.add_argument('--maps', type=Path, default=HERE.parent / 'maps')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    result = audit(args.resources, args.maps)
    text = json.dumps(result, indent=2) + '\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
