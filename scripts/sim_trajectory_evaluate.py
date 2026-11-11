#!/usr/bin/env python3
"""Offline same-stamp, same-base trajectory scoring. No ROS/control imports."""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import numpy as np


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-9:
        raise ValueError('invalid quaternion')
    return q / norm


def quat_multiply(a, b):
    a, b = quat_normalize(a), quat_normalize(b)
    av, bv = a[:3], b[:3]
    return quat_normalize(np.r_[a[3]*bv + b[3]*av + np.cross(av, bv), a[3]*b[3] - av@bv])


def rotate(q, v):
    q = quat_normalize(q)
    u = q[:3]
    return np.asarray(v, dtype=float) + 2*np.cross(u, np.cross(u, v) + q[3]*np.asarray(v))


def slerp(a, b, fraction):
    a, b = quat_normalize(a), quat_normalize(b)
    dot = float(a@b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        return quat_normalize(a + fraction*(b-a))
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b)/math.sin(angle)


def yaw(q):
    x, y, z, w = quat_normalize(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def compose(a, b):
    return {'position': (np.asarray(a['position']) + rotate(a['orientation'], b['position'])).tolist(),
            'orientation': quat_multiply(a['orientation'], b['orientation']).tolist()}


def inverse(pose):
    q = quat_normalize(pose['orientation']) * [-1, -1, -1, 1]
    return {'position': rotate(q, -np.asarray(pose['position'])).tolist(), 'orientation': q.tolist()}


def chassis_to_base(pose, offset=(0., 0., -.10)):
    return {'position': (np.asarray(pose['position']) + rotate(pose['orientation'], offset)).tolist(),
            'orientation': quat_normalize(pose['orientation']).tolist()}


def interpolate_pose(rows, stamps, stamp_ns, max_gap_ns):
    i = bisect_left(stamps, stamp_ns)
    if i < len(rows) and stamps[i] == stamp_ns:
        return rows[i]['pose']
    if i == 0 or i == len(rows):
        raise ValueError('truth_not_bracketed')
    a, b = rows[i-1], rows[i]
    gap = stamps[i] - stamps[i-1]
    if gap <= 0 or gap > max_gap_ns:
        raise ValueError('truth_gap_exceeded')
    f = (stamp_ns - stamps[i-1])/gap
    return {'position': ((1-f)*np.asarray(a['pose']['position']) + f*np.asarray(b['pose']['position'])).tolist(),
            'orientation': slerp(a['pose']['orientation'], b['pose']['orientation'], f).tolist()}


def _ordered(rows):
    # Reject rollback rather than silently sorting different simulation epochs.
    stamps = [r['stamp_ns'] for r in rows]
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('clock_rollback_in_odometry')
    unique = {r['stamp_ns']: r for r in rows}
    return list(unique.values())


def evaluate(events, max_truth_gap=.15, min_travel=.20, min_coverage=.90):
    result = {'status': 'FAIL', 'reasons': [], 'metrics': None,
              'alignment': None, 'truth_used_for_control': False}
    header = next((r for r in events if r.get('kind') == 'metadata'), {})
    mode = header.get('mode')
    if mode not in ('prior', 'mapping', 'saved-local'):
        result['reasons'].append('missing_or_invalid_mode'); return result
    truth = [r for r in events if r.get('kind') == 'truth']
    estimates = [r for r in events if r.get('kind') == 'estimate']
    result.update(mode=mode, estimate_samples=len(estimates), truth_samples=len(truth),
                  contact_messages=sum(r.get('contacts', 0) > 0 for r in events if r.get('kind') == 'contact'),
                  contact_entries=sum(r.get('contacts', 0) for r in events if r.get('kind') == 'contact'),
                  opt_path_messages=sum(r.get('kind') == 'opt_path' for r in events),
                  nonempty_normal_opt_path_messages=sum(r.get('kind') == 'opt_path' and
                      r.get('command_count',0) > 0 and r.get('command_flag') == 1 for r in events),
                  nonzero_command_messages=sum(r.get('kind') == 'command' and
                      max(abs(x) for x in r['velocity']) > 1e-4 for r in events))
    if not truth or not estimates:
        result['reasons'].append('missing_truth_or_estimate'); return result
    try:
        truth, estimates = _ordered(truth), _ordered(estimates)
    except ValueError as exc:
        result['reasons'].append(str(exc)); return result
    if any(r.get('parent_frame') != 'world' or
           not r.get('child_frame', '').endswith('chassis') for r in truth):
        result['reasons'].append('unexpected_truth_frame'); return result
    stamps = [r['stamp_ns'] for r in truth]
    pairs, rejected = [], {}
    for row in estimates:
        reason = None
        if row.get('stamp_ns', 0) <= 0:
            reason = 'invalid_zero_measurement_stamp'
        elif row.get('parent_frame') != 'camera_init' or row.get('child_frame') != 'base_footprint':
            reason = 'unexpected_estimate_frame'
        elif mode in ('prior', 'saved-local') and row.get('tf_status') != 'OK':
            reason = 'map_tf_unavailable_at_measurement_stamp'
        if reason:
            rejected[reason] = rejected.get(reason, 0)+1; continue
        try:
            gt = chassis_to_base(interpolate_pose(truth, stamps, row['stamp_ns'], int(max_truth_gap*1e9)))
            if not np.isfinite(gt['position']).all(): raise ValueError('nonfinite_truth')
            est = row['pose_map'] if mode in ('prior', 'saved-local') else row['pose']
            if not np.isfinite(est['position']).all(): raise ValueError('nonfinite_estimate')
            quat_normalize(est['orientation'])
        except (ValueError, KeyError) as exc:
            reason = str(exc); rejected[reason] = rejected.get(reason, 0)+1; continue
        pairs.append((row['stamp_ns'], est, gt))
    result.update(matched_samples=len(pairs), coverage=len(pairs)/len(estimates), rejected=rejected)
    if len(pairs) < 2:
        result['reasons'].append('insufficient_synchronized_samples'); return result
    gauge = {'position': [0.,0.,0.], 'orientation': [0.,0.,0.,1.]}
    if mode in ('mapping', 'saved-local'):
        gauge = compose(pairs[0][2], inverse(pairs[0][1]))
        result['alignment'] = {'type': 'fixed_first_synchronized_pose_SE3',
                               'stamp_ns': pairs[0][0], 'transform_world_from_local': gauge,
                               'evaluation_only': True, 'per_frame_alignment': False}
    else:
        result['alignment'] = {'type': 'none_map_equals_world', 'evaluation_only': True}
    errors, yaw_errors, positions = [], [], []
    for _, est, gt in pairs:
        est = compose(gauge, est)
        errors.append(np.asarray(est['position'])-gt['position'])
        yaw_errors.append(math.atan2(math.sin(yaw(est['orientation'])-yaw(gt['orientation'])),
                                     math.cos(yaw(est['orientation'])-yaw(gt['orientation']))))
        positions.append(gt['position'])
    errors, positions = np.asarray(errors), np.asarray(positions)
    traveled = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    displacement = float(np.linalg.norm(positions[-1]-positions[0]))
    extent = float(np.linalg.norm(positions.max(0)-positions.min(0)))
    result['metrics'] = {'position_rmse_m': float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
                         'position_xy_rmse_m': float(np.sqrt(np.mean(np.sum(errors[:,:2]**2, axis=1)))),
                         'yaw_rmse_deg': float(np.rad2deg(np.sqrt(np.mean(np.asarray(yaw_errors)**2)))),
                         'final_position_error_m': float(np.linalg.norm(errors[-1])),
                         'final_yaw_error_deg': float(abs(np.rad2deg(yaw_errors[-1]))),
                         'truth_travel_m': traveled, 'truth_displacement_m': displacement,
                         'truth_extent_m': extent, 'matched_duration_s': (pairs[-1][0]-pairs[0][0])*1e-9}
    result['motion_status'] = 'MOVING' if extent >= min_travel else 'INSUFFICIENT_MOTION'
    if extent < min_travel: result['reasons'].append('insufficient_motion_static_data_not_motion_acceptance')
    if result['coverage'] < min_coverage: result['reasons'].append('insufficient_sample_coverage')
    if result['contact_messages']: result['reasons'].append('body_contact_detected')
    result['status'] = 'SCORED' if not result['reasons'] else 'FAIL'
    result['acceptance_note'] = 'SCORED means valid motion evidence, not automatic navigation acceptance.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-truth-gap', type=float, default=.15)
    parser.add_argument('--min-travel', type=float, default=.20)
    parser.add_argument('--min-coverage', type=float, default=.90)
    args = parser.parse_args()
    if args.out.exists(): parser.error('output exists; choose a new path')
    if args.max_truth_gap <= 0 or args.min_travel <= 0 or not 0 < args.min_coverage <= 1:
        parser.error('positive gap/travel and coverage in (0,1] required')
    events = [json.loads(line) for line in args.recording.read_text().splitlines() if line.strip()]
    result = evaluate(events, args.max_truth_gap, args.min_travel, args.min_coverage)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['status'] == 'SCORED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
