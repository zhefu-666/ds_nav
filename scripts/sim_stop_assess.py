#!/usr/bin/env python3
"""Fail-closed physical cancel/failure scoring with independent full-run evidence."""
import argparse
import json
import math
from pathlib import Path
from sim_evidence_gate import evaluate, shutdown_gate


def assess(events, mission, run, shutdown, launch_log):
    start = mission['sim_stop_request_ns']
    cleanup = mission['sim_cleanup_ns']
    truth = [e for e in events if e.get('kind') == 'truth']
    end = max((e['stamp_ns'] for e in truth), default=start)
    begin = min((e['stamp_ns'] for e in truth), default=start)
    report = evaluate(events, begin, end) if end > begin else dict(passed=False, reasons=['missing_truth'])
    reasons = report['reasons']
    reasons.extend(shutdown_gate(shutdown, launch_log))
    cancelled = run.get('injected_cancel_stage') == 'passage' and mission.get('cancelled') is True
    failed = (run.get('injected_failure', {}).get('kind') == 'odometry_publisher_exit'
              and mission.get('completed') is False and not mission.get('cancelled') and bool(mission.get('failure')))
    if not (cancelled or failed) or mission.get('disabled_confirmed') is not True:
        reasons.append('stop_or_injection_not_confirmed')
    if not start <= cleanup <= end:
        reasons.append('invalid_stop_cleanup_interval')
    before = [e for e in truth if e['stamp_ns'] <= start]
    center = {'west':(-2.5,4.46), 'east':(2.5,-2.46)}.get(mission.get('route', {}).get('tunnel'))
    in_tunnel = bool(center and before and 0 <= start-before[-1]['stamp_ns'] <= 150_000_000
                     and abs(before[-1]['pose']['position'][0]-center[0]) < 1.
                     and abs(before[-1]['pose']['position'][1]-center[1]) < .4)
    if not in_tunnel: reasons.append('stop_not_proven_inside_tunnel')
    commands = [e for e in events if e.get('kind') == 'command']
    moving = any(any(abs(v)>1e-3 for v in e['velocity']) for e in commands
                 if start-2_000_000_000 <= e['stamp_ns'] < start)
    if not moving: reasons.append('no_motion_before_stop')
    after = [e for e in commands if e['stamp_ns'] >= cleanup]
    if not after or any(any(not math.isfinite(v) or abs(v)>1e-9 for v in e['velocity']) for e in after):
        reasons.append('cleanup_commands_missing_or_nonzero')
    stable = [e for e in truth if e['stamp_ns'] >= start+2_000_000_000]
    observation = (stable[-1]['stamp_ns']-stable[0]['stamp_ns'])*1e-9 if stable else 0.
    speeds, rates = [], []
    for a, b in zip(stable, stable[1:]):
        dt = (b['stamp_ns']-a['stamp_ns'])*1e-9
        if not 0 < dt <= .15:
            reasons.append('stop_observation_truth_gap'); continue
        speeds.append(math.dist(a['pose']['position'][:2], b['pose']['position'][:2])/dt)
        def yaw(row):
            x,y,z,w = row['pose']['orientation']
            return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        rates.append(abs(math.remainder(yaw(b)-yaw(a), 2*math.pi))/dt)
    if observation < 1 or not speeds or max(speeds) > .05 or max(rates) > .05:
        reasons.append('physical_stop_not_proven')
    report.update(passed=not reasons, in_tunnel_at_stop=in_tunnel,
                  observation_sim_s=observation, max_xy_speed_m_s=max(speeds,default=None),
                  max_yaw_rate_rad_s=max(rates,default=None))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(); p=args.run_directory
    report=assess([json.loads(line) for line in (p/'trajectory.jsonl').read_text().splitlines()],
                  json.loads((p/'mission/result.json').read_text()), json.loads((p/'run_result.json').read_text()),
                  json.loads((p/'shutdown.json').read_text()), (p/'simulation.log').read_text())
    with args.out.open('x') as stream: json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
