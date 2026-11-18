#!/usr/bin/env python3
"""Offline evidence completeness gate; never overwrite historical assessments.

A missing contact stream is unknown, not zero contacts. Inputs use simulation
nanoseconds. Explicit bounds prevent a short surviving fragment passing a run.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def check_stream(events, kind, start_ns, end_ns, max_gap_ns):
    rows = [e for e in events if e.get('kind') == kind]
    stamps = [e.get('stamp_ns') for e in rows]
    reasons = []
    if any(not isinstance(t, int) for t in stamps):
        return dict(passed=False, reasons=['invalid_stamp'], samples=len(rows))
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        reasons.append('clock_rollback')
    stamps = sorted(set(t for t in stamps if start_ns <= t <= end_ns))
    if len(stamps) < 2:
        reasons.append('missing_or_insufficient_samples')
    gaps = [b-a for a, b in zip([start_ns]+stamps, stamps+[end_ns])]
    largest = max(gaps)
    if largest > max_gap_ns:
        reasons.append('stream_gap_exceeded')
    return dict(passed=not reasons, reasons=reasons, samples=len(stamps),
                maximum_gap_ns=largest, allowed_gap_ns=max_gap_ns)


def evaluate(events, start_ns, end_ns, max_gap_ns=150_000_000):
    if not 0 <= start_ns < end_ns or max_gap_ns <= 0:
        raise ValueError('positive ordered interval and gap required')
    streams = {kind: check_stream(events, kind, start_ns, end_ns, max_gap_ns)
               for kind in ('truth', 'contact_status', 'command')}
    reasons = [kind+':'+reason for kind, report in streams.items()
               for reason in report['reasons']]
    if not events or events[-1].get('kind') != 'footer':
        reasons.append('missing_recorder_footer')
    else:
        if events[-1].get('reason') not in ('duration_reached', 'interrupted'):
            reasons.append('recorder_failed_or_unknown_exit')
        counts = dict(Counter(row.get('kind') for row in events[:-1]))
        if counts != events[-1].get('counts') or 'footer' in counts:
            reasons.append('recorder_count_mismatch')
    contacts = [e for e in events if e.get('kind') in ('contact', 'contact_status')
                and start_ns <= e.get('stamp_ns', -1) <= end_ns]
    if any(not isinstance(e.get('contacts'), int) or e['contacts'] < 0 for e in contacts):
        reasons.append('invalid_contact_count')
    if any(e.get('contacts', 0) > 0 for e in contacts):
        reasons.append('body_contact_detected')
    if any(e.get('complete') is not True for e in contacts
           if e.get('kind') == 'contact_status'):
        reasons.append('incomplete_contact_components')
    for row in events:
        if row.get('kind') == 'command':
            velocity = row.get('velocity', [])
            if len(velocity) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in velocity):
                reasons.append('invalid_command')
                break
        if row.get('kind') != 'truth' or not start_ns <= row.get('stamp_ns', -1) <= end_ns:
            continue
        pose = row.get('pose', {})
        p, q = pose.get('position', []), pose.get('orientation', [])
        values = p+q
        if (len(p) != 3 or len(q) != 4
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or not .99 <= sum(v*v for v in q) <= 1.01):
            reasons.append('invalid_truth_pose')
            break
    return dict(passed=not reasons, evaluation_only=True, truth_used_for_control=False,
                interval_ns=[start_ns, end_ns], streams=streams, reasons=reasons)


def shutdown_gate(report, launch_log):
    reasons = []
    if report.get('shutdown_clean') is not True or report.get('remaining') != []:
        reasons.append('shutdown_not_clean_or_unverified')
    if any(row.get('signal') in ('SIGTERM', 'SIGKILL') for row in report.get('signals', [])):
        reasons.append('supervisor_escalated_shutdown')
    if 'escalating to' in launch_log or "sending signal 'SIGKILL'" in launch_log:
        reasons.append('launch_escalated_shutdown')
    return reasons


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--start-ns', type=int, required=True)
    parser.add_argument('--end-ns', type=int, required=True)
    parser.add_argument('--shutdown-report', type=Path, required=True)
    parser.add_argument('--simulation-log', type=Path, required=True)
    parser.add_argument('--assessment', type=Path, required=True,
                        help='existing task/stop assessment to combine with evidence gates')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    events = [json.loads(line) for line in args.trajectory.read_text().splitlines()]
    result = evaluate(events, args.start_ns, args.end_ns)
    assessment = json.loads(args.assessment.read_text())
    baseline = assessment.get('passed', assessment.get('PASS')) is True
    result['task_assessment'] = str(args.assessment.resolve())
    result['task_passed'] = baseline
    if not baseline:
        result['reasons'].append('task_assessment_failed_or_missing_verdict')
        result['passed'] = False
    result['reasons'].extend(shutdown_gate(
        json.loads(args.shutdown_report.read_text()), args.simulation_log.read_text()))
    result['passed'] = result['passed'] and not result['reasons']
    with args.out.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
