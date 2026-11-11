#!/usr/bin/env python3
"""Read-only TF observer. Detect parent/source conflicts; missing evidence fails.
Use --bag for CLOSED bags or --duration WALL_SECONDS for an isolated live domain.
"""
import argparse
import json
import math
from pathlib import Path
import time


class TfAudit:
    def __init__(self):
        self.parents, self.channels, self.publishers, self.last = {}, {}, {}, {}
        self.errors = set()
        self.count = 0

    def observe(self, parent, child, stamp_ns, channel, publisher=None):
        self.count += 1
        if not parent or not child or parent == child or stamp_ns < 0:
            self.errors.add('invalid_tf_edge')
            return
        self.parents.setdefault(child, set()).add(parent)
        self.channels.setdefault(child, set()).add(channel)
        if publisher is not None:
            self.publishers.setdefault(child, set()).add(str(publisher))
        key = (child, parent, channel, publisher)
        if channel == '/tf':
            if key in self.last and stamp_ns < self.last[key]:
                self.errors.add('dynamic_stamp_rollback:'+child)
            self.last[key] = stamp_ns

    def report(self, required):
        conflicts = sorted(k for k,v in self.parents.items() if len(v)>1)
        mixed = sorted(k for k,v in self.channels.items() if len(v)>1)
        sources = sorted(k for k,v in self.publishers.items() if len(v)>1)
        missing = sorted(set(required)-self.parents.keys())
        # Follow parent links, rather than treating repeated child names as proof.
        cycles = set()
        roots = {}
        for start in self.parents:
            chain, current = [], start
            while current in self.parents and len(self.parents[current]) == 1:
                if current in chain:
                    cycles.update(chain[chain.index(current):])
                    break
                chain.append(current)
                current = next(iter(self.parents[current]))
            roots[start] = current
        disconnected = len({roots[k] for k in required if k in roots}) > 1
        reasons = sorted(self.errors)
        if cycles: reasons.append('tf_cycle')
        if disconnected: reasons.append('required_frames_disconnected')
        if conflicts: reasons.append('multiple_parents')
        if mixed: reasons.append('dynamic_static_conflict')
        if sources: reasons.append('multiple_publishers')
        if missing: reasons.append('missing_required_frames')
        if not self.count: reasons.append('no_tf_evidence')
        return {'status': 'FAIL' if reasons else 'PASS', 'reasons': reasons,
                'transform_count': self.count, 'parents': {k:sorted(v) for k,v in self.parents.items()},
                'duplicate_parent_frames': conflicts, 'dynamic_static_conflicts': mixed,
                'multiple_publisher_frames': sources, 'missing_frames': missing,
                'cycle_frames': sorted(cycles), 'required_frames_disconnected': disconnected,
                'scope': 'TF topology/source/monotonicity only; not pose accuracy or exact-stamp TF availability'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--bag', type=Path)
    modes.add_argument('--duration', type=float)
    parser.add_argument('--required', nargs='+', default=['base_footprint', 'camera_init'])
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists(): parser.error('output exists')
    if args.duration is not None and (not math.isfinite(args.duration) or not 0 < args.duration <= 600):
        parser.error('duration must be finite and in (0,600] wall seconds')
    audit = TfAudit()
    def receive(msg, channel, publisher=None):
        for t in msg.transforms:
            audit.observe(t.header.frame_id, t.child_frame_id,
                          t.header.stamp.sec*10**9+t.header.stamp.nanosec, channel, publisher)
    if args.bag:
        from sim_replay_evaluate import bag_messages
        for topic,msg in bag_messages(args.bag, ['/tf', '/tf_static']):
            receive(msg, topic)
    else:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        node = Node('sim_tf_validation_observer')
        def callback(channel):
            def on_message(msg):
                # This installed Humble executor exposes message-only callbacks.
                receive(msg, channel)
            return on_message
        node.create_subscription(TFMessage, '/tf', callback('/tf'), 100)
        node.create_subscription(TFMessage, '/tf_static', callback('/tf_static'),
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        started = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic()-started < args.duration:
                rclpy.spin_once(node, timeout_sec=.1)
        finally:
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    result = audit.report(args.required)
    result['publisher_identity_available'] = False
    result['publisher_check_note'] = 'Bag and this Humble callback API do not identify per-message publishers; no single-publisher claim.'
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
