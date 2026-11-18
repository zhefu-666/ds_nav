#!/usr/bin/env python3
"""Bounded Nav2 single/multiple goal trial; truth never controls the mission.

Requires a running formal simulation with --navigation. Results are evidence of
Nav2 action outcomes, not ground-truth accuracy acceptance (evaluate separately).
"""
import argparse
import json
import math
import signal
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool, String


class Trial(Node):
    def __init__(self):
        super().__init__('sim_navigation_trial',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.valid = False
        self.valid_at = -math.inf
        self.control = ''
        self.control_at = -math.inf
        self.localization_status = ''
        self.localization_status_at = -math.inf
        self.posture_ready = False
        self.posture_at = -math.inf
        self.mode = self.create_publisher(String, '/sim/control_mode', 10)
        self.posture = self.create_publisher(String, '/huananhu/posture', 10)
        self.create_subscription(Bool, '/sim/localization_valid', self.on_valid, 10)
        self.create_subscription(String, '/sim/control_status', self.on_control, 10)
        self.create_subscription(String, '/sim/localization_status', self.on_localization_status, 10)
        self.create_subscription(Bool, '/sim/posture_ready', self.on_posture, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.lifecycle_request = None
        self.lifecycle_request_at = -math.inf
        self.navigator_active = False

    def active(self):
        # A dropped service reply must not pin the first readiness probe for
        # the entire mission timeout. Retry the real lifecycle query only;
        # an absent response never makes the navigator active.
        if (self.lifecycle_request is not None and not self.lifecycle_request.done() and
                time.monotonic()-self.lifecycle_request_at>2.):
            self.lifecycle.remove_pending_request(self.lifecycle_request)
            self.lifecycle_request.cancel()
            self.lifecycle_request=None
        if self.lifecycle_request is not None and self.lifecycle_request.done():
            response = self.lifecycle_request.result()
            self.navigator_active = response is not None and response.current_state.id == 3
            self.lifecycle_request = None
        if self.lifecycle_request is None and self.lifecycle.service_is_ready():
            self.lifecycle_request = self.lifecycle.call_async(GetState.Request())
            self.lifecycle_request_at = time.monotonic()
        return self.navigator_active

    def on_valid(self, msg):
        self.valid, self.valid_at = msg.data, time.monotonic()

    def on_control(self, msg):
        self.control = msg.data
        self.control_at = time.monotonic()

    def on_localization_status(self, msg):
        self.localization_status = msg.data
        self.localization_status_at = time.monotonic()

    def on_posture(self, msg):
        self.posture_ready = msg.data
        self.posture_at = time.monotonic()

    def healthy(self):
        return self.valid and time.monotonic()-self.valid_at < .5

    def wait(self, predicate, seconds, mode='disabled'):
        end = time.monotonic()+seconds
        last = -math.inf
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            if time.monotonic()-last > .1:
                self.mode.publish(String(data=mode))
                last = time.monotonic()
            if predicate():
                return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X','Y','YAW'))
    parser.add_argument('--mission', help='JSON list of [x,y,yaw] goals in local/map coordinates')
    parser.add_argument('--ready-timeout', type=float, default=180)
    parser.add_argument('--goal-timeout', type=float, default=180)
    parser.add_argument('--posture', choices=['stand','tunnel'], default='tunnel')
    parser.add_argument('--model', choices=['huananhu','low_profile_robot'], default='huananhu')
    parser.add_argument('--cancel-after', type=float, help='bounded cancellation test, wall seconds after acceptance')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if args.cancel_after is not None and (not math.isfinite(args.cancel_after) or args.cancel_after <= 0):
        parser.error('--cancel-after must be finite and positive')
    goals = json.loads(Path(args.mission).read_text()) if args.mission else [args.goal]
    if not goals or any(g is None or len(g)!=3 or not all(math.isfinite(x) for x in g) for g in goals):
        parser.error('provide finite --goal X Y YAW or --mission')
    # JSON may contain integer coordinates; ROS Point setters require float.
    goals = [[float(v) for v in goal] for goal in goals]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Trial()
    result = {'completed':False, 'evaluation_scope':'Nav2 action and safety, not accuracy',
              'goals':[], 'failure':None}
    handle = None
    sent = None
    future = None
    def request_stop(_signum, _frame):
        # Keep ROS alive for action cancellation, disabling control and the report.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        # Repeat posture/mode requests while waiting; one-shot startup messages
        # can be lost before DDS discovery has completed.
        last_posture = [-math.inf]
        def ready():
            if args.model == 'huananhu' and time.monotonic()-last_posture[0] > .2:
                node.posture.publish(String(data=args.posture))
                last_posture[0] = time.monotonic()
            posture_ok = args.model == 'low_profile_robot' or (
                node.posture_ready and time.monotonic()-node.posture_at < .5)
            return node.healthy() and posture_ok and node.client.server_is_ready() and node.active()
        if not node.wait(ready, args.ready_timeout):
            raise RuntimeError('localization_posture_or_nav2_readiness_timeout')
        for position in goals:
            sent = None
            future = None
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = position[:2]
            goal.pose.pose.orientation.z = math.sin(position[2]/2)
            goal.pose.pose.orientation.w = math.cos(position[2]/2)
            sent = node.client.send_goal_async(goal)
            if not node.wait(sent.done, 10, mode='auto'):
                raise RuntimeError('goal_submission_timeout')
            handle = sent.result()
            if not handle.accepted:
                raise RuntimeError('goal_rejected')
            future = handle.get_result_async()
            item = {'target':position, 'action_status':None,
                    'sim_begin_ns':node.get_clock().now().nanoseconds}
            result['goals'].append(item)
            begin, invalid_since = time.monotonic(), None
            previous_clock_ns = node.get_clock().now().nanoseconds
            while not future.done() and time.monotonic()-begin < args.goal_timeout:
                node.wait(lambda:future.done(), .1, mode='auto')
                clock_ns = node.get_clock().now().nanoseconds
                if clock_ns < previous_clock_ns:
                    raise RuntimeError('clock_reset_during_goal')
                paused = (clock_ns == previous_clock_ns and
                          node.localization_status == 'simulation_clock_paused' and
                          time.monotonic()-node.localization_status_at < .5)
                previous_clock_ns = clock_ns
                if args.cancel_after is not None and time.monotonic()-begin >= args.cancel_after:
                    raise RuntimeError('requested_cancel_test')
                if paused:
                    # The existing gate already holds zero while the simulator
                    # is paused. Retain the action, with its wall timeout and
                    # explicit cancellation still enforced; never clear a real
                    # estimator loss or accept a stale pause status.
                    invalid_since = None
                elif not node.healthy():
                    invalid_since = invalid_since or time.monotonic()
                    if time.monotonic()-invalid_since > .5:
                        raise RuntimeError('localization_invalid_during_goal')
                else:
                    invalid_since = None
                if node.control in ('body_contact_stop','estop'):
                    raise RuntimeError(node.control)
            item.update(wall_seconds=time.monotonic()-begin,
                        sim_end_ns=node.get_clock().now().nanoseconds,
                        action_status=future.result().status if future.done() else None)
            if item['action_status'] != 4:
                raise RuntimeError('goal_failed_or_timed_out')
            handle = None
        result['completed'] = True
    except (RuntimeError, KeyboardInterrupt) as exc:
        result['failure'] = str(exc) or 'interrupted'
    finally:
        # Stop auto control immediately, including during a goal-acceptance race.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        result['cleanup_errors'] = []
        result['sim_stop_request_ns'] = node.get_clock().now().nanoseconds
        try:
            node.mode.publish(String(data='disabled'))
            if handle is None and sent is not None:
                if node.wait(sent.done, 3):
                    handle = sent.result()
                else:
                    result['cleanup_errors'].append('goal_acceptance_unresolved')
            if handle is not None and handle.accepted and not result['completed']:
                result['cancel_requested'] = True
                cancelled = handle.cancel_goal_async()
                acknowledged = node.wait(cancelled.done, 3)
                response = cancelled.result() if acknowledged else None
                result['cancel_acknowledged'] = bool(response and response.goals_canceling)
                future = future or handle.get_result_async()
                if node.wait(future.done, 8):
                    result['cancel_terminal_status'] = future.result().status
                else:
                    result['cleanup_errors'].append('cancel_terminal_timeout')
            node.wait(lambda:False, .5, mode='disabled')
        except Exception as exc:
            result['cleanup_errors'].append(str(exc))
        finally:
            control_fresh = time.monotonic()-node.control_at <= .5
            if result['completed'] and (node.control != 'disabled' or not control_fresh):
                result['cleanup_errors'].append('disabled_control_not_observed')
            result['final_control_status'] = node.control
            result['final_control_fresh'] = control_fresh
            result['sim_cleanup_ns'] = node.get_clock().now().nanoseconds
            result['cancel_test_passed'] = bool(
                args.cancel_after is not None and result['failure'] == 'requested_cancel_test'
                and result.get('cancel_acknowledged')
                and result.get('cancel_terminal_status') == 5
                and node.control == 'disabled' and control_fresh and not result['cleanup_errors'])
            target = Path(args.out)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(result, indent=2)+'\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if (result['completed'] or result['cancel_test_passed']) and not result['cleanup_errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
