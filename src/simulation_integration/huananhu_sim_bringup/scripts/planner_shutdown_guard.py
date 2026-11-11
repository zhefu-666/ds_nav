#!/usr/bin/env python3
"""Historical fallback; the default entry now uses simulation_planner_server.

Serialize simulation planner lifecycle cleanup before forwarding shutdown.

The child has its own session: launch's group SIGINT must not race its executor
against Nav2's preshutdown cleanup. This does not change planning behavior.
"""
import signal
import subprocess
import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.signals import SignalHandlerOptions


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: planner_shutdown_guard.py PLANNER_EXECUTABLE [ARGS...]')
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sim_planner_shutdown_guard')
    child = subprocess.Popen(sys.argv[1:], start_new_session=True)
    failed = False
    try:
        while child.poll() is None and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
        if child.poll() is not None:
            return child.returncode if child.returncode >= 0 else 128 - child.returncode
        # Do not install rclpy's signal handler: launch owns our shutdown request.
        try:
            state = node.create_client(GetState, '/planner_server/get_state')
            change = node.create_client(ChangeState, '/planner_server/change_state')

            def call(client, request):
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f'service unavailable: {client.srv_name}')
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
                if not future.done() or future.result() is None:
                    raise RuntimeError(f'service timed out: {client.srv_name}')
                return future.result()

            current = call(state, GetState.Request()).current_state.id
            if current == State.PRIMARY_STATE_ACTIVE:
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_DEACTIVATE
                if not call(change, request).success:
                    raise RuntimeError('planner deactivation rejected')
                current = State.PRIMARY_STATE_INACTIVE
            if current == State.PRIMARY_STATE_INACTIVE:
                request = ChangeState.Request()
                request.transition.id = Transition.TRANSITION_CLEANUP
                if not call(change, request).success:
                    raise RuntimeError('planner cleanup rejected')
                current = State.PRIMARY_STATE_UNCONFIGURED
            if current not in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_FINALIZED):
                raise RuntimeError(f'unexpected planner lifecycle state: {current}')
            print('[planner_shutdown_guard] lifecycle cleanup complete', flush=True)
        except Exception as error:
            failed = True
            print(f'[planner_shutdown_guard] cleanup failed: {error}', file=sys.stderr, flush=True)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                failed = True
                child.terminate()
                try:
                    child.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        node.destroy_node()
        rclpy.shutdown()
    return 1 if failed else (child.returncode if child.returncode >= 0 else 128 - child.returncode)


if __name__ == '__main__':
    sys.exit(main())
