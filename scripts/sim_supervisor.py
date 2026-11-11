#!/usr/bin/env python3
"""Own and clean only the process group started for this simulation launch."""
import os
import signal
import subprocess
import sys
import time


def main():
    command = sys.argv[1:]
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        raise SystemExit('simulation command required')
    owner_pid = os.getppid()
    child = subprocess.Popen(command, start_new_session=True)
    interrupted = False

    def forward(_signum, _frame):
        nonlocal interrupted
        interrupted = True
        try:
            # Let launch coordinate its children once; signalling the whole
            # group here would deliver duplicate SIGINT to each ROS process.
            os.kill(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    try:
        while True:
            try:
                code = child.wait(timeout=0.2)
                break
            except subprocess.TimeoutExpired:
                # A terminated harness can otherwise orphan a healthy simulator
                # indefinitely. Only monitor the parent that started this run.
                if os.getppid() != owner_pid and not interrupted:
                    forward(signal.SIGTERM, None)
    finally:
        # Launch may exit before its Gazebo descendants. Waiting only for the
        # launch PID would leave them and inherited runtime locks alive.
        for sig, budget in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)]:
            try:
                os.killpg(child.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic()+budget
            while time.monotonic() < deadline:
                try:
                    os.killpg(child.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(.1)
        child.wait()
    return 130 if interrupted else code


if __name__ == '__main__':
    sys.exit(main())
