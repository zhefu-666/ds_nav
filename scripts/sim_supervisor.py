#!/usr/bin/env python3
"""Own this launch's descendants, including Gazebo's separate process groups."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def process_table():
    """PID identity includes kernel start ticks, so reused PIDs are never killed."""
    table = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            table[int(entry.name)] = (int(fields[1]), int(fields[19]), fields[0])
        except (OSError, ValueError, IndexError):
            continue
    return table


def owned_processes(table, owner_pid, known):
    owned = {pid: birth for pid, birth in known.items()
             if pid in table and table[pid][1] == birth}
    parents = {owner_pid} | set(owned)
    while True:
        found = {pid: row[1] for pid, row in table.items()
                 if row[0] in parents and pid != owner_pid}
        added = set(found)-parents
        owned.update(found)
        if not added:
            return owned
        parents.update(added)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shutdown-report', type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        parser.error('simulation command required')
    if args.shutdown_report and args.shutdown_report.exists():
        parser.error('shutdown report exists; choose a new path')
    # Adopt orphaned grandchildren from this run, not unrelated system orphans.
    # Gazebo GUI forks its server into another process group; killpg misses it.
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(36, 1, 0, 0, 0) != 0:  # Linux PR_SET_CHILD_SUBREAPER
        raise OSError(ctypes.get_errno(), 'cannot establish simulation ownership')
    owner_pid = os.getppid()
    child = subprocess.Popen(command, start_new_session=True)
    known = {}
    interrupted = False
    shutdown_started = None
    report = dict(launch_pid=child.pid, signals=[], remaining=[], shutdown_clean=False)

    def collect():
        nonlocal known
        table = process_table()
        known = owned_processes(table, os.getpid(), known)
        # Reap adopted descendants; leave Popen's own child for Popen.wait().
        for pid in list(known):
            if table[pid][2] == 'Z' and pid != child.pid:
                try:
                    os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                    pass
        return {pid: birth for pid, birth in known.items() if table[pid][2] != 'Z'}

    def forward(_signum, _frame):
        nonlocal interrupted, shutdown_started
        if interrupted:
            return
        interrupted = True
        shutdown_started = time.monotonic()
        try:
            os.kill(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    try:
        while True:
            collect()
            if shutdown_started is not None and time.monotonic()-shutdown_started >= 25:
                report['launch_shutdown_timeout'] = True
                code = 130
                break
            try:
                code = child.wait(timeout=.2)
                break
            except subprocess.TimeoutExpired:
                if os.getppid() != owner_pid and not interrupted:
                    forward(signal.SIGTERM, None)
    finally:
        for sig, budget in [(signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)]:
            live = collect()
            if not live:
                break
            for pid, birth in live.items():
                current = process_table().get(pid)
                if current is None or current[1] != birth or current[2] == 'Z':
                    continue
                try:
                    os.kill(pid, sig)
                    report['signals'].append(dict(pid=pid, start_ticks=birth, signal=sig.name))
                except ProcessLookupError:
                    pass
            deadline = time.monotonic()+budget
            while time.monotonic() < deadline and collect():
                time.sleep(.1)
        child.wait()
        report['remaining'] = sorted(collect())
        report['launch_returncode'] = child.returncode
        report['shutdown_clean'] = (not report.get('launch_shutdown_timeout') and not report['remaining'] and not any(
            row['signal'] in ('SIGTERM', 'SIGKILL') for row in report['signals']))
        if args.shutdown_report:
            with args.shutdown_report.open('x') as stream:
                json.dump(report, stream, indent=2)
        print('SIM_SHUTDOWN '+json.dumps(report), flush=True)
    return 130 if interrupted else code


if __name__ == '__main__':
    sys.exit(main())
