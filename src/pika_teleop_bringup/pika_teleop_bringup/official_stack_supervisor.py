"""Run the official Pika launch in an isolated process group.

The outer bringup launch only has to stop this supervisor.  The supervisor
then forwards shutdown to the complete official process group, including the
locator, both gripper processes, RViz, and any other descendants.
"""

from __future__ import annotations

import argparse
import errno
import os
import signal
import subprocess
import sys
import time
from collections.abc import Sequence


def _group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _signal_group(process_group: int, signal_number: int) -> None:
    try:
        os.killpg(process_group, signal_number)
    except ProcessLookupError:
        pass


def _wait_for_group(
    process_group: int,
    process: subprocess.Popen[bytes],
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        # poll() reaps the group leader after it exits.  This lets killpg(...,
        # 0) report only descendants that are genuinely still running.
        process.poll()
        if not _group_exists(process_group):
            return True
        time.sleep(0.05)
    process.poll()
    return not _group_exists(process_group)


def _stop_group(
    process_group: int,
    process: subprocess.Popen[bytes],
    interrupt_timeout: float,
    terminate_timeout: float,
) -> None:
    if not _group_exists(process_group):
        process.poll()
        return

    print(
        '[pika_official_supervisor] stopping official Pika process group',
        flush=True,
    )
    _signal_group(process_group, signal.SIGINT)
    if _wait_for_group(process_group, process, interrupt_timeout):
        print(
            '[pika_official_supervisor] official Pika processes stopped',
            flush=True,
        )
        return

    print(
        '[pika_official_supervisor] graceful stop timed out; sending SIGTERM',
        flush=True,
    )
    _signal_group(process_group, signal.SIGTERM)
    if _wait_for_group(process_group, process, terminate_timeout):
        print(
            '[pika_official_supervisor] official Pika processes stopped',
            flush=True,
        )
        return

    print(
        '[pika_official_supervisor] forced cleanup of remaining processes',
        flush=True,
    )
    _signal_group(process_group, signal.SIGKILL)
    _wait_for_group(process_group, process, 1.0)


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Supervise one command and clean up all of its descendants.',
    )
    parser.add_argument('--interrupt-timeout', type=float, default=10.0)
    parser.add_argument('--terminate-timeout', type=float, default=3.0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command and args.command[0] == '--':
        args.command = args.command[1:]
    if not args.command:
        parser.error('a command is required after --')
    if args.interrupt_timeout < 0.0 or args.terminate_timeout < 0.0:
        parser.error('timeouts must be non-negative')
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    shutdown_requested = False

    def request_shutdown(_signal_number: int, _frame: object) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        process = subprocess.Popen(args.command, start_new_session=True)
    except OSError as error:
        if error.errno == errno.ENOENT:
            print(
                f'[pika_official_supervisor] command not found: {args.command[0]}',
                file=sys.stderr,
                flush=True,
            )
            return 127
        raise

    process_group = process.pid
    print(
        f'[pika_official_supervisor] official Pika stack started '
        f'(process_group={process_group})',
        flush=True,
    )

    try:
        while not shutdown_requested:
            return_code = process.poll()
            if return_code is not None:
                # A nested launch can exit before one of its children.  Clean
                # the group even in that case so no hardware owner survives.
                _stop_group(
                    process_group,
                    process,
                    args.interrupt_timeout,
                    args.terminate_timeout,
                )
                return return_code
            time.sleep(0.1)
    finally:
        _stop_group(
            process_group,
            process,
            args.interrupt_timeout,
            args.terminate_timeout,
        )

    # A requested launch shutdown is expected and should not be reported as a
    # process failure by the outer launch.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
