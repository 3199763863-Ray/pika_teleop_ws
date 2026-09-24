#!/usr/bin/env python3
"""Set the binary Pika locator's ROS log level without changing its safety logic.

The vendor locator source is not installed, so its repeated WARN call cannot be
throttled at the call site. This launch edit suppresses those WARN calls; the
teleop bridge reports unusable input at most once per second per side.
"""

import argparse
import ast
import os
from pathlib import Path
import shutil
import tempfile


ANCHOR = "        name='pika_double_locator',\n        output='screen',"
INSERT = (
    "        name='pika_double_locator',\n"
    "        ros_arguments=[\n"
    "            '--log-level',\n"
    "            os.environ.get('PIKA_LOCATOR_LOG_LEVEL', 'error'),\n"
    "        ],\n"
    "        output='screen',"
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--pika-ros-ws',
        type=Path,
        default=Path(os.environ.get('PIKA_ROS_WS', Path.home() / 'pika_ros')),
    )
    parser.add_argument('--launch-file', type=Path)
    parser.add_argument('--restore', action='store_true')
    args = parser.parse_args()
    launch_file = args.launch_file or (
        args.pika_ros_ws / 'install/pika_locator/share/pika_locator/launch/'
        'pika_double_locator.launch.py'
    )
    backup = launch_file.with_name(launch_file.name + '.before_log_limit')

    if args.restore:
        if not backup.is_file():
            parser.error(f'Backup not found: {backup}')
        shutil.copy2(backup, launch_file)
        print(f'Restored {launch_file}')
        return

    original = launch_file.read_text(encoding='utf-8')
    normalized = original.replace('\r\n', '\n')
    if INSERT in normalized:
        print(f'Already configured: {launch_file}')
        return
    if normalized.count(ANCHOR) != 1:
        parser.error(f'Expected locator launch block not found exactly once: {launch_file}')
    updated = normalized.replace(ANCHOR, INSERT)
    ast.parse(updated, filename=str(launch_file))
    if not backup.exists():
        shutil.copy2(launch_file, backup)
    with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=launch_file.parent,
        prefix=launch_file.name + '.', suffix='.tmp', delete=False,
    ) as staged:
        staged.write(updated)
        staged_path = Path(staged.name)
    try:
        shutil.copymode(launch_file, staged_path)
        os.replace(staged_path, launch_file)
    finally:
        staged_path.unlink(missing_ok=True)
    print(f'Configured {launch_file}: locator WARN suppressed; ERROR retained')
    print('Restart the official sensor launch to apply the new log level.')


if __name__ == '__main__':
    main()
