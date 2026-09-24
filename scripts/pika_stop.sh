#!/bin/bash
# 命令行关闭 pika 录制传输（代替三击夹爪）
set -e
SIDE="${1:-left}"
case "$SIDE" in
  left|right) ;;
  *) echo "用法: $0 [left|right]（默认 left）" >&2; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
source "$HOME/pika_teleop_ws/install/setup.bash"
ros2 service call "/pika_teleop/${SIDE}/manual_enable" \
  pika_teleop_interfaces/srv/SetTeleopEnabled \
  "{enable: false, reason: 'manual'}"
