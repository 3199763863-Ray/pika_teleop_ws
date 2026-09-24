#!/bin/bash
# 命令行停止 pika 录制传输（等价于三击夹爪）
set -e
SIDE="${1:-left}"
case "$SIDE" in
  left|right) ;;
  *) echo "用法: $0 [left|right]（默认 left）" >&2; exit 1 ;;
esac

source /opt/ros/humble/setup.bash
source "$HOME/pika_teleop_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-65}"

SERVICE="/pika_teleop/${SIDE}/manual_enable"
if ! ros2 service list 2>/dev/null | grep -qx "$SERVICE"; then
  echo "错误: 找不到服务 $SERVICE (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)" >&2
  echo "请确认 pika_teleop.launch.py / pika_bag.launch.py 已启动。" >&2
  exit 1
fi

timeout 15 ros2 service call "$SERVICE" \
  pika_teleop_interfaces/srv/SetTeleopEnabled \
  "{enable: false, reason: 'manual'}"
