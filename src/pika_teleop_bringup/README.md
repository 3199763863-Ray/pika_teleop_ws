# Pika Teleop Bringup

本包提供正式与 Bag/Demo 两份 launch、官方 Pika 进程 supervisor 和两份 ROS 参数文件。源文件分别是 [`pika_teleop.launch.py`](launch/pika_teleop.launch.py)、[`pika_bag.launch.py`](launch/pika_bag.launch.py)、[`pika_config.yam`](config/ros/pika_config.yam)、[`pika_bag_config.yam`](config/ros/pika_bag_config.yam)。文件后缀确实是 `.yam`。

## 两种 launch

| 模式 | 本仓库启动的节点 | 起点和外部依赖 |
|---|---|---|
| 正式 | `pika_session_manager`、`pika_teleop_publisher`、`pika_realman_mapper` | Mapper 每侧启用时必须读取 RealMan TCP TF；Session Manager 需要外部 Recorder 和左右回零 Action；运动接收器也在本仓库之外 |
| Bag/Demo | `pika_teleop_virtual_receiver`、`pika_teleop_publisher`、`pika_realman_mapper` | 使用配置 TCP 起点；无 Session Manager、Recorder 和自动回零 |

两模式均由 Bridge 订阅官方 `/pika_pose_l|r` 和 `/gripper_l|r/joint_state`，以 20 Hz 发布 State；Mapper 对有效侧以 20 Hz 发布 Pose/速度，夹爪命令独立限频为最多 4 Hz。Bag launch 不自动调用 `ros2 bag record`，也不模拟双击。两个 launch 使用相同的 `set_enabled` 服务名，不能同时运行。

## 构建与启动

在目标 Ubuntu 22.04 / ROS 2 Humble 机器上：

```bash
cd ~/pika_teleop_ws
source /opt/ros/humble/setup.bash
source ~/pika_ros/install/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=65  # 此机器示例；应与外部节点一致
```

官方 Pika 已运行时，任选一个 launch：

```bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
# 或：ros2 launch pika_teleop_bringup pika_bag.launch.py
```

如果还没有启动官方 Pika，以正式模式为例可运行：

```bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py \
  start_pika_official:=true \
  pika_ros_ws:="$HOME/pika_ros" \
  left_serial_port:=/dev/pika_gripper_left \
  right_serial_port:=/dev/pika_gripper_right
```

Bag 模式将命令中的 `pika_teleop.launch.py` 换成 `pika_bag.launch.py`。`start_pika_official` 默认 `false`；官方节点已运行时不要再次拉起。默认工作区使用当前用户 `~/pika_ros`，串口默认使用上述别名，可按机器实际路径覆盖。部署机 `user2@192.168.31.97` 的别名按 USB 物理口绑定，换口后先确认左右映射。

当选择由本 launch 启动官方 Pika 时，[`official_stack_supervisor.py`](pika_teleop_bringup/official_stack_supervisor.py) 在独立进程组内运行官方 `sensor_tools open_multi_sensor_with_teleop.launch.py`。Ctrl+C 后依次请求 SIGINT，超时再升级为 SIGTERM/SIGKILL，以清理它拉起的定位器、串口和 RViz 等子进程。supervisor 负责进程退出清理，不能修复位姿卡顿、Wi-Fi 或传感器断流。独立于本 launch 启动的官方进程需由其原启动终端停止。

两份 launch 均尝试将 `~/.ros/log/pika_teleop_latest` 链向本次运行的 `~/.ros/log/<时间戳>-...` 目录，并写入 `run_info.txt`（launch 文件、时间、命令参数）。节点输出同时进入终端和 ROS 日志。符号链接或文件写入失败被忽略，应以 launch 实际报告的日志目录为准。

## 参数边界

正式 YAML：Bridge State 20 Hz、`stale_stop_ms` 未写入而使用代码默认 **50 ms**；Mapper 命令 20 Hz、State watchdog 200 ms、`require_realman_start_tf=true`。Bag YAML：Bridge State 20 Hz、`stale_stop_ms=2000 ms`；Mapper 命令 20 Hz、State watchdog 200 ms、`require_realman_start_tf=false`；已提交的 Virtual Receiver watchdog 为 200 ms。部署机 2026-10-08 的未提交 Bag 配置把最后一项改为 **2000 ms**，本任务保留该值。完整对照见 [根 README](../../README.md) 与各节点 README。

正式模式 Pose `frame_id` 是 `l/base_link`、`r/base_link`，Bag Pose 是 `l/work/pikabase`、`r/work/pikabase`；两模式速度消息均用 `l/work/pikabase`、`r/work/pikabase`。`l/link_6`、`r/link_6` 是正式模式启动时查询的 TCP TF。外部接收器需要按自己的 TF 约定解释这些名称，不能从字符串相似推断等价。

修改 `.yam` 后需重启 launch；如果构建时没有 `--symlink-install`，还需重建让安装目录更新。只改本包文档无需重建或重启节点。

## Bag 手动录制

先在 Bag 模式分别启用左右侧并确认目标 Topic 有消息，再在同一 ROS 域的另一终端执行：

```bash
ros2 bag record -o pika_realman_demo \
  /pika/l/cartesian_pose /pika/l/cartesian_velocity /pika/l/gripper_percentage \
  /pika/r/cartesian_pose /pika/r/cartesian_velocity /pika/r/gripper_percentage
```

停止录制后运行 `ros2 bag info pika_realman_demo` 检查。此操作由用户单独执行；Bag launch 本身不会生成数据集。正式模式也仅通过 `/recording/manage` 请求**外部** Recorder，真正写盘路径和相机健康由其实现决定。

## 命令行启停与日志限流

在部署机上，`bash ~/pika_teleop_ws/scripts/pika_start.sh left` 与 `bash ~/pika_teleop_ws/scripts/pika_stop.sh left`（也可用 `right`）调用 Bridge 的 `manual_enable`，分别对应请求双击 START 与三击 USER_STOP。脚本自动加载 ROS/本工作区，`ROS_DOMAIN_ID` 未设置时默认 65。START 的实际成功还取决于输入、下游服务和正式模式录制准入。

可选运行 `python3 ~/pika_teleop_ws/scripts/limit_pika_locator_logs.py`，将官方定位器 launch 的日志级别设为 `error`，并在官方工作区生成 `.before_log_limit` 备份；重新启动官方栈后生效，`--restore` 可还原。它只抑制重复 WARN，不改变 Bridge 超时保护，也不能修复真实数据断流。

## 最小检查

在已完成 `source` 且 ROS 域一致的终端：

```bash
ros2 topic hz /pika_teleop/left/state
ros2 topic info -v /pika_teleop/left/state
ros2 topic echo /pika/l/cartesian_velocity --qos-reliability reliable
ros2 topic echo /pika_session/state --qos-durability transient_local  # 仅正式模式
tail -n 100 ~/.ros/log/pika_teleop_latest/launch.log
```

`INPUT_UNUSABLE` 指原始位姿或夹爪不满足 Bridge 时效/数值条件；`STALE_STOP` 与 `POSE_JUMP_STOP` 是 Bridge 停止原因；`SESSION STOPPED: state watchdog timeout` 是 Mapper State 接收超时。日志频率限制只减少输出，不代表故障消失。真机首次动作需低速、空载并保持急停可用。
