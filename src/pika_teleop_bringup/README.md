# Pika Teleop Bringup

本包集中保存 Pika→RealMan 遥操栈的正式共享参数和一键 launch。

同时提供独立的 Bag/Demo 模式，用于不连接 Recorder、相机、RealMan Action 和 Session Manager 时，以真实双击手势产生六个 Mapper Topic，供用户手动录制 rosbag。正式模式与 Bag 模式不得同时启动。

## 架构与数据流

```text
Pika Sense L/R
  -> 官方 Pika 节点（raw pose / gripper）
  -> pika_teleop_publisher（手势、freshness、jump、安全状态）
  -> /pika_teleop/{left,right}/state
  -> pika_realman_mapper（相对运动 + 配置中的固定 TCP 零位）
  -> /pika/l|r/cartesian_pose / velocity / gripper_percentage
  -> 远端 RealMan Receiver / SDK

pika_teleop_publisher
  <-> /pika_teleop/{left,right}/set_enabled
  <-> pika_session_manager
      -> /recording/manage
      -> /l/execute_motion + /r/execute_motion
      -> /pika_session/start_allowed / force_stop_all / state
```

一条示教只有一个全局 Recording Session；左右手可分别进入 ACTIVE 并加入同一 session。任一侧正常 USER_STOP 会结束整条 episode，顺序固定为：Pika 全局停止 → Recording STOP → 左右 MoveJ 并行复位 → Recording PREPARE → 重新开放 START。

## 构建与启动

```bash
cd /home/lei/pika_teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

官方 Pika 节点已经启动时：

```bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
```

需要 launch 同时启动官方 Pika 节点时：

```bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py start_pika_official:=true
```

官方节点已运行时不要再传 `true`。正式 launch 不启动 `pika_teleop_virtual_receiver`。

## Bag/Demo 模式

官方 Pika 节点已经启动时：

```bash
ros2 launch pika_teleop_bringup pika_bag.launch.py
```

需要同时启动官方 Pika 节点时：

```bash
ros2 launch pika_teleop_bringup pika_bag.launch.py start_pika_official:=true
```

Bag launch 只启动 Virtual Receiver、Publisher 和 Mapper。它读取 `config/ros/pika_bag_config.yam`，显式设置 `use_session_gate=false` 和 `accept_start=true`，不启动 Session Manager，也不访问 Recorder、相机或机械臂 Action。

左右分别双击并确认六个 Mapper Topic持续输出后，另开终端手动录制：

```bash
ros2 bag record -o pika_realman_demo \
  /pika/l/cartesian_pose \
  /pika/l/cartesian_velocity \
  /pika/l/gripper_percentage \
  /pika/r/cartesian_pose \
  /pika/r/cartesian_velocity \
  /pika/r/gripper_percentage
```

停止录制后检查：

```bash
ros2 bag info pika_realman_demo
```

Bag launch 不会自动启动 `ros2 bag record`，不会自动模拟双击，也不会让未 START 的一侧无条件输出。

## 运行前提

- 远端必须提供 `/recording/manage`、`/l/execute_motion`、`/r/execute_motion` 和 RealMan command receiver。
- 第一条采集前，操作员必须先让左右机械臂处于 `pika_config.yam` 的默认关节位；节点启动只 PREPARE Recorder，不自动移动真机。
- 首次真实动作须低速、空载、保留急停，并确认默认 TCP、轴方向、关节零位、夹爪范围和接收端 watchdog。

## 验收观察

每个终端先加载当前工作区：

```bash
source /opt/ros/humble/setup.bash
source /home/lei/pika_teleop_ws/install/setup.bash
```

观察 Session：

```bash
ros2 topic echo /pika_session/state
ros2 topic echo /pika_session/start_allowed
```

观察 Publisher state（不要手写消息类型）：

```bash
ros2 topic echo /pika_teleop/left/state --qos-reliability best_effort --qos-durability volatile
ros2 topic echo /pika_teleop/right/state --qos-reliability best_effort --qos-durability volatile
```

观察 Mapper 输出：

```bash
ros2 topic echo /pika/l/cartesian_pose --qos-reliability reliable --qos-durability volatile
ros2 topic echo /pika/r/cartesian_pose --qos-reliability reliable --qos-durability volatile
```

`set_enabled` 是 Service，不是可监听 event；以 Session Manager 日志、`/pika_session/state` 和 state 的 `enabled/valid` 验收。数据由远端 Recorder 保存，本地这套节点不生成 rosbag，实际保存位置由 Recorder 的 profile/配置决定。

共享配置固定为：

```text
/home/lei/pika_teleop_ws/src/pika_teleop_bringup/config/ros/pika_config.yam
```
