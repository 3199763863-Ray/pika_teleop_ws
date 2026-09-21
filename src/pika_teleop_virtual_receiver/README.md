# Pika Teleop Virtual Receiver

`pika_teleop_virtual_receiver` 是本机下游模拟器，用于验证 Pika Teleop Bridge 的 Service 握手、左右 state、接收频率、数据内容和 watchdog。它不连接或控制 RealMan，不修改收到的数据，也不保存数据或录制 rosbag。

## Package 与节点

```text
Package:    pika_teleop_virtual_receiver
Node:       pika_teleop_virtual_receiver
Executable: virtual_receiver
```

源码结构：

```text
pika_teleop_virtual_receiver/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── pika_teleop_virtual_receiver
├── README.md
└── pika_teleop_virtual_receiver/
    ├── __init__.py
    └── receiver_node.py
```

该 package 是独立的 `ament_python` package，只依赖 `rclpy` 和 `pika_teleop_interfaces`。

## ROS 接口

Virtual Receiver 是以下两个 Service 的 Server，默认接受所有 START/STOP：

| Service | Type |
|---|---|
| `/pika_teleop/left/set_enabled` | `pika_teleop_interfaces/srv/SetTeleopEnabled` |
| `/pika_teleop/right/set_enabled` | `pika_teleop_interfaces/srv/SetTeleopEnabled` |

收到成功的 `enable=true, reason=USER_START` 后，对应侧的 `expected_enabled` 设为 true。收到 `enable=false` 及 `USER_STOP`、`STALE_STOP` 或 `POSE_JUMP_STOP` 后，对应侧设为 false。日志显示 side、enable、reason、success 和 message。

Receiver 订阅：

| Topic | Type | QoS |
|---|---|---|
| `/pika_teleop/left/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, KEEP_LAST depth 1 |
| `/pika_teleop/right/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, KEEP_LAST depth 1 |

state callback 不逐帧打印，只保存 latest state、receipt time、RX count，并累计数据异常。左右数据、计数、期望状态、watchdog 和统计完全独立。

## 每秒摘要

节点每秒分别输出 LEFT 和 RIGHT：

- `rx_hz`；
- `expected_enabled`、`state.enabled`、`state.valid`、`velocity_valid`；
- `safe_stop` 与 `control_gate`；
- `frame_id`；
- position x/y/z；
- quaternion x/y/z/w 和 norm；
- linear velocity vx/vy/vz；
- angular velocity wx/wy/wz；
- `gripper_position`；
- `pose_age_ms`、`gripper_age_ms`；
- Pose/Gripper source timestamp；
- 累计异常计数。

数据检查包含 position、quaternion、Twist、gripper 和 age 的有限性，以及 quaternion norm 是否大于 epsilon。异常只告警和统计，不做修复、滤波或坐标转换。

## Control Gate 与状态一致性

虚拟控制门仅在以下条件全部成立时显示 `ACCEPTED`：

```text
state.enabled == true
AND state.valid == true
AND watchdog 未超时
```

其他情况均为 `BLOCKED`。`velocity_valid=false` 只表示当前 Twist 不可用，不会单独使整个 state 无效。

成功处理 Service request 后，Receiver 等待该 request 之后的新 state 与 `expected_enabled` 一致。超过 `state_transition_warn_ms` 仍不一致会 warning，但节点继续运行。

## Watchdog

每侧独立保存最后一条 state 的本机单调时钟 receipt time。超过 `state_timeout_ms` 未收到 state 时：

```text
LEFT/RIGHT WATCHDOG TIMEOUT
safe_stop=true
control_gate=BLOCKED
```

state 恢复后会输出 `WATCHDOG RECOVERED`。该行为只模拟未来机器人接收端的安全自检，不控制真实机器人。

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `state_transition_warn_ms` | `100.0` | Service 成功后等待 state.enabled 跟随的告警时间 |
| `state_timeout_ms` | `100.0` | 每侧 state 接收 watchdog 超时 |
| `accept_start` | `true` | false 时拒绝 START，用于测试发送端失败路径 |

拒绝 START 测试：

```bash
ros2 run pika_teleop_virtual_receiver virtual_receiver --ros-args \
  -p accept_start:=false
```

此时返回：

```text
success=false
message="START rejected by virtual receiver"
```

## 构建

```bash
cd /home/lei/pika_teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/lei/pika_teleop_ws/install/setup.bash
```

## 推荐运行顺序

终端 1：Pika 官方双 Sense 节点（已启动时不要重复启动）：

```bash
source ~/pika_ros/install/setup.bash
cd ~/pika_ros/scripts
bash start_multi_sensor_whit_teleop.bash
```

终端 2：先启动 Virtual Receiver，提供 Service Server：

```bash
source /home/lei/pika_teleop_ws/install/setup.bash
ros2 run pika_teleop_virtual_receiver virtual_receiver
```

终端 3：再启动 Teleop Publisher：

```bash
source /home/lei/pika_teleop_ws/install/setup.bash
ros2 run pika_teleop_bridge pika_teleop_publisher
```

## 完整手动验收流程

1. 初始状态：确认左右 `rx_hz` 接近 100，`enabled=false`、`valid=false`、`control_gate=BLOCKED`。
2. 左 Sense 双击：确认收到 LEFT `enable=true reason=USER_START`，随后 left 变为 `enabled=true`、`valid=true`、`control_gate=ACCEPTED`，right 不变。
3. 右 Sense 双击：确认右侧独立 START，左右可同时 ACTIVE。
4. 坐标轴：Sense 向前移动应看到 z 增加；向左移动应看到 y 增加；向下移动应看到 x 增加。输出坐标定义为 `+X` 向下、`+Y` 向左、`+Z` 向前，Receiver 不再转换。
5. 姿态：分别绕目标 X/Y/Z 方向旋转 Sense，人工观察 quaternion 和对应 angular velocity；Receiver 不推断实际动作方向。
6. Twist：平移时对应 linear 分量变化，旋转时对应 angular 分量变化，静止后低通输出逐渐接近零。Receiver 不重新求导。
7. Gripper：分别开合左右夹爪，确认 `gripper_position` 只在对应侧明显变化，不做范围映射。
8. STOP：ACTIVE 时三击，确认第三击数据仍到达，随后收到 `enable=false reason=USER_STOP`，下一状态为 `enabled=false valid=false control_gate=BLOCKED`。
9. Safety：若自然出现 `STALE_STOP` 或 `POSE_JUMP_STOP`，确认对应 disable reason；不要求故意制造设备漂移。
10. Watchdog：测试结束时关闭 `pika_teleop_publisher`，约 `state_timeout_ms` 后确认左右出现 `WATCHDOG TIMEOUT` 且 gate 为 `BLOCKED`。
11. 可选拒绝：以 `accept_start:=false` 重启 Receiver，双击后确认 Publisher 不进入 ACTIVE，并从 `PENDING_START` 回到 IDLE。

真实手动验收完成前不要录制正式 rosbag；本节点本身不会创建任何数据文件。
