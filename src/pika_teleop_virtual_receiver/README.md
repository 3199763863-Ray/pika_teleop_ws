# Pika Teleop Virtual Receiver

`pika_teleop_virtual_receiver` 只在 Bag/Demo 模式启动。它提供下游启停服务和 State 接收 watchdog；可在单侧正常停止后向外部 RealMan Action Server 异步发送该侧预设 `MOVEJ` 复位 Goal。它不启动 Recorder、不自动录包，也不改变 Mapper 输出。

## 接口与行为

本节点提供 `/pika_teleop/left/set_enabled`、`/pika_teleop/right/set_enabled`（`pika_teleop_interfaces/srv/SetTeleopEnabled`），订阅 `/pika_teleop/left/state`、`/pika_teleop/right/state`（`PikaTeleopState`，`BEST_EFFORT + VOLATILE + depth 1`）。Bridge 是服务客户端。正式模式由 Session Manager 提供同名服务，两模式不能同时启动。

服务接受 START/STOP 后，节点保存预期的 `enabled`，等待**服务请求之后的新 State** 与之匹配；超过 `state_transition_warn_ms` 会告警。`accept_start=false` 可让它拒绝 START，用于检查 Bridge 的失败路径。收到 State 后只保存最新帧、接收时间、计数和异常统计；不会每帧打印。

Bag YAML 默认启用 `reset_on_user_stop`。仅当该侧 START 已被服务接受，且随后收到本轮 `enabled=true, valid=true` State，才将其视为 ACTIVE。此后 Bridge 因三击夹爪或 `scripts/pika_stop.sh <side>` 发出 `reason=USER_STOP`，节点接受 STOP、登记一次待复位；等待 STOP 请求**之后**的新 `enabled=false, valid=false` State，再经过 `reset_dispatch_delay_ms`，异步向对应 Action Server 发送一次预设关节 `MOVEJ` Goal。交接期间该侧 START 被拒绝，另一侧不受影响。没有新 disabled State、状态矛盾或 watchdog 超时都会取消待复位。`STALE_STOP`、`POSE_JUMP_STOP`、启动未成功、重复 STOP、单纯 disabled State 和节点退出都不触发复位。

STOP 服务立即回执，不等 Action Server 或机器人运动。Action Server 缺失、发送异常或拒绝 Goal 只记录告警，不自动重试；收到 Goal 接受回执也只表示服务端接收，**不代表机器人已回到预设角度**。本节点不查询 Action Result，发出 Goal 后会允许新 START；操作者必须现场确认本侧 MoveJ 已完成后才能重新启动遥操。交接延迟只能降低与速度指令争用的概率；外部接收器仍需负责真实停速、互斥、限位和急停。将 Bag YAML 的 `reset_on_user_stop` 设为 `false`，可关闭整个复位流程。

每侧用本机单调时钟计算距最后一条 State 的时间。超过 `state_timeout_ms` 时报告 `WATCHDOG TIMEOUT`，设置 `safe_stop=true`、控制门 `BLOCKED`；State 恢复后报告 `WATCHDOG RECOVERED`。这是模拟器自身的显示和门控，不是外部 RealMan 接收器的安全实现。`velocity_valid=false` 仅说明 State Twist 不可用，不单独阻断 `enabled && valid` 的 State。

`periodic_summary=false` 时仅在服务、状态变化、超时、恢复或首次数据异常时打印。设为 `true` 才每秒输出左右接收频率、Pose、Twist、夹爪、年龄和累计异常；本节点不会主动修复非有限数或四元数错误。

## 参数及当前运行值

| 参数 | 代码默认 | 已提交 Bag YAML | 作用 |
|---|---:|---:|---|
| `state_timeout_ms` | 100 | **2000** | 每侧 State 接收 watchdog |
| `state_transition_warn_ms` | 100 | 100（未写入 YAML） | 服务后等待 State 变化的告警阈值 |
| `accept_start` | true | true | 是否接受 START |
| `periodic_summary` | false | false（未写入 YAML） | 每秒完整摘要开关 |
| `reset_on_user_stop` | false | **true** | 启用单侧正常 STOP 复位；关闭时维持原 Bag 停止行为 |
| `left_reset_action` / `right_reset_action` | `/l/execute_motion` / `/r/execute_motion` | 同左 | 对应侧外部 Action 名称 |
| `left_reset_joint_degrees` | 未设置 | `[12.172, 25.223, 73.054, -16.703, 80.307, 14.455]` | 左臂六关节预设角，单位度 |
| `right_reset_joint_degrees` | 未设置 | `[-9.89, 18.046, 79.074, 15.505, 79.606, -6.194]` | 右臂六关节预设角，单位度 |
| `reset_velocity_percent` / `reset_blend_radius_percent` | 10 / 0 | 10 / 0 | Goal 的速度百分比和融合半径百分比，均需在 0–100 |
| `reset_timeout_sec` | 120 | 120 | 写入 Goal 的服务端超时字段；不是本节点等待时长 |
| `reset_dispatch_delay_ms` | 150 | **4000** | 新 disabled State 之后的异步交接延迟；Bag 试验值覆盖外部 router 的 3000 ms 输入超时和取消耗时，不保证驱动已释放运动所有权 |
| `reset_stop_confirm_timeout_ms` | 1000 | 1000 | STOP 后等待新 disabled State 的最长时间；超时取消复位 |

Bag YAML 的初始复位关节角、Action 名称及速度参数来自当时的正式 YAML，但已复制成独立配置；运行时不读取正式 YAML。启用复位时，两个关节角数组均须为六个有限数，Action 名称非空，其余参数须在上述范围内，否则节点启动失败。Bridge 的 Bag 原始数据 `stale_stop_ms=2000 ms` 与这里的 State watchdog 是两层不同检查；Mapper 还有独立的 `state_timeout_ms=200 ms`。

## 启动和检查

推荐用 [Bag launch](../pika_teleop_bringup/README.md) 启动完整链路。在已加载 `/opt/ros/humble/setup.bash`、本工作区并设置同一 `ROS_DOMAIN_ID` 的 Ubuntu 终端：

```bash
ros2 launch pika_teleop_bringup pika_bag.launch.py
ros2 topic hz /pika_teleop/left/state
ros2 topic echo /pika_teleop/left/state --qos-reliability best_effort
ros2 action list -t | grep execute_motion
```

现场应先在低速、空载、无障碍物且急停可用时确认 Action Server。双击左侧 START 后三击 STOP，观察 `USER_STOP` → `RESET STOP confirmed` → `RESET Goal sent` / `accepted`，并**目视确认只有左臂完成复位**；右侧重复同样检查。另一侧处于遥操中时，确认未误发其 Goal。再检查异常 STOP 和 Action Server 下线的情况：均不得误发复位，正常 STOP 仍应成功。MoveJ 进行中不得再次双击启用同侧。

独立运行时可以临时覆盖参数，例如 `ros2 run pika_teleop_virtual_receiver virtual_receiver --ros-args -p accept_start:=false`；没有 Bag YAML 时 `reset_on_user_stop` 默认 false，watchdog 使用代码默认 100 ms。Bag launch 不自动调用 `ros2 bag record`。
