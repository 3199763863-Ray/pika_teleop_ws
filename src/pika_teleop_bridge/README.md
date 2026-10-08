# Pika Teleop Bridge

`pika_teleop_bridge` 的可执行节点是 `pika_teleop_publisher`。它缓存官方 Pika 双 Sense 的四路输入，识别夹爪双击/三击，检查输入时效与位姿跳变，并发布左右独立的 `PikaTeleopState`。正式和 Bag launch 均用 YAML 将 State 周期设为 **20 Hz**；代码单独运行时的默认值仍是 100 Hz。

## 输入、输出与 QoS

QoS 是 ROS 2 发布者与订阅者约定的传输可靠性和缓存方式。四个官方输入均为 `RELIABLE + VOLATILE + KEEP_LAST(1)` 订阅：

| Topic | 类型 |
|---|---|
| `/pika_pose_l`、`/pika_pose_r` | `geometry_msgs/msg/PoseStamped` |
| `/gripper_l/joint_state`、`/gripper_r/joint_state` | `sensor_msgs/msg/JointState` |

回调只保存最新值，左右位姿和夹爪不要求严格同一时刻到达。输出 `/pika_teleop/left/state`、`/pika_teleop/right/state` 为 `pika_teleop_interfaces/msg/PikaTeleopState`，QoS 为 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`。即使未启用遥操，State 仍按控制周期发布，此时 `enabled=false`、`valid=false`。

State 中的 `pose` 是 `pika_teleop_frame` 下的 Pika 绝对 Pose，不因 START 归零；`twist` 是 Bridge 对连续新 Pose 估计的速度。还包含原始 `gripper_position`、`enabled`、`valid`、`velocity_valid`、位姿/夹爪源时间戳和年龄。`velocity_valid=false` 时 Twist 为零或不可作为新速度使用。字段定义见 [`PikaTeleopState.msg`](../pika_teleop_interfaces/msg/PikaTeleopState.msg)。

## 固定预变换与速度

Bridge 在发布 State 前执行固定变换 `(x,y,z)→(-z,y,x)`，并对姿态执行同一坐标变换；这一步仍然存在。State `header.frame_id` 固定为 `pika_teleop_frame`。后面的 Mapper 才根据左右 Pika 位置建立共享基准，不能把 Bridge 的预变换写成最终 RealMan 坐标映射。

Bridge 只在收到**新的**位姿样本时，用源时间戳计算相邻位置和姿态变化并一阶低通。角速度以 `rad/s` 表达在 Bridge 的 `pika_teleop_frame` 中。第一帧、无效时间戳、非正时间差或超过 `velocity_max_dt_ms` 的采样间隔会清零并重建速度基线；相同 Pose 不重复求导。Mapper 另用自己的多点窗口、卡尔曼和死区生成下游速度，详见 [Mapper](../pika_realman_mapper/README.md)。

## 手势、Service 与保护

一次点击是夹爪完成“开→合→开”。IDLE 双击申请 `USER_START`，ACTIVE 三击执行 `USER_STOP`。Bridge 是 `/pika_teleop/left|right/set_enabled`（`SetTeleopEnabled`）的客户端；正式模式由 Session Manager 提供服务，Bag 模式由 Virtual Receiver 提供。Bridge 自身还提供 `/pika_teleop/left|right/manual_enable`，供 [`scripts/pika_start.sh`](../../scripts/pika_start.sh) 和 [`scripts/pika_stop.sh`](../../scripts/pika_stop.sh) 使用。命令行入口请求启动/停止，真正进入 ACTIVE 仍取决于下游服务响应和输入有效性。

正式模式订阅 `/pika_session/start_allowed`（`Bool`，reliable/transient-local）以限制新 START，也订阅 `/pika_session/force_stop_all`（`Empty`，reliable/volatile）以本地停止两侧。Bag 设置 `use_session_gate=false`。Bridge 不发布独立的 `event` Topic。

单侧状态：`IDLE → PENDING_START → ACTIVE`。START 异步等待服务成功；等待期间 State 保持 disabled。超时、服务拒绝或数据失效会回到 IDLE。ACTIVE 中位姿/夹爪缺失、非有限或年龄超过 `stale_stop_ms`，会先本地停机，再异步发送 `STALE_STOP`；相邻新 Pose 位置跳变超过 `max_position_jump_m` 或姿态跳变超过 `max_rotation_jump_deg`，会发送 `POSE_JUMP_STOP`。STOP 后不自动恢复，需重新启动手势。Bridge 只限流记录状态变化和故障，日志抑制不改变这些保护。

## 参数来源与当前生效值

代码默认值位于 [`node.py`](pika_teleop_bridge/node.py)，两种 launch 分别加载 [正式 YAML](../pika_teleop_bringup/config/ros/pika_config.yam) 与 [Bag YAML](../pika_teleop_bringup/config/ros/pika_bag_config.yam)。

| 参数 | 代码默认 | 正式生效 | Bag 生效 | 用途 |
|---|---:|---:|---:|---|
| `state_rate_hz` | 100 | **20** | **20** | State 发布与手势检查周期，Hz |
| `stale_stop_ms` | **50** | **50**（YAML 未写） | **2000** | Pose 或夹爪最大允许年龄；放宽会延长断流后的容忍时间 |
| `velocity_max_dt_ms` | 50 | 150 | 150 | Bridge Twist 连续求导的最大时间差 |
| `start_service_timeout_ms` | 10000 | 10000 | 10000 | START 服务响应最长等待 |
| `use_session_gate` | true | true | false | 是否使用 Session START 准入 |
| `velocity_filter_cutoff_hz` | 10 | 10 | 10 | Bridge Twist 低通截止频率，Hz |
| `max_position_jump_m` | 0.08 | 0.08 | 0.08 | ACTIVE 时新 Pose 最大位置跳变 |
| `max_rotation_jump_deg` | 45 | 45 | 45 | ACTIVE 时新 Pose 最大姿态跳变 |
| `gripper_open_threshold` / `gripper_close_threshold` | 0.075 / 0.025 | 同默认 | 同默认 | 手势开合迟滞；与 Mapper 夹爪百分比标定无关 |
| `click_max_interval_ms` / `gesture_reset_timeout_ms` | 450 / 1200 | 同默认 | 同默认 | 点击合并与手势清空时间 |

正式 50 ms 与 Bag 2000 ms 差别很大。若经常出现 `INPUT_UNUSABLE pose_age_ms=...`，先确认源 Topic、时间戳、QoS 和实际年龄，再决定是否调整超时。改 YAML 后重启对应 launch。

## 检查

在 Ubuntu 机器上先 `source /opt/ros/humble/setup.bash`、`source ~/pika_teleop_ws/install/setup.bash`，并设置与节点相同的 `ROS_DOMAIN_ID`：

```bash
ros2 topic hz /pika_teleop/left/state
ros2 topic info -v /pika_teleop/left/state
ros2 topic echo /pika_teleop/left/state --qos-reliability best_effort
ros2 service type /pika_teleop/left/set_enabled
```

Bridge 只负责 Pika 输入标准化和本地安全停止。Recorder、RealMan 目标映射及机械臂运动分别由 Session Manager、Mapper 和外部接收器负责。
