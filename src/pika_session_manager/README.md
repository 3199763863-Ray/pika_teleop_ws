# Pika Session Manager

`pika_session_manager` 只在正式模式启动，负责一条全局录制 episode（一次示教从 Recorder START 到 STOP）、左右遥操准入与正常结束后的双臂回零。Recorder、RealMan Action Server 和运动接收器均为外部进程；本包只调用它们的接口。

## 状态与停止语义

```text
启动 → PREPARING --外部 Recorder PREPARE 成功→ READY
READY --首侧 USER_START / Recorder START 成功→ RECORDING
RECORDING --另一侧 USER_START→ RECORDING（加入同一 session_id）
RECORDING --USER_STOP→ STOPPING --双侧新 disabled State + 交接延迟→ RESETTING
RESETTING --左右 MoveJ 成功且 Recorder STOP 已确认→ PREPARING → READY
RESETTING --Recorder STOP 未确认或 MoveJ 失败→ FAILED（人工处理）
RECORDING --STALE_STOP / POSE_JUMP_STOP→ STOPPING → FAILED（不自动 MoveJ）
```

实际代码还使用短暂的 `STARTING` 状态等待首侧 Recorder START 响应。若 Recorder 服务未就绪、调用失败或拒绝 START，则回到 READY 且该侧 START 失败；返回成功但状态不是 `RECORDING` 则进入 FAILED。PREPARE 失败或服务暂不可用时按 `prepare_retry_sec` 重试，成功后才发布可启动。`start_allowed=true` 只在 READY 与 RECORDING。

左右分别 START，但共用一条 recording session。任意**已加入侧**发出正常 USER_STOP，Manager 立即把两侧标为停止、关闭 START gate、发布 `/pika_session/force_stop_all`，并异步请求 Recorder STOP。只有收到**本轮 STOP 之后**左右两侧新的 `enabled=false, valid=false` State，才启动交接计时；未在 `stop_state_timeout_ms` 内确认则进入 FAILED，绝不派发 MoveJ。若 Recorder STOP 成功，从双侧确认与成功回复两者较晚的时刻再等 `reset_dispatch_delay_ms`；若 Recorder 不可用、拒绝、报错或超过交接时限仍无回复，从双侧确认时刻等同样的延迟。随后左右各发一次 MoveJ，等待 Goal 和 Result。Action Server 不可用、拒绝、执行失败或超时均进入 FAILED，不自动重试。

Recorder 失败或无回复时仍可尝试归位，但**录制状态不会被伪造为成功**：即使双臂 MoveJ 成功，也进入 FAILED，须人工核查 Recorder 后恢复。迟到回复只写日志，不重新归位，也不会把 FAILED 改成 READY。`reset_on_user_stop=false` 时正常 STOP 仍请求 Recorder；只有 STOP 确认成功才直接进入 PREPARING，否则超时后 FAILED。未真正启动的取消不自动 MoveJ；已有另一侧活动时仍按有效会话的停止流程处理。

`STALE_STOP` / `POSE_JUMP_STOP` 是异常停止：仍请求 Recorder STOP，但进入 FAILED，**不会自动 MoveJ**。FAILED 没有自动恢复路径；操作员需检查录制、机器人和现场状态，再重启 Session Manager。初次启动只请求 PREPARE，不自动把机械臂移至预设关节位。`middle_reset_joint_degrees` 目前仅校验/保留，不发送中臂回零 Action。4000 ms 只是试验性交接缓冲，不能证明外部速度控制器已释放机器人；`arm is busy` 等 Action 拒绝仍需现场处理。

## ROS 接口

| 方向 | 名称 | 类型与作用 |
|---|---|---|
| 服务端 | `/pika_teleop/left|right/set_enabled` | `pika_teleop_interfaces/srv/SetTeleopEnabled`；Bridge 的 USER_START、USER_STOP、STALE_STOP、POSE_JUMP_STOP |
| 发布 | `/pika_session/start_allowed` | `std_msgs/msg/Bool`；reliable、transient-local、depth 1 |
| 发布 | `/pika_session/state` | `std_msgs/msg/String`；reliable、transient-local、depth 1 |
| 发布 | `/pika_session/force_stop_all` | `std_msgs/msg/Empty`；reliable、volatile、depth 1 |
| 订阅 | `/pika_teleop/left|right/state` | `pika_teleop_interfaces/msg/PikaTeleopState`；best effort、volatile、depth 1，确认本轮双侧已停用 |
| 客户端 | `/recording/manage` | `realman_recording_msgs/srv/ManageRecording`；PREPARE、START、STOP |
| Action 客户端 | `/l/execute_motion`、`/r/execute_motion` | `realman_msgs/action/ExecuteMotion`；正常结束时 MoveJ |

`recording_status_topic` 当前仅声明/读取配置，Manager **没有订阅** `/recording/status`。接口文件还定义 ADOPT、DISCARD 等命令，但本节点仅调用 PREPARE、START、STOP。Recorder 是否录到相机、保存到磁盘及数据质量均要在外部 Recorder 核查，不能从本节点的成功响应推断落盘已验证。

Bag 模式改由 Virtual Receiver 提供同名 `set_enabled` 服务，因此正式和 Bag launch 不能同时运行。

## 当前配置

以下值来自 [正式 YAML](../pika_teleop_bringup/config/ros/pika_config.yam)，代码默认见 [`node.py`](pika_session_manager/node.py)：

| 参数 | 生效值 | 作用 |
|---|---|---|
| `recording_service` | `/recording/manage` | 外部 Recorder 服务 |
| `recording_status_topic` | `/recording/status` | 保留配置；当前无订阅 |
| `recording_profile` / `recording_task` | `teleop_v1` / `pick_and_place` | START 请求的录制配置与任务标签 |
| `recording_duration_sec` | 0 | 请求不设固定时长，由 STOP 结束 |
| `record_cameras` | true | PREPARE/START 请求包含相机录制意图；不在本机生成图像 |
| `prepare_retry_sec` | 2 | PREPARE 失败后的重试间隔 |
| `reset_on_user_stop` | true | 正常 USER_STOP 后允许双臂自动 MoveJ；设为 false 时只结束录制 |
| `reset_dispatch_delay_ms` | 4000 | 双侧停用与 Recorder STOP 成功回复较晚者之后的归位延迟；Recorder 未成功确认时从双侧停用确认起算 |
| `stop_state_timeout_ms` | 6000 | 等待本轮双侧新 disabled State 的上限；超时不移动并进入 FAILED |
| `left_reset_action` / `right_reset_action` | `/l/execute_motion` / `/r/execute_motion` | 左右回零服务端名称 |
| `left_reset_joint_degrees` | `[12.172,25.223,73.054,-16.703,80.307,14.455]` | 左臂正常回零目标，度 |
| `right_reset_joint_degrees` | `[-9.89,18.046,79.074,15.505,79.606,-6.194]` | 右臂正常回零目标，度 |
| `middle_reset_joint_degrees` | `[0,17.997,70,0,90,8.997]` | 仅校验/保留 |
| `reset_velocity_percent` / `reset_blend_radius_percent` | 10 / 0 | MoveJ 速度与交融比例 |
| `reset_timeout_sec` | 120 | 每侧 MoveJ goal 的超时参数，秒；Manager 额外给结果响应 5 秒余量 |

Action（有目标、反馈、结果的长时间操作接口）的目标命令为 `MOVEJ`、参考类型为 `BASE`，`connect=false`。首次真实采集前操作员须按现有流程确认机器人处于可安全起动的位置；实际接收器还需独立实现限位和急停。

## 观察

在同一 ROS 域、完成 `source` 的终端：

```bash
ros2 topic echo /pika_session/state --qos-durability transient_local
ros2 topic echo /pika_session/start_allowed --qos-durability transient_local
ros2 service type /pika_teleop/left/set_enabled
ros2 action list
```

具体启动方式见 [根 README](../../README.md) 与 [Bringup](../pika_teleop_bringup/README.md)。

修改正式 YAML 后重新构建并在**下次**启动正式 launch 时生效；不要同时运行 Bag launch。需要查看本次失败原因时，结合 `/pika_session/state` 和该次 ROS launch 的日志检查 `RECORDER_STOP_UNCONFIRMED`、State 确认和左右 Action Result。没有真实 Recorder 与机械臂测试时，隔离假服务通过不代表现场归位验收完成。
