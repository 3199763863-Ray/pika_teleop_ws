# Pika Session Manager

`pika_session_manager` 统一管理一条全局录制 episode、左右遥操准入、正常结束后的双臂并行复位，以及下一轮 Recorder PREPARE。Publisher 与 Mapper 不直接依赖录制或复位 Action。

## 状态机

```text
启动: PREPARING --PREPARE成功--> READY

READY --首侧USER_START/Recorder START成功--> RECORDING
RECORDING --另一侧USER_START--> RECORDING（加入同一 session）
RECORDING --USER_STOP--> STOPPING --Recorder STOP成功--> RESETTING
RESETTING --左右MoveJ均成功--> PREPARING --PREPARE成功--> READY

STALE_STOP / POSE_JUMP_STOP:
RECORDING -> STOPPING -> Recorder STOP -> FAILED（禁止自动MoveJ）

任一 STOP、reset 或异常结果失败 -> FAILED
```

`READY` 与 `RECORDING` 发布 `start_allowed=true`；其它状态均为 false。初始启动只 PREPARE Recorder，不会让机械臂运动。

## 接口

提供：

- `/pika_teleop/left/set_enabled`
- `/pika_teleop/right/set_enabled`
- `/pika_session/start_allowed` (`Bool`, reliable/transient-local)
- `/pika_session/force_stop_all` (`Empty`, reliable/volatile)
- `/pika_session/state` (`String`, reliable/transient-local)

调用：

- `/recording/manage` (`realman_recording_msgs/srv/ManageRecording`)
- `/l/execute_motion` (`realman_msgs/action/ExecuteMotion`)
- `/r/execute_motion` (`realman_msgs/action/ExecuteMotion`)

第一次 START 只有在 Recorder 返回 `success=true, state=RECORDING` 后才确认；另一侧随后加入时不重复 START。任一正常 USER_STOP 会先全局强停 Pika command，再 STOP Recorder，成功后才并行发送左右 MoveJ。异常 STOP 会停止录制但明确禁止自动复位。

## 安全前提

- 第一条数据开始前，操作员须先使用现有流程把机器人移到配置的默认关节位。
- 正式运行不要同时启动 `pika_teleop_virtual_receiver`，否则两个同名 Service Server 会冲突。
- RealMan Action Server 必须在正常 USER_STOP 前可用；任何一侧 reject、失败、ABORTED 或 TIMEOUT 都进入 `FAILED`。
- `FAILED` 第一版需人工排查并重启 Session Manager 恢复。

参数统一由 `pika_teleop_bringup/config/ros/pika_config.yam` 提供。
