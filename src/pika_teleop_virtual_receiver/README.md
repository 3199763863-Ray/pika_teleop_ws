# Pika Teleop Virtual Receiver

`pika_teleop_virtual_receiver` 只在 Bag/Demo 模式启动。它模拟下游启停服务和 State 接收 watchdog（收不到消息多久就阻断控制门），方便观察 Bridge；**不连接 RealMan、不录制数据，也不改变 Mapper 输出**。

## 接口与行为

本节点提供 `/pika_teleop/left/set_enabled`、`/pika_teleop/right/set_enabled`（`pika_teleop_interfaces/srv/SetTeleopEnabled`），订阅 `/pika_teleop/left/state`、`/pika_teleop/right/state`（`PikaTeleopState`，`BEST_EFFORT + VOLATILE + depth 1`）。Bridge 是服务客户端。正式模式由 Session Manager 提供同名服务，两模式不能同时启动。

服务接受 START/STOP 后，节点保存预期的 `enabled`，等待**服务请求之后的新 State** 与之匹配；超过 `state_transition_warn_ms` 会告警。`accept_start=false` 可让它拒绝 START，用于检查 Bridge 的失败路径。收到 State 后只保存最新帧、接收时间、计数和异常统计；不会每帧打印。

每侧用本机单调时钟计算距最后一条 State 的时间。超过 `state_timeout_ms` 时报告 `WATCHDOG TIMEOUT`，设置 `safe_stop=true`、控制门 `BLOCKED`；State 恢复后报告 `WATCHDOG RECOVERED`。这是模拟器自身的显示和门控，不是外部 RealMan 接收器的安全实现。`velocity_valid=false` 仅说明 State Twist 不可用，不单独阻断 `enabled && valid` 的 State。

`periodic_summary=false` 时仅在服务、状态变化、超时、恢复或首次数据异常时打印。设为 `true` 才每秒输出左右接收频率、Pose、Twist、夹爪、年龄和累计异常；本节点不会主动修复非有限数或四元数错误。

## 参数及当前运行值

| 参数 | 代码默认 | 已提交 Bag YAML | 作用 |
|---|---:|---:|---|
| `state_timeout_ms` | 100 | **200** | 每侧 State 接收 watchdog |
| `state_transition_warn_ms` | 100 | 100（未写入 YAML） | 服务后等待 State 变化的告警阈值 |
| `accept_start` | true | true | 是否接受 START |
| `periodic_summary` | false | false（未写入 YAML） | 每秒完整摘要开关 |

2026-10-08 的部署机有一项**未提交配置**，将 Bag `state_timeout_ms` 改为 `2000 ms`；该机运行时会按 2000 ms 生效。此变动未被本次文档任务修改，详情见根目录 [LOG](../../LOG.md)。Bridge 的 Bag 原始数据 `stale_stop_ms=2000 ms` 与这里的 State watchdog 是两层不同检查；Mapper 还有独立的 `state_timeout_ms=200 ms`。

## 启动和检查

推荐用 [Bag launch](../pika_teleop_bringup/README.md) 启动完整链路。在已加载 `/opt/ros/humble/setup.bash`、本工作区并设置同一 `ROS_DOMAIN_ID` 的 Ubuntu 终端：

```bash
ros2 launch pika_teleop_bringup pika_bag.launch.py
ros2 topic hz /pika_teleop/left/state
ros2 topic echo /pika_teleop/left/state --qos-reliability best_effort
```

独立运行时可以临时覆盖参数，例如 `ros2 run pika_teleop_virtual_receiver virtual_receiver --ros-args -p accept_start:=false`；这时 watchdog 使用代码默认 100 ms，除非另外传入配置文件或参数。Bag launch 不自动调用 `ros2 bag record`。
