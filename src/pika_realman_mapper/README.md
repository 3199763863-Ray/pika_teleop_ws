# Pika RealMan Mapper

`pika_realman_mapper` 读取 Bridge 的左右 `PikaTeleopState`，输出 Pose、速度与夹爪百分比。速度是主要运动命令；`cartesian_pose` 仍为兼容接口。本包不调用 RealMan SDK，也不执行机器人限位、录制或回零。

## 坐标流程

输入 State 已由 Bridge 执行固定预变换 `(x,y,z)→(-z,y,x)`，其 `frame_id` 为 `pika_teleop_frame`。Mapper **在这份预变换后的 Pose 上**建立左右共用基准：左右 Pika 原点中点作原点，Y 正向从左到右，Z 取 Pika 官方竖直轴（在 `pika_teleop_frame` 中配置为 `[-1,0,0]`），X 为 `Y×Z`。源码见 [`pika_base.py`](pika_realman_mapper/pika_base.py)。

标定要求左右 State 都在 Mapper watchdog 时限内收到、两侧 Pose 源时间戳相差不超过 `pika_base_pair_max_stamp_delta_ms`、两手柄水平间距足够，且连续窗口内位置波动不超过阈值。当前配置需 **10 个**稳定的不同样本对，最大波动 **0.01 m**、最小水平间距 **0.10 m**、时间戳最大差 **100 ms**。成功后基准冻结到节点重启。若启动时左右 Pika 的摆放方向不代表预期的 RealMan 基座方向，数值方向可能不符；代码没有用 RealMan TF 旋转校准这组轴。

每次某侧 State 变为 `enabled && valid`，正式模式读取该侧最新可用 RealMan TCP TF 一次：`l/base_link ← l/link_6` 或 `r/base_link ← r/link_6`。若 TF 缺失或位姿非法，该侧继续等待，**不建立 Mapper 会话，也不发布目标**。TF 恢复后以当时 Pika Pose 和 TCP Pose 建立零位；后续不连续读取 TCP TF。Bag 模式 `require_realman_start_tf=false`，改用配置的默认 TCP 位姿。

会话有效并发布命令时，Mapper 同时广播虚拟 TF：正式模式 `l/base_link → l_pika_base_link`、`r/base_link → r_pika_base_link`；Bag 模式父节点为配置的 `l/work/pikabase`、`r/work/pikabase`。虚拟基准轴按设计与对应机器人基座轴平行，平移量让启用瞬间的 Pika 原点与 TCP 原点重合；停止后不继续广播。TF（坐标变换树）用于表达坐标关系，TCP 是机械臂末端工具中心点。

## 位置、姿态和速度

位置始终相对本次启用时的 Pika 位置：`目标位置 = 启动 TCP 位置 + translation_scale × (当前 Pika 共享基准位置 − 启动 Pika 共享基准位置)`。手柄启动姿态不再旋转这个位移，也没有 Mapper 侧的额外固定 90° 位置旋转。

Pose 的姿态字段单独由 `orientation_mapping_mode` 决定：当前 `relative` 以启动 Pika 姿态变化叠加到启动 TCP 姿态；`absolute` 使用映射后的 Pika 当前姿态。`left/right_base_from_pika_quaternion_xyzw` 只参与这个兼容 Pose 姿态计算，当前配置均为单位四元数。

速度估计器取映射后的目标**位置**和共享基准下的当前 Pika **姿态**，按源时间戳使用最近 5 个有效新 Pose 样本的首尾差求线速度及空间角速度。角速度单位 `rad/s`。窗口未满时输出零；过短样本间隔被忽略，超过 `velocity_max_dt_ms` 则清空窗口重新建基线。原始六轴速度经各轴一维卡尔曼、一阶低通和向量模长死区。触发死区时相应滤波状态归零，持续小速度不会跨帧累计越过阈值。当前线速度死区 `0.005 m/s`，角速度死区 `0.012 rad/s`。

这套计算**假定**标定出的共享 Pika 轴与 RealMan 基座命令轴方向一致。当前速度 `frame_id` 仍是 `l/work/pikabase`、`r/work/pikabase`，代码没有验证该外部命名坐标系与标定基准之间的 TF。实际接收器如何解释该名称须现场逐轴核对，不能将 `l/link_6` 或 `l/base_link` 与它互换。

## 接口与发布条件

| 输入 / 输出 | 类型 | 条件或 QoS |
|---|---|---|
| `/pika_teleop/left|right/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT、VOLATILE、depth 1 |
| `/tf`、`/tf_static` | RealMan TCP 变换 | 正式模式启用时查询一次 |
| `/pika/l|r/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | 有效会话内 20 Hz；`frame_id` 见下表 |
| `/pika/l|r/cartesian_velocity` | `geometry_msgs/msg/TwistStamped` | 有效会话内 20 Hz；m/s、rad/s |
| `/pika/l|r/gripper_percentage` | `std_msgs/msg/Float32` | 有效会话内独立限频，最多 4 Hz；范围 0–1 |
| `/tf` | `l_pika_base_link`、`r_pika_base_link` | 有效会话并发布命令时动态广播 |

命令 Topic 的 QoS 均为 `RELIABLE + VOLATILE + depth 1`。Pose 和速度 Topic 可以存在而没有消息：该侧必须收到未超时、数值合法、`enabled=true && valid=true` 的 State，完成共享基准标定，并在正式模式取得 TCP TF。该侧 State 超过 `state_timeout_ms`、变为 invalid 或停止后，Mapper 清除会话和速度状态；watchdog 触发后须先收到 disabled State 才可重新建立会话。

| 消息 `frame_id` | 正式模式 | Bag 模式 |
|---|---|---|
| 左/右 Pose | `l/base_link`、`r/base_link` | `l/work/pikabase`、`r/work/pikabase` |
| 左/右 Velocity | `l/work/pikabase`、`r/work/pikabase` | 同正式 |
| 启动 TCP TF 名称 | `l/link_6`、`r/link_6` | 参数保留，但不查询 TF |

## 参数

下表“默认”来自 [`node.py`](pika_realman_mapper/node.py)；正式和 Bag 生效值来自 [两份 YAML](../pika_teleop_bringup/README.md)。未在 YAML 中写入的值使用代码默认。

| 参数 | 默认 | 正式 / Bag | 影响 |
|---|---:|---:|---|
| `command_rate_hz` | 100 | **20 / 20** | Pose/速度命令发布周期，Hz |
| `state_timeout_ms` | 100 | **200 / 200** | State 接收 watchdog |
| `require_realman_start_tf` | true | **true / false** | 启用时是否必须读取当前 TCP TF |
| `left/right_tcp_frame` | `l/link_6`、`r/link_6` | 同默认 | 正式模式 TF 查询目标 |
| `left/right_pika_base_frame` | `l_pika_base_link`、`r_pika_base_link` | 同默认 | 虚拟 TF 子节点名称 |
| `pika_base_calibration_stable_samples` | 10 | 10 / 10 | 稳定标定窗口样本对数 |
| `pika_base_calibration_max_spread_m` | 0.01 | 0.01 / 0.01 | 窗口内最大位置波动 |
| `pika_base_min_controller_separation_m` | 0.10 | 0.10 / 0.10 | 左右最小水平间距 |
| `pika_base_pair_max_stamp_delta_ms` | 100 | 100 / 100 | 左右源时间戳最大差 |
| `pika_vertical_axis_in_teleop_frame_xyz` | `[-1,0,0]` | 同默认 | 共享基准竖直方向 |
| `translation_scale_left/right` | 1 / 1 | 1 / 1 | 相对位移及线速度倍率 |
| `orientation_mapping_mode` | relative | relative / relative | 仅影响兼容 Pose 姿态 |
| `gripper_publish_rate_hz` | **4** | 4 / 4（YAML 未写） | 夹爪命令最大发布频率，Hz |
| `gripper_filter_cutoff_hz` | 10 | 10 / 10（YAML 未写） | 夹爪百分比一阶滤波截止频率，Hz |
| `left/right_gripper_closed_position` | 0 / 0 | 0 / 0 | 夹爪百分比零点 |
| `left/right_gripper_open_position` | 0.1 / 0.1 | **0.0967 / 0.0967** | 夹爪全开原始值 |
| `velocity_derivative_window_samples` | 2 | **5 / 5** | 线/角速度共用求导窗口 |
| `velocity_min_dt_ms` / `velocity_max_dt_ms` | 5 / 50 | **5 / 150** | 有效新样本的时间间隔 |
| `velocity_kalman_enabled` | false | **true / true** | 六轴卡尔曼开关 |
| `linear_velocity_kalman_process_variance` / `linear_velocity_kalman_measurement_variance` | 0.2 / 0.05 | 同默认 | 线速度响应速度 / 毛刺抑制权重 |
| `angular_velocity_kalman_process_variance` / `angular_velocity_kalman_measurement_variance` | 0.5 / 0.1 | 同默认 | 角速度响应速度 / 毛刺抑制权重 |
| `velocity_filter_cutoff_hz` | 10 | 10 / 10 | 低通截止频率，Hz |
| `linear_velocity_deadband_mps` | 0.02 | **0.005 / 0.005** | 线速度向量死区，m/s |
| `angular_velocity_deadband_radps` | 0.03 | **0.012 / 0.012** | 角速度向量死区，rad/s |

两份 YAML 都配置左默认 TCP 位置 `[-0.323,-0.028,0.304]`、姿态 xyzw `[0.990,0.003,-0.045,-0.137]`，右位置 `[-0.299,0.014,0.319]`、姿态 xyzw `[0.989,-0.052,-0.047,0.128]`。正式模式这些数组只用于参数校验，Bag 模式把它们作为起点。两侧 `base_from_pika_quaternion_xyzw` 均为 `[0,0,0,1]`，仅参与兼容 Pose 姿态。具体数组和 frame 名称仍以当前 `.yam` 为准。

## 运行检查

在目标 Ubuntu 终端加载 `/opt/ros/humble/setup.bash`、官方工作区和本工作区，再运行正式或 Bag launch。观察 `PIKA BASE CALIBRATED` 和每侧 `SESSION STARTED` 日志；正式模式若出现 `START WAITING: TF ... unavailable`，应检查外部 TF 树及配置帧名。可执行：

```bash
ros2 topic hz /pika/l/cartesian_velocity
ros2 topic echo /pika/l/cartesian_velocity --qos-reliability reliable
ros2 topic info -v /pika/l/cartesian_velocity
ros2 run tf2_ros tf2_echo l/base_link l/link_6   # 正式模式，外部 TF 已启动时
```

现场动作方向、命令接收器的坐标解释和真机安全限制需由外部系统实测确认。
