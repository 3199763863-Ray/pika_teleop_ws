# Pika RealMan Mapper

`pika_realman_mapper` 是位于稳定 Pika Bridge 与 RealMan 控制节点之间的独立 ROS 2 映射层。它从左右 `PikaTeleopState` 建立每次遥操 session 的固定起点，查询 RealMan 当前 TCP TF，并以 100 Hz 发布左右笛卡尔目标、Base-frame 速度和夹爪百分比。

本 package 不实现 START/STOP Service、不调用 RealMan SDK、不处理跨机 DDS、不重复判断 Sense freshness，也不录制 rosbag。

## 分层架构

```text
Pika Sense
  ↓ raw pose / gripper
pika_teleop_publisher
  ↓ /pika_teleop/{left,right}/state
pika_realman_mapper ← TF2: base_link -> link_6
  ↓ pose / velocity / gripper percentage command topics
同事 RealMan Receiver
  ↓ RealMan SDK
RealMan
```

安全责任严格分层：

- Publisher：Sense freshness、NaN/Inf、PoseJump、手势和 START Service 状态机，结果汇总到 `state.enabled/state.valid`。
- Mapper：`enabled && valid` gate、state receipt watchdog、TF reference、最基本 finite/quaternion 检查、session 生命周期和速度 dt。
- RealMan Receiver：command receive watchdog、机器人限位、急停和 SDK 实时下发安全。

Mapper 的 `state_timeout_ms` 只测量多久没有收到新 state，不读取 `pose_age_ms` 或 `gripper_age_ms` 做第二套 Sense stale 判断。

## Package

```text
Package:    pika_realman_mapper
Node:       pika_realman_mapper
Executable: pika_realman_mapper
```

```text
pika_realman_mapper/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/pika_realman_mapper
├── README.md
└── pika_realman_mapper/
    ├── __init__.py
    ├── node.py
    ├── pose_mapper.py
    ├── velocity.py
    └── quaternion_utils.py
```

- `quaternion_utils.py`：xyzw normalize、乘法、逆、最短旋转和向量旋转。
- `pose_mapper.py`：固定 session 起点的相对位姿与夹爪线性映射，不依赖上一帧目标。
- `velocity.py`：从连续的新 target pose 和 Pika source stamp 估计 Base-frame Twist，并执行一阶低通。
- `node.py`：ROS 参数、state gate、异步 TF 等待、左右 session/watchdog 和固定周期发布。

## 输入与 TF

输入 state：

| Topic | Type | QoS |
|---|---|---|
| `/pika_teleop/left/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, depth 1 |
| `/pika_teleop/right/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, depth 1 |

默认 TF：

| Side | Target/base frame | Source/TCP frame |
|---|---|---|
| Left | `l/base_link` | `l/link_6` |
| Right | `r/base_link` | `r/link_6` |

节点使用 `tf2_ros.Buffer` 和 `TransformListener`，不自行订阅 `/tf` 后逐层相乘。TF 不可用时通过异步 future 等待，100 Hz timer 不阻塞；每个 `tf_lookup_timeout_ms` 窗口会回收并重新申请等待，warning 按秒节流。TF 恢复且 state 仍 `enabled && valid` 时才锁存起点，绝不使用零位姿或旧 reference。

当前假设 `link_6` 与对应夹爪/TCP 控制坐标系重合。

## 输出

仅发布 l/r，不创建 `/pika/m/*`：

| Topic | Type | Frame / unit |
|---|---|---|
| `/pika/l/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `l/base_link`, m + xyzw |
| `/pika/r/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `r/base_link`, m + xyzw |
| `/pika/l/cartesian_velocity` | `geometry_msgs/msg/TwistStamped` | `l/base_link`, m/s + rad/s |
| `/pika/r/cartesian_velocity` | `geometry_msgs/msg/TwistStamped` | `r/base_link`, m/s + rad/s |
| `/pika/l/gripper_percentage` | `std_msgs/msg/Float32` | `[0,1]` |
| `/pika/r/gripper_percentage` | `std_msgs/msg/Float32` | `[0,1]` |

六个输出均为 BEST_EFFORT、VOLATILE、KEEP_LAST depth 1。ACTIVE COMMAND 时按 `command_rate_hz` 固定刷新，即使当前 Pose 没变化也继续发布；session reset 后立即停止该侧全部输出，不重复旧目标。

## Session reference

某侧首次收到 `enabled=true && valid=true` 且 TF 可用时，在同一初始化流程锁存：

```text
Pika_start_pose = 当前 state.pose
RM_start_pose   = 当前 base_link -> link_6
```

第一个 target 因此等于/接近当前 `RM_start_pose`，不会跳到 Pika 的绝对坐标。STOP、invalid、Mapper finite 防御失败或 watchdog timeout 都会清除两个起点、上一 target 和速度滤波器。下次 session 必须重新查询 TF。

watchdog timeout 后要求先观察到该侧 `enabled=false` 才允许重新建立 session，避免数据流恢复时沿用或自动续接中断前的 ACTIVE 状态。

## 相对位姿公式

### 平移

```text
delta_p_pika = p_pika_t - p_pika_0
delta_p_base = translation_scale * R_base_from_pika * delta_p_pika
p_target     = p_rm_0 + delta_p_base
```

目标永远由两个固定 START 起点与当前 Pika Pose 计算，不基于上一帧累加，因此不会产生累计漂移。RealMan 当前反馈只在 START 时读取一次。

### 姿态（Base/fixed-frame）

四元数顺序统一为 xyzw，所有参与运算的四元数均 normalize：

```text
q_delta_pika = q_pika_t ⊗ inverse(q_pika_0)
q_delta_base = q_map ⊗ q_delta_pika ⊗ inverse(q_map)
q_target     = normalize(q_delta_base ⊗ q_rm_0)
```

相对增量按固定 RealMan Base Frame 解释，不采用 Tool-frame 增量。实现处理 `q/-q` 等价并使用 shortest rotation，不做四元数分量加减。

## 默认轴映射

左右默认参数相同：

```text
q_base_from_pika = [0.70710678, 0.0, -0.70710678, 0.0]

X_base = -Z_pika
Y_base = -Y_pika
Z_base = -X_pika
```

即：

```text
Pika 向前 (+Z) → RealMan Base -X
Pika 向左 (+Y) → RealMan Base -Y
Pika 向下 (+X) → RealMan Base -Z
```

平移、姿态和由 target pose 求出的速度共享同一 mapping，不在三处写独立符号交换。

该映射是根据当前已确认的 RealMan base/link6 轴关系与 Pika 物理方向得到的第一版配置。接入真实机械臂后，必须在低速、安全条件下逐轴验证。若左右安装或控制语义不同，只修改左右 `*_base_from_pika_quaternion_xyzw` 参数，不修改相对位姿主算法。

## Cartesian velocity

只在 `pose_source_stamp` 改变时，用 Base Frame 下连续两个 target pose 更新速度：

```text
dt = stamp_now - stamp_previous
linear = (p_target_now - p_target_previous) / dt

q_delta = q_target_now ⊗ inverse(q_target_previous)
angular = shortest_axis * shortest_angle / dt
```

六个分量使用因果一阶低通：

```text
tau = 1 / (2*pi*velocity_filter_cutoff_hz)
alpha = dt / (tau + dt)
filtered = previous_filtered + alpha * (raw - previous_filtered)
```

首帧、无效/非递增时间、`dt > velocity_max_dt_ms` 时输出零 Twist 并重新建立速度 baseline。没有新 Pose 时，ACTIVE command tick 发布最近的 filtered Twist；session reset 后停止发布。

## Gripper percentage

每侧独立计算：

```text
percentage =
  (gripper_position - closed_position)
  / (open_position - closed_position)

percentage = clamp(percentage, 0, 1)
```

`0.0` 表示完全闭合，`1.0` 表示完全张开。默认 `0.0/0.1` 是当前初始标定值，必须根据真实左右 Sense 极限开合数据微调。`open_position <= closed_position` 会在节点启动时报错。

## Watchdog 与发布 gate

仅在以下条件全部满足时发布该侧三个 command：

```text
session_initialized
AND state.enabled
AND state.valid
AND state receipt watchdog 正常
AND Mapper Pose/Gripper finite + quaternion 合法
```

ACTIVE/等待 TF 时超过 `state_timeout_ms` 未收到新 state，立即 reset 并停止输出。Mapper 不使用消息中的 age 字段重复判断 Sense freshness。

RealMan 同事端仍必须实现 command receive watchdog；最终链路为：

```text
Bridge valid/stale + Mapper state watchdog + Receiver command watchdog
```

## 参数

| 参数 | 默认值 |
|---|---|
| `command_rate_hz` | `100.0` |
| `state_timeout_ms` | `100.0` |
| `left_base_frame` | `l/base_link` |
| `left_tcp_frame` | `l/link_6` |
| `right_base_frame` | `r/base_link` |
| `right_tcp_frame` | `r/link_6` |
| `translation_scale_left` | `1.0` |
| `translation_scale_right` | `1.0` |
| `left_base_from_pika_quaternion_xyzw` | `[0.70710678, 0.0, -0.70710678, 0.0]` |
| `right_base_from_pika_quaternion_xyzw` | `[0.70710678, 0.0, -0.70710678, 0.0]` |
| `velocity_filter_cutoff_hz` | `10.0` |
| `velocity_max_dt_ms` | `50.0` |
| `tf_lookup_timeout_ms` | `200.0` |
| `left_gripper_closed_position` | `0.0` |
| `left_gripper_open_position` | `0.1` |
| `right_gripper_closed_position` | `0.0` |
| `right_gripper_open_position` | `0.1` |

## 构建与运行

```bash
cd /home/lei/pika_teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/lei/pika_teleop_ws/install/setup.bash
```

运行 Mapper：

```bash
source /home/lei/pika_teleop_ws/install/setup.bash
ros2 run pika_realman_mapper pika_realman_mapper
```

Mapper 不提供 START Service。真实联调时由同事 RealMan Receiver 提供 `/pika_teleop/{left,right}/set_enabled`；本机纯 ROS 验收可先运行现有 Virtual Receiver：

```bash
source /home/lei/pika_teleop_ws/install/setup.bash
ros2 run pika_teleop_virtual_receiver virtual_receiver
```

推荐本机顺序：官方 Pika 节点 → Virtual Receiver（Service Server）→ Teleop Publisher → TF broadcaster/机器人状态发布 → Mapper。Mapper 启动后、START 前不会持续发布有效 command。

## 手动验收

先只观察 ROS 数据，不连接真实 SDK：

1. 用 `ros2 topic list | grep '^/pika/[lr]/'` 确认六个 l/r Topic 存在且不存在 `/pika/m/*`。
2. START 前用 `ros2 topic hz` 确认 command 没有持续刷新。
3. 双击 START；日志应显示 Pika/RM 起点，第一条 target 接近当前 RealMan TCP Pose。
4. Pika 向前/左/下移动，分别检查 Base `-X/-Y/-Z`；移动 5 cm、scale=1 时 target 约变化 0.05 m。
5. 返回 Pika START，target 应回到 RM START；重复来回不得累计漂移。
6. 绕目标轴旋转，观察 quaternion 和 angular velocity 连续；`q/-q` 不得造成 360° 突跳。
7. 开合左右夹爪，确认 percentage 连续且 clamp 在 `[0,1]`，左右互不影响。
8. 三击 STOP，确认该侧三个 command Topic 立即停止刷新，另一侧不受影响。
9. 再次 START，确认重新锁存当前 Pika Pose 和当前 RealMan TF，不复用上次起点。
10. ACTIVE 时停止 Publisher，约 100 ms 后应看到 watchdog warning 且 command 停止；恢复后先让该侧回到 disabled，再重新 START。
11. TF 不可用时应只看到节流 warning、无 command；TF 恢复后才建立 reference。

真实机械臂接入前仍必须确认：左右 Pika→Base 实际轴方向、左右夹爪开闭标定、跨机器 ROS 2 DDS 通信，以及同事端 command watchdog/SDK 实时下发。首次真实动作必须低速、空载并保留急停条件。
