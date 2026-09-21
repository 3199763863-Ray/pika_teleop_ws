# Pika Teleop Bridge

Pika Teleop Bridge 是与具体机器人无关的 ROS 2 输入桥。它读取 Pika 官方双 Sense 的四路异步数据，执行 freshness / 数值 / 位姿跳变保护、固定坐标系转换和速度估计，以 100 Hz 发布左右独立的 `PikaTeleopState`，并通过异步 Service 与 Session Manager 握手启停。

本项目不包含 RealMan SDK、机器人 TCP、IK/FK、工作空间限制、相对位姿映射、Camera、Dataset 或数据录制逻辑。本版本不录 rosbag，运行时不会自动保存采集数据。

## 工程与源码树

工作区固定为：

```text
/home/lei/pika_teleop_ws
```

```text
/home/lei/pika_teleop_ws/src/
├── pika_teleop_interfaces/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── msg/
│   │   └── PikaTeleopState.msg
│   └── srv/
│       └── SetTeleopEnabled.srv
└── pika_teleop_bridge/
    ├── README.md
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/pika_teleop_bridge
    └── pika_teleop_bridge/
        ├── __init__.py
        ├── pika_input.py
        ├── gesture.py
        ├── safety.py
        ├── frame_transform.py
        ├── velocity.py
        └── node.py
```

- `pika_teleop_interfaces`：`ament_cmake` 接口包，生成 state message 和启停 service。
- `pika_teleop_bridge`：`ament_python` 运行包，正式 executable 为 `pika_teleop_publisher`。

## 最终架构与数据流

```text
Pika 官方节点
  ├── /pika_pose_l
  ├── /pika_pose_r
  ├── /gripper_l/joint_state
  └── /gripper_r/joint_state
             │
             ▼
      PikaInputAdapter
  latest cache / source stamp
  receipt time / age / sequence
             │
             ▼
       100 Hz control tick
             │
      ┌──────┴──────┐
      ▼             ▼
  left pipeline  right pipeline
  stale/finite   stale/finite
  PoseJumpGuard  PoseJumpGuard
  frame transform
  VelocityEstimator + LPF
      │             │
      ▼             ▼
 /left/state    /right/state

启停链路（左右本地状态独立，共享全局 episode）：

双击 → 检查 session gate → PENDING_START → async enable service → success → ACTIVE
三击/stale/jump → 本地立即 IDLE → async disable service
Session Manager force_stop_all → 左右 ACTIVE/PENDING_START 全部本地 IDLE
```

左右两侧拥有各自的 `GestureDetector`、`PoseJumpGuard`、`VelocityEstimator`、状态机、Service client 和 timeout。一侧等待、停止或异常不会改变另一侧。

## 类与职责

- `PikaInputAdapter`：用 depth 1 的四个订阅缓存 latest value；callback 只更新时间、消息引用和 sequence，不做手势或数学运算。`get_snapshot()` 非阻塞返回四路 `TimedSample`。
- `GestureDetector`：以滞回阈值识别完整“开→合→开”；IDLE 双击申请 START，ACTIVE 三击执行 STOP。
- `PoseJumpGuard`：只在 ACTIVE 且收到新 Pose 时比较连续 raw Pose；固定旋转不改变距离和最短旋转角，因此安全检查在 frame transform 前执行。
- `PikaFrameTransform`：统一转换 position、orientation 和向量到 `pika_teleop_frame`。
- `VelocityEstimator`：每侧一个，只消费新的、已转换 Pose，按 source timestamp 估计 Twist 并执行一阶低通。
- `PikaTeleopPublisher`：组合以上组件，维护状态机、异步 Service future、安全停止和 100 Hz state 发布。

## ROS 接口

### 输入 Topic

| Topic | Type | 输入 QoS |
|---|---|---|
| `/pika_pose_l` | `geometry_msgs/msg/PoseStamped` | RELIABLE, VOLATILE, depth 1 |
| `/pika_pose_r` | `geometry_msgs/msg/PoseStamped` | RELIABLE, VOLATILE, depth 1 |
| `/gripper_l/joint_state` | `sensor_msgs/msg/JointState` | RELIABLE, VOLATILE, depth 1 |
| `/gripper_r/joint_state` | `sensor_msgs/msg/JointState` | RELIABLE, VOLATILE, depth 1 |

### 输出 State

| Topic | Type | QoS / rate |
|---|---|---|
| `/pika_teleop/left/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, depth 1, 100 Hz |
| `/pika_teleop/right/state` | `pika_teleop_interfaces/msg/PikaTeleopState` | BEST_EFFORT, VOLATILE, depth 1, 100 Hz |

```text
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float64 gripper_position

bool enabled
bool valid
bool velocity_valid

builtin_interfaces/Time pose_source_stamp
builtin_interfaces/Time gripper_source_stamp

float32 pose_age_ms
float32 gripper_age_ms
```

- `header.stamp` 是 bridge control tick 时间；`header.frame_id` 固定为 `pika_teleop_frame`。
- `pose` 始终是 Teleop frame 下的绝对 Pika Pose，不在 START 时归零。
- `enabled` 仅在该侧为 ACTIVE 时为 true。
- `valid` 仅在 `enabled=true` 且 Pose/Gripper 当前均可用时为 true。
- `velocity_valid` 仅在当前 session 已得到两个合法的新 Pose sample 并成功估速后为 true；否则 Twist 为零。
- `pose_source_stamp` / `gripper_source_stamp` 保留官方消息时间；age 以 source stamp 为主，无有效 source stamp 时回退 receipt time。

### 启停 Service

Bridge 是 Service Client；正式运行由 `pika_session_manager` 实现 Service Server：

| Service | Type |
|---|---|
| `/pika_teleop/left/set_enabled` | `pika_teleop_interfaces/srv/SetTeleopEnabled` |
| `/pika_teleop/right/set_enabled` | `pika_teleop_interfaces/srv/SetTeleopEnabled` |

```text
bool enable
string reason
---
bool success
string message
```

正式请求：

- START：`enable=true, reason="USER_START"`
- 用户停止：`enable=false, reason="USER_STOP"`
- stale 停止：`enable=false, reason="STALE_STOP"`
- 位姿跳变停止：`enable=false, reason="POSE_JUMP_STOP"`

启停只通过上述两个 Service 完成，不保留重复的 Topic 通道。

Bridge 还订阅：

- `/pika_session/start_allowed`：gate=false 时只阻止新的 IDLE→START，不取消已进入 PENDING_START 的请求。
- `/pika_session/force_stop_all`：不递归调用 Service，直接清除左右 ACTIVE/PENDING_START、手势、位姿保护和速度状态。

## 单侧状态机与安全语义

```text
IDLE
  └── 双击且数据 usable
        └── call_async(enable=true, USER_START)
              └── PENDING_START
                    ├── success + 数据仍 usable → ACTIVE
                    ├── success + 数据已失效 → IDLE + async STALE_STOP
                    └── failure / timeout → IDLE

ACTIVE
  ├── 三击 → 本地 IDLE + async USER_STOP
  ├── missing/non-finite/stale → 本地 IDLE + async STALE_STOP
  └── pose jump → 本地 IDLE + async POSE_JUMP_STOP
```

`PENDING_START` 期间 timer 不等待 future，state 仍以 100 Hz 发布，但 `enabled=false, valid=false`。只有 response 的 `success=true` 且 response 到来时数据仍 usable，才进入 ACTIVE。

STOP 的顺序固定为：先将本地 mode 设为 IDLE，立即清除 enabled/valid，并 reset gesture、guard、velocity；再发异步 disable 请求。Service 失败不会重新启用本地状态。

下游不能只依赖 Service，必须同时以三层门控控制运动：

1. 仅处理 `state.enabled && state.valid` 的运动目标；
2. `valid=false` 时不再执行新运动目标；
3. state 接收超时必须立即停止遥操；收到 disable Service 后明确停止并清理 session。

Service 是启停握手，state 标志和下游 watchdog 是断线、延迟 response 及 Service 异常时的安全兜底。

## 坐标系转换

原始 Pika：`+X_old` 向夹爪前方、`+Y_old` 向左、`+Z_old` 向上。输出 Teleop：`+X_new` 向下、`+Y_new` 向左、`+Z_new` 向夹爪前方，保持右手系。

```text
x_new = -z_old
y_new =  y_old
z_new =  x_old
```

采用固定旋转：

```text
C = [[0, 0, -1],
     [0, 1,  0],
     [1, 0,  0]] = R_y(-90°)

p_new = C p_old
R_new = C R_old C⁻¹
q_new = q_C ⊗ q_old ⊗ q_C⁻¹
q_C = (0, -√0.5, 0, √0.5)   # x,y,z,w
```

因此 old `+X/+Y/+Z` 分别映射到 new `+Z/+Y/-X`。Orientation 使用固定旋转共轭，不做四元数分量重排。速度由转换后的连续 Pose 计算，因此线速度和角速度也表达在 `pika_teleop_frame`。

## 速度估计

速度只在 `pose.new_sample=true` 时更新；100 Hz tick 没有新 Pose 时保留最近的 filtered Twist，不对同一 Pose 重复求导。

```text
dt = current_pose_source_stamp - previous_pose_source_stamp
v_raw = (p_current - p_previous) / dt

q_delta = inverse(q_previous) ⊗ q_current
omega_raw = shortest_axis * shortest_angle / dt
```

估计器先处理 `q/-q` 等价并选择最短旋转，然后对六个 Twist 分量执行因果一阶低通：

```text
tau = 1 / (2*pi*velocity_filter_cutoff_hz)
alpha = dt / (tau + dt)
filtered = previous_filtered + alpha * (raw - previous_filtered)
```

第一帧、stamp 无效、`dt<=0`、`dt>velocity_max_dt_ms`、Pose 非有限时，输出零 Twist 且 `velocity_valid=false`。timestamp 异常、START、USER_STOP、STALE_STOP 和 POSE_JUMP_STOP 都会 reset estimator；START 后第一条新 Pose 只建立 baseline，第二条合法新 Pose 才产生有效速度。

## Pose jump 与 freshness

- usable 要求 Pose 和 Gripper 均存在、有限，四元数可归一化，并且两路 age 均位于 `[0, stale_stop_ms]`。
- ACTIVE 时 stale/非法检查先于 Pose jump。
- Pose jump 只检查 raw Pika 的新 Pose：位置欧氏距离大于 `max_position_jump_m`，或归一化四元数最短角大于 `max_rotation_jump_deg`，即停止该侧。
- `q` 与 `-q` 通过四元数点积绝对值处理，不会产生假的 360° 跳变。

## 100 Hz tick 的实际顺序

每次 timer：

1. 获取同一 control time 下的四路 latest snapshot；
2. 分别计算左右 usable；
3. 轮询该侧 START future / timeout，并在成功 response 时重新检查 usable；
4. ACTIVE 时依次做 stale/非法检查和 raw Pose jump 检查；
5. 将当前合法 raw Pose 转到 `pika_teleop_frame`；
6. ACTIVE、usable 且为新 Pose 时更新对应 `VelocityEstimator`；
7. 构造并发布左右 state；
8. state 发布后处理新的 gripper gesture；因此完成三击的样本仍作为正常 ACTIVE 数据发布，下一 tick 显示 disabled；
9. 非阻塞检查 disable Service future 的结果。

## 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `state_rate_hz` | `100.0` | state 发布/控制 tick 频率 |
| `stale_stop_ms` | `50.0` | Pose 或 Gripper 最大 age |
| `gripper_open_threshold` | `0.075` | open 滞回阈值 |
| `gripper_close_threshold` | `0.025` | close 滞回阈值 |
| `click_max_interval_ms` | `450.0` | 单击闭合时长及连续 click 最大间隔 |
| `gesture_reset_timeout_ms` | `1200.0` | 未完成 gesture 重置时间 |
| `max_position_jump_m` | `0.08` | 连续新 Pose 最大位置跳变 |
| `max_rotation_jump_deg` | `45.0` | 连续新 Pose 最大旋转跳变 |
| `start_service_timeout_ms` | `10000.0` | 等待 Session Manager 完成 Recorder START 的 response 超时 |
| `use_session_gate` | `true` | 是否在新 START 前检查 Session Manager gate |
| `velocity_filter_cutoff_hz` | `10.0` | Twist 一阶低通截止频率 |
| `velocity_max_dt_ms` | `50.0` | 连续 Pose 最大合法 source dt |

所有时间/频率/跳变参数必须是有限正数；夹爪 open 阈值必须大于 close 阈值。

## 构建与启动

构建：

```bash
cd /home/lei/pika_teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/lei/pika_teleop_ws/install/setup.bash
```

终端 1，启动官方双 Sense 采集节点（已启动时不要重复启动）：

```bash
source ~/pika_ros/install/setup.bash
cd ~/pika_ros/scripts
bash start_multi_sensor_whit_teleop.bash
```

终端 2，推荐用正式 bringup 启动本地核心节点：

```bash
source /opt/ros/humble/setup.bash
source /home/lei/pika_teleop_ws/install/setup.bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
```

Session Manager 及远端 Recorder 应在双击 START 前就绪。若 Server 不存在，双击后该侧保持 `PENDING_START`，state 持续 disabled，并在 `start_service_timeout_ms` 后回到 IDLE。正式 launch 不要与 Virtual Receiver 同时运行。

## 验收查看命令

每个新终端先 source 新工作区，否则 CLI 可能报 `The passed message type is invalid`：

```bash
source /home/lei/pika_teleop_ws/install/setup.bash
```

查看 state（推荐不手写类型）：

```bash
ros2 topic echo /pika_teleop/left/state \
  --qos-reliability best_effort \
  --qos-durability volatile

ros2 topic echo /pika_teleop/right/state \
  --qos-reliability best_effort \
  --qos-durability volatile
```

查看接口和频率：

```bash
ros2 interface show pika_teleop_interfaces/msg/PikaTeleopState
ros2 interface show pika_teleop_interfaces/srv/SetTeleopEnabled
ros2 topic hz /pika_teleop/left/state
ros2 topic hz /pika_teleop/right/state
ros2 service type /pika_teleop/left/set_enabled
ros2 service type /pika_teleop/right/set_enabled
```

Service request 不能用 `ros2 topic echo` 监听；应由下游 Server 日志确认收到的 `enable/reason`，同时观察 state 的 `enabled/valid/velocity_valid`。

## START 零点与下游映射

Bridge 始终发布 Teleop frame 下的绝对 Pose。Mapper 收到第一条 `enabled=true && valid=true` state 后，同时保存：

```text
Pika_start_pose
配置中的 RealMan default TCP pose
```

之后由 Mapper 计算相对位姿和机器人目标；不要把第一条 Pika 绝对 Pose 直接作为机器人绝对目标。

## 当前边界

Bridge 只负责标准化 Teleop state、手势与 Sense 侧安全，不包含 Recorder、RealMan Action 或 SDK 逻辑。正式 Service Server 由 `pika_session_manager` 提供，相对位姿由 `pika_realman_mapper` 处理，远端 Receiver 仍负责 command watchdog、限速、工作空间与机械臂安全。

本地节点不录 rosbag。正式示教数据由远端 Recorder 保存，保存位置由 Recorder 的 profile/配置决定。
