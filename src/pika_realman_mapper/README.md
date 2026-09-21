# Pika RealMan Mapper

`pika_realman_mapper` 把左右 `PikaTeleopState` 的 session 相对运动映射为 RealMan 笛卡尔目标。它不访问 RealMan TF、不管理录制、不调用机械臂 Action/SDK，也不提供启停 Service。

## 固定零位

每侧第一次收到 `enabled=true && valid=true` 时同时锁存：

```text
Pika_start_pose = 当前 state.pose
RM_start_pose   = 共享配置中的 default TCP pose
```

第一帧 target 因而等于配置的 default TCP pose。后续目标按固定起点计算，不逐帧累计：

```text
delta_p_base = scale * R_base_from_pika * (p_current - p_start)
p_target = p_rm_default + delta_p_base

q_delta_pika = q_current * inverse(q_start)
q_delta_base = q_map * q_delta_pika * inverse(q_map)
q_target = q_delta_base * q_rm_default
```

默认 TCP 位姿与全部运行参数来自：

```text
/home/lei/pika_teleop_ws/src/pika_teleop_bringup/config/ros/pika_config.yam
```

四元数统一使用 `xyzw`，节点启动时会检查有限值并归一化默认姿态。缺少默认 TCP 数组会直接启动失败，避免退回隐含零位。

## 接口

输入：

- `/pika_teleop/left/state`
- `/pika_teleop/right/state`

输出：

- `/pika/l|r/cartesian_pose`，`geometry_msgs/msg/PoseStamped`
- `/pika/l|r/cartesian_velocity`，`geometry_msgs/msg/TwistStamped`
- `/pika/l|r/gripper_percentage`，`std_msgs/msg/Float32`

输出仅在该侧 `enabled && valid`、state watchdog 正常且数值合法时发布。STOP、invalid 或 watchdog timeout 会清除 session reference 和速度状态并停止该侧输出。RealMan 接收端仍必须实现 command watchdog、限位、急停和 SDK 安全控制。

## 运行

推荐通过 bringup 启动，确保加载共享配置：

```bash
cd /home/lei/pika_teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
```

真实动作前必须低速、空载逐轴确认 Pika 到左右 Base 的方向、默认 TCP 精度和夹爪标定。
