# Pika RealMan Mapper

`pika_realman_mapper` 把左右 `PikaTeleopState` 映射为 RealMan 笛卡尔目标。位置始终采用相对启动零位映射，姿态可选绝对或相对模式。节点不访问 RealMan TF、不管理录制、不调用机械臂 Action/SDK，也不提供启停 Service。

## 位姿模式

位置始终按启动零位计算：

```text
delta_p_fixed = p_current - p_start
delta_p_start_frame = inverse(R_pika_start) * delta_p_fixed
delta_p_base = R_rm_start * scale * R_base_from_pika * delta_p_start_frame
p_target = p_rm_default + delta_p_base
```

位置差会先转入启动时的 Pika 局部坐标系，再映射到 RealMan 起始 TCP 轴，最后转入机械臂基坐标系。
Pose 保持相对启动零位；原地旋转手柄不产生位置变化。当前 TCP 姿态是映射目标姿态，节点不读取机械臂实测 TF。

姿态由 `orientation_mapping_mode` 选择。正式和 Bag 配置当前使用相对姿态。
启动时记录完整的 Pika 位姿和默认 TCP 位姿，后续相对变化均按固定起点计算，不逐帧累计：

```text
q_delta_pika = inverse(q_start) * q_current
q_delta_base = q_map * q_delta_pika * inverse(q_map)
q_target = q_rm_default * q_delta_base
```

Bridge 已按目标坐标关系将原始 Pika 坐标变为 `(-z, y, x)`：Pika +X
对应 RealMan +Z，Pika -Z 对应 RealMan +X，Y 轴不变。Mapper 的默认
`q_map = [0.0, 0.0, 0.0, 1.0]`（xyzw）为单位旋转，避免重复变换。

默认 TCP 位姿与全部运行参数来自：

```text
~/pika_teleop_ws/src/pika_teleop_bringup/config/ros/pika_config.yam
```

四元数统一使用 `xyzw`，节点启动时会检查有限值并归一化默认姿态。缺少默认 TCP 数组会直接启动失败，避免退回隐含零位。

## 速度平滑

目标速度使用最近 `velocity_derivative_window_samples` 个目标 Pose 的首尾变化在机械臂基坐标系中求导。
滤波和死区处理完成后，线速度与角速度按最新目标姿态转入当前 TCP 坐标系。
当前 Pika 自身 +X、+Y、-Z 对应当前 RealMan TCP +Z、+Y、+X；转动 Pika 后仍保持该关系。
该轴对应关系适用于当前 `relative` 姿态模式；方向快速变化时，求导窗口和滤波会有短暂的响应滞后。
两种速度均以 TCP 原点为作用点，原地旋转不会产生伪线速度。窗口未填满时输出 0；过短的样本间隔会被忽略，超时则清空窗口并重新建立基线。

原始六轴速度依次经过独立的一维卡尔曼滤波和一阶低通滤波。正式与 Bag 配置默认启用卡尔曼滤波，可通过 `velocity_kalman_enabled` 关闭。线速度和角速度分别提供过程方差、测量方差参数，随后再应用各自死区；死区触发时对应卡尔曼状态也会归零，持续小速度不会跨帧累计越过死区。

## 接口

输入：

- `/pika_teleop/left/state`
- `/pika_teleop/right/state`

输出：

- `/pika/l|r/cartesian_pose`，`geometry_msgs/msg/PoseStamped`
- `/pika/l|r/cartesian_velocity`，`geometry_msgs/msg/TwistStamped`；`frame_id` 分别由 `left_velocity_frame/right_velocity_frame` 指定，当前为 `l/link_6`、`r/link_6`
- `/pika/l|r/gripper_percentage`，`std_msgs/msg/Float32`

`cartesian_pose` 仍在配置的机械臂基坐标系表达。下游应按当前末端坐标系解释 `cartesian_velocity`，
不得继续把分量当作基坐标系速度。速度 frame 参数仅改变名称，不增加工具安装旋转；工具轴不同则需另行标定轴映射。

Bridge state 输入使用 `BEST_EFFORT + VOLATILE + KEEP_LAST depth=1`；六个 RealMan command 输出按照接收端接口约定使用 `RELIABLE + VOLATILE + KEEP_LAST depth=1`。两套 QoS 分开定义，避免把 RELIABLE 错误地应用到上游 BEST_EFFORT state 订阅。

输出仅在该侧 `enabled && valid`、state watchdog 正常且数值合法时发布。STOP、invalid 或 watchdog timeout 会清除 session reference 和速度状态并停止该侧输出。RealMan 接收端仍必须实现 command watchdog、限位、急停和 SDK 安全控制。

## 运行

推荐通过 bringup 启动，确保加载共享配置：

```bash
cd ~/pika_teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
```

真实动作前必须低速、空载逐轴确认 Pika 到左右 Base 的方向、默认 TCP 精度和夹爪标定。
