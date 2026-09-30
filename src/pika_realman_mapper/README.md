# Pika RealMan Mapper

`pika_realman_mapper` 把左右 `PikaTeleopState` 映射为 RealMan 笛卡尔目标。速度是主要控制接口；Pose Topic 继续保留，供兼容和调试使用。

## 坐标流程

节点启动后，用左右 Pika 的一段稳定同步数据建立一个共用 Pika 基准：

- 原点：左右 Pika 原点的中点。
- Y 正方向：左 Pika 原点指向右 Pika 原点。
- Z 正方向：竖直向上，依据官方 Pika `base_link` 的 Z 轴。
- X 正方向：由 `X = Y × Z` 得到，安装正确时指向后方。

这个基准只在节点启动后标定一次。左右输入先转换到该基准，手柄自身姿态不会旋转线速度坐标轴。

每次双击启用某一侧遥操时，正式模式读取一次 TF：

```text
l/base_link <- l/link_6
r/base_link <- r/link_6
```

节点把该侧当前 Pika 原点与当前 RealMan TCP 原点重合，并发布动态虚拟 TF：

```text
l/base_link -> l_pika_base_link
r/base_link -> r_pika_base_link
```

虚拟坐标系的轴与 RealMan 基座轴平行。TF 只用于建立启动原点，运行中不持续读取 TCP 姿态。正式模式缺少对应 TF 时，该侧保持等待，不建立遥操 session；Bag 模式使用配置中的默认 TCP 位姿作为回退。

## 位置、姿态与速度

位置始终相对本次双击时的零位：

```text
Pika 当前位置 - Pika 启动位置
-> translation_scale
-> RealMan 启动 TCP 位置
```

位置差已经在与 RealMan 基座对齐的共用 Pika 基准中表达，不再乘手柄启动姿态，也不再增加固定的 90 度坐标变换。

线速度使用最近 `velocity_derivative_window_samples` 个位置样本的首尾差求导。角速度使用同一窗口内 Pika 姿态的空间旋转变化求导，结果直接表达在对齐后的 RealMan 基座轴上。手柄当前姿态不会改变速度坐标轴；手柄真实的转动仍会产生相应的角速度。

原始六轴速度依次经过独立的一维卡尔曼滤波、一阶低通滤波和死区。死区触发时对应滤波状态归零，因此持续小速度不会逐帧累计越过死区。角速度单位为 `rad/s`。

`orientation_mapping_mode` 只控制兼容用 `/pika/l|r/cartesian_pose` 的姿态字段；它不改变速度坐标轴。`left/right_base_from_pika_quaternion_xyzw` 也只保留给该 Pose 姿态映射。

## 接口

输入：

- `/pika_teleop/left/state`
- `/pika_teleop/right/state`
- 正式模式下的 `/tf`、`/tf_static`

输出：

- `/pika/l|r/cartesian_pose`，`geometry_msgs/msg/PoseStamped`
- `/pika/l|r/cartesian_velocity`，`geometry_msgs/msg/TwistStamped`
- `/pika/l|r/gripper_percentage`，`std_msgs/msg/Float32`
- `l_pika_base_link`、`r_pika_base_link` 虚拟 TF

现有 Pose 和速度 Topic 的 `frame_id` 仍分别由 `left/right_base_frame` 与 `left/right_velocity_frame` 指定，本次坐标标定不会重命名它们。

## 主要配置

- `require_realman_start_tf`：正式模式为 `true`；Bag 模式为 `false`。
- `left/right_tcp_frame`：启动时读取的 RealMan TCP TF 名称。
- `left/right_pika_base_frame`：发布的左右虚拟 Pika 基准名称。
- `pika_base_calibration_stable_samples`：启动标定所需连续样本数。
- `pika_base_calibration_max_spread_m`：标定窗口内允许的最大手柄位置波动。
- `pika_base_min_controller_separation_m`：左右 Pika 的最小水平间距。
- `pika_base_pair_max_stamp_delta_ms`：左右样本允许的最大时间差。
- `pika_vertical_axis_in_teleop_frame_xyz`：官方 Pika 竖直轴在 Bridge 输出坐标中的方向。
- `translation_scale_left/right`：位置变化和线速度的缩放比例。
- `velocity_derivative_window_samples`：线速度和角速度共同使用的求导窗口。
- `velocity_filter_cutoff_hz`、`velocity_kalman_*`：速度滤波参数。
- `linear_velocity_deadband_mps`、`angular_velocity_deadband_radps`：线速度和角速度死区。

共享配置位于：

```text
~/pika_teleop_ws/src/pika_teleop_bringup/config/ros/pika_config.yam
~/pika_teleop_ws/src/pika_teleop_bringup/config/ros/pika_bag_config.yam
```

## 运行

```bash
cd ~/pika_teleop_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pika_teleop_bringup pika_teleop.launch.py
```

真实动作前应低速、空载逐轴检查 X/Y/Z 与旋转方向。
