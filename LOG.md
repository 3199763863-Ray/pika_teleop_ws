# 项目维护记录

## 2026-10-08：文档版本同步与备份整理

**依据**：GitHub 与部署机 `/home/user2/pika_teleop_ws` 均为 `main`，执行时 HEAD `e6d8d40e9f66bde4d8b49f0440843f3cff58eade`。先读取现行源码、接口、两份 launch 与 YAML，再修文档。部署机原有一项未提交工作：`pika_bag_config.yam` 中 Virtual Receiver `state_timeout_ms` 从已提交的 `200.0` 改为 `2000.0`；本任务未改该文件，也未切换/重置分支。

### 修正的主要旧描述

- 将现行 Bridge State 与 Mapper Pose/速度写为 **20 Hz**，夹爪命令单独为最多 **4 Hz**；100 Hz 仅作为代码默认值说明。
- 将正式 Bridge `stale_stop_ms` 写为代码默认 **50 ms**、Bag YAML **2000 ms**；Mapper watchdog 为 **200 ms**。Bag Virtual Receiver 在已提交 YAML 中为 **200 ms**，部署机未提交覆盖后为 **2000 ms**。
- 修正 Mapper 当前 5 帧求导、卡尔曼与低通、线速度 `0.005 m/s` / 角速度 `0.012 rad/s` 死区、夹爪全开值 `0.0967`。Bridge PoseJump 默认 `0.08 m` / `45°`。
- 解释 Bridge 固定预变换、Mapper 双 Pika 共用基准、每侧 START 时 RealMan TCP TF、Bag 默认 TCP 回退，以及 Pose 与速度分开的 `frame_id`；删除重复做固定 90° 旋转和始终使用固定 TCP 的旧说法。
- 说明正式录制由外部 Recorder 落盘、真机命令由外部接收器执行。正常 USER_STOP 后才允许外部双臂 MoveJ；STALE_STOP/POSE_JUMP_STOP 进入 FAILED 且不自动回零。
- 将此机器地址标为 **2026-10-08 的部署示例** `192.168.31.97`，移除过期提交/分支领先关系的永久现状描述。

### 修改文件

- `README.md`：重新整理项目入口、八个 Package、两模式、坐标与参数、构建、检查和外部边界。
- `src/pika_teleop_bridge/README.md`：同步 20 Hz State、QoS、预变换、手势与两模式 stale。
- `src/pika_realman_mapper/README.md`：同步共享基准、TF 起点、速度计算、输出 frame 与参数。
- `src/pika_session_manager/README.md`：同步 Recorder/Action 状态机、异常停止与人工恢复。
- `src/pika_teleop_virtual_receiver/README.md`：同步 watchdog、日志策略、已提交值与部署机覆盖值。
- `src/pika_teleop_bringup/README.md`：同步 launch、supervisor、脚本、日志链接和 Bag 手动录制。
- `LOG.md`：新增本次审计记录。仓库原无统一 LOG。

### 备份审查与处理

以下 **11 个 Git 跟踪文件**均是同目录正式文件的旧快照：正式文件存在，名称无源码、launch、测试或文档引用；旧内容仍可从 Git 历史恢复。本次只对这些文件标记删除，没有清理未跟踪文件、用户数据、官方 Pika 工作区、ROS 日志或 rosbag：

1. `src/pika_realman_mapper/README.md.bak-20260930`
2. `src/pika_realman_mapper/pika_realman_mapper/node.py.bak-20260930`
3. `src/pika_realman_mapper/pika_realman_mapper/node.py.bak-frame`
4. `src/pika_realman_mapper/pika_realman_mapper/pose_mapper.py.deadbak`
5. `src/pika_realman_mapper/pika_realman_mapper/velocity.py.dbak`
6. `src/pika_teleop_bridge/pika_teleop_bridge/node.py.bak-20260924_151016`
7. `src/pika_teleop_bringup/config/ros/pika_bag_config.yam.bak-frame`
8. `src/pika_teleop_bringup/config/ros/pika_bag_config.yam.bak250`
9. `src/pika_teleop_bringup/config/ros/pika_config.yam.bak-frame`
10. `src/pika_teleop_bringup/launch/pika_bag.launch.py.bak-20260930`
11. `src/pika_teleop_bringup/launch/pika_teleop.launch.py.bak-20260930`

未增加宽泛的 `.gitignore` 规则，避免误伤真正的示例或 fixture。无需要保留但无法判断的备份候选。

### 检查与范围

- `git diff --check`：通过，未发现空白错误；Windows Git 显示换行转换提示，不影响差异内容。
- 旧值检索：命中 `100 Hz`、`0.02 m/s`、`0.03 rad/s` 仅用于**代码默认值**与现行 YAML 对照，不再作为现行运行值。
- 文档相对链接、被引用的路径/launch/脚本与 11 个备份引用：已静态核对；无剩余备份使用处。
- 修改范围：六份 README、此 LOG 和上述 11 个旧备份；未修改节点、接口、YAML 参数、launch 或脚本。未启动设备、Recorder 或真机；本次不声称 ROS2 实机测试通过。
- 未 commit、未 push。部署机原有未提交 Bag watchdog 覆盖保持原样。

### 发现但未处理的问题 / 后续实机确认

- Mapper 假设由左右 Pika 与竖直轴构成的共享基准与 RealMan 基座命令轴方向一致，但没有用 RealMan TF 旋转标定该方向。速度数值使用此基准，却标记为 `l/work/pikabase` / `r/work/pikabase`；外部接收器对这些 frame 的解释需要现场逐轴确认。
- 正式 TCP 起点使用 TF 缓冲区的最新可用变换；当前代码没有单独的 TF 最大年龄检查。外部 TF 停止更新时需确认实际行为。
- 正式 Bridge 50 ms stale 阈值与 20 Hz State 周期同量级；在官方位姿卡顿或网络延迟时可能频繁触发停机。已有部署反馈称 Pika 位姿读取卡顿和 Wi-Fi 问题仍未定位。本次只记录，不改保护值。
- 外部 Recorder 的实际落盘、相机数据、RealMan 真机动作、速度方向、急停、回零与长期稳定性均需在现场另行验证。

## 2026-10-08：Bag 正常停止后的单侧 MoveJ Goal

**基线**：本次从干净的 `main` `1f06d5d36c6b3b30979e1a562f57ed5b030ba0a0` 独立检出。该提交的 Bag YAML 已包含 Virtual Receiver `state_timeout_ms=2000.0`；上节记录的“未提交 200→2000”是当时的历史状态，本次未改写旧记录。正式配置 `pika_config.yam`、Session Manager、Bridge、Mapper 均未修改。

### 实现与文件

- `receiver_node.py`、新增 `reset_handoff.py`：Virtual Receiver 只在 `reset_on_user_stop=true` 时建立 Action Client。每侧独立记录本轮已确认 ACTIVE；仅已确认 ACTIVE 的 `USER_STOP` 建立待复位，等待请求之后的新 disabled State，再非阻塞延迟派发单侧 MoveJ。确认超时、矛盾 State、watchdog 或异常 STOP 取消；重复 STOP 不重复发送。Action 不可用、发送失败或拒绝只记录告警，STOP 仍回执成功。只观察 Goal 接受/拒绝，**不获取 Action Result，也不判断物理到位**。
- `pika_bag_config.yam`：将左右复位 Action 名、六关节角和 Goal 参数从正式 YAML 的当前值复制为 Bag 独立配置，并启用功能；保留原有 2000 ms State watchdog 与遥操频率等值。`reset_on_user_stop=false` 可退回原 Bag 行为。
- `pika_bag.launch.py`、Virtual Receiver `package.xml`：补齐 Bag 运行时 `realman_msgs` 的 Python 路径与依赖。`setup.py` 和新增 `test/`：接入状态交接与假 Action Server 测试。
- 根 README、Virtual Receiver README、Bringup README：说明触发条件、单侧行为、配置、人工确认、外部 Action Server 及现场验证步骤；将 Bag watchdog 现行值统一修正为 2000 ms。

### 已执行检查

- Windows 本地 `python -m unittest discover -s src/pika_teleop_virtual_receiver/test -p 'test_*.py' -v`：5 项纯逻辑测试通过；6 项 ROS 集成测试因本机没有 ROS Python 包而跳过。
- 部署机仅在 `/tmp/pika_bag_reset_test_20261008` 的**隔离测试工作区**执行 `colcon build --symlink-install --packages-up-to pika_teleop_bringup`：8 个包构建通过；`realman_msgs.action.ExecuteMotion` 导入通过。另用 Bag launch 的 `_local_python_path(...)` 生成子进程环境，Action 类型导入通过。
- 同一隔离目录执行 `colcon test --packages-select pika_teleop_virtual_receiver`：11 项通过，包含独立 ROS 域中的假 Action Server 接受且不结束动作、服务端缺失、拒绝 Goal、发送异常、关闭开关和非法关节配置校验。`colcon test-result --verbose` 读到 0 项，因为该包的 unittest 运行器没有生成 JUnit XML；测试实际结果见 `colcon test` 输出。
- `git diff --check` 通过；静态检索确认 Virtual Receiver 未调用 `get_result_async()`、同步 Action 等待或阻塞 `sleep`。

### 限制与风险

- 未启动官方 Pika、Recorder 或真机机械臂；未验证预设关节角的实际安全性、外部 Action Server 的响应和物理到位。临时测试代码没有部署到 `/home/user2/pika_teleop_ws`。
- 新 disabled State 加交接延迟仅降低与实时速度发布的竞态，不能原子保证外部 RealMan 接收器已停速。Goal 发出后不等待执行结果，用户必须现场确认本侧 MoveJ 完成后才再次 START；下游仍须提供互斥、限位和急停。
- 本次没有提交或推送；工作区保留改动供人工审查。

## 2026-10-09：Bag 复位派发延迟试验

- 现场三次右侧 `USER_STOP` 均成功下发 MoveJ Goal，但 RealMan 驱动拒绝，工控机日志为 `Rejecting motion goal: arm r is busy`。行为树 Pika 速度 router 不接收 `USER_STOP`，只在速度输入停止 3000 ms 后开始取消速度 Action；原 Bag 交接延迟 150 ms 早于右臂释放。
- 将 Bag YAML 的可调项 `reset_dispatch_delay_ms` 设为 **4000.0 ms**，从新 disabled State 被确认时开始计时；正式 YAML 和 Virtual Receiver 的独立运行代码默认值不变。同步更新三个 README 的当前值。
- 这是等待外部 router 超时的现场试验值，不是释放右臂所有权的确认。若其他控制端仍占用右臂，或取消耗时超过余量，MoveJ 仍可能被拒绝；长期方案需明确取消速度 Action 并确认释放后再归位。
