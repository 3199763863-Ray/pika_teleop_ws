# 项目维护记录

## 2026-10-15：降低启动时 DDS 发现流量

- 在 `ROS_DOMAIN_ID=65` 下短暂启动 Bag、官方定位器、左右串口和脚踏节点，未触发 START 或机器人 Action。逐秒网卡计数复现启动后约 5 秒的发送峰值：有线约 6–10 MB/s、WiFi 约 7–24 MB/s，随后均降至每秒几百 KB。抓包只截取包头；约 30 万个有线包、61 万个 WiFi 包为 52 字节 DDS `HEARTBEAT`，属于参与者发现/匹配流量，不是图像数据。官方位姿和夹爪话题在待机时只有本机 Bridge 订阅，故降低其 100 Hz 采样不能解决该峰值。
- 复核发现，先前 UDP `interfaceWhiteList` 限制的是本地源地址，不能阻止源地址为 `192.168.30.143` 的包经 WiFi 路由发往 RealMan WiFi/WireGuard 地址；旧节标题和“仅使用有线”表述并不准确。
- Fast DDS 2.6 的临时 XML 将每个远端参与者的 `max_unicast_locators` 从默认 4 设为 1。相同启动场景下，WiFi 前几秒约 0.5 MB/s，有线仍约 10 MB/s；RealMan 只读订阅 `/gripper_l/joint_state` 实际收到了 Pika 官方数据。`/pika_pose_l` 当时无样本，因此不能据此断言定位数据路径和真机控制已经完整验证。
- 把该参数作为 Bag/正式 launch 的 `dds_remote_unicast_locators`，默认 1，可设 4 回到原地址数量。官方源码、Topic 频率、控制逻辑和 RealMan 工控机均未改。此配置的目的在于减少重复 DDS 包；如果 RealMan 网络拓扑变动，必须重新核对跨机通信。
- 同步到 `user2@192.168.31.97:/home/user2/pika_teleop_ws` 后，六个文件的 SHA-256 与本地一致，`colcon build --symlink-install --packages-select pika_teleop_bringup` 通过，Bag/正式 launch 的 `--show-args` 均显示默认值 1。再用已安装的新 Bag launch 启动官方节点实测，生成的 XML 确为 `max_unicast_locators=1`；启动阶段 WiFi 每秒约 0.54–0.56 MB，有线仍约 10 MB/s。测试后清理了 launch 与官方子进程。未执行 Git add/commit/push，也未进行真机 START/速度控制测试。
- 随后在相同 `ROS_DOMAIN_ID=65` 的 Bag+官方节点待机场景逐秒采样约 1 分钟：启动前 5 秒有线最高 10.24 MB/s、WiFi 最高 0.60 MB/s；第 11 秒后连续 55 个样本，有线平均 107 KB/s、最高 141 KB/s，WiFi 平均 33 KB/s、最高 52 KB/s（均为整机网卡发送计数，含后台流量）。无 USER_START、归位 Goal 或测试进程残留。此测量支持待机带宽稳定下降，不能替代真实遥操、正式模式或长期运行验证。

## 2026-10-14：Pika DDS 有线源地址白名单（后续发现仍有重复发送）

- 基线为部署机干净的 `main` `d6173ea100a64f0f1c1465f6b5be9c47daa817f8`，本地隔离工作副本 `outputs/wired_dds_repo`。Bag 空闲运行时曾测到 Pika WiFi 发送约 4 MB/s；包头统计显示官方 `pika_double_locator_node` 同时向 RealMan 的有线 `192.168.30.200`、WiFi `192.168.31.224`、Tailscale `100.97.41.73` 和 WireGuard `10.200.0.13` 地址发送 DDS 数据，后者从 Pika 部署机不可达。RealMan 工控机仅做只读检查。
- Bag 与正式 launch 新增默认开启的 `dds_wired_only` 和可覆盖的 `dds_wired_peer`（当前 `192.168.30.200`）。启动时根据 peer 路由读取本机物理有线网卡 IPv4，生成本次日志目录下的 Fast DDS XML；自定义 UDPv4 白名单只列该地址，另保留 SHM。本 launch 启动的自有节点及官方子进程继承 XML。不可用时直接报错；无有线环境可显式关闭。官方 Pika 代码、RealMan 工控机、Topic 频率及控制逻辑均未改。
- 启动时的网卡、IPv4、目标和 XML 路径会同时进入 `launch.log` 与 `run_info.txt`。当前用户暂不方便重启 Pika，本次不重启运行节点；**修改后的网络带宽仍待下次由用户启动后实测**，不能将修改前的 4 MB/s 当作修改后结果。
- 静态验证：部署机路由识别为 `enp131s0 / 192.168.30.143`，传入 WiFi peer 时明确拒绝；XML 解析后 UDP 白名单只有该有线地址，并保留 SHM。`colcon build --symlink-install --packages-select pika_teleop_bringup` 成功；安装模块可导入，Bag/正式 launch 的 `--show-args` 都显示新增参数；`git diff --check` 与 Python 语法检查通过。以上均未启动 Pika 或机械臂，未验证真实 DDS 发布后的带宽与通信连通性。
- 未执行 Git add/commit/push。下次由用户自行启动后，应核对官方定位进程继承的 `FASTRTPS_DEFAULT_PROFILES_FILE`，再分别采样有线/WiFi/Tailscale 的带宽及 ROS Topic、Service、Action 连通性。

## 2026-10-10：正式模式超时与 Recorder 解耦复位

基线为干净的 GitHub `main` `2b882ab37dd46354cf6aea12ebe12b308b8be414`，另建 `outputs/formal_reset_repo`，没有覆盖上一脚踏任务工作副本。部署机同为该提交，但正式 YAML 已有未提交的 Mapper `state_timeout_ms: 2000.0`（相对提交为 200.0）；本次保留此现场值并纳入本地工作副本，未改 Mapper 源码、TF、坐标映射、脚踏及 Bag YAML/Launch/Virtual Receiver。

- 正式 YAML：Bridge 显式设 `stale_stop_ms: 1000.0`；Session Manager 新增 `reset_on_user_stop: true`、`reset_dispatch_delay_ms: 4000.0`、`stop_state_timeout_ms: 6000.0`。后者用于新双侧 disabled State 缺失时保守失败。
- Session Manager：正常 STOP 立刻关闭双侧和 START gate，异步请求 Recorder STOP；只在本轮新双侧 disabled State 到达后计时派发左右各一次 MoveJ。Recorder 成功时从其回复和双侧确认的较晚时刻起等 4 秒；失败、无服务、异常或 4 秒内无回复时仍可从双侧确认起等 4 秒后归位，但结果保留 `RECORDER_STOP_FAILED/UNCONFIRMED`，即使 Action 成功也留在 FAILED。迟到回复仅记录，不再派发或自动 READY。State 缺失、异常停止、未真正启动的取消均不触发 MoveJ。
- 原有 Action Goal/Result 等待保留；新增发送异常和无结果超时处理，不在 busy 时自动重试。`reset_on_user_stop=false` 时仍请求 Recorder STOP，成功直接 PREPARING，失败或超时 FAILED。更新根、Session Manager、Bringup 与 Bridge README；新增正式模式假 Recorder/State/Action 测试，适配既有 START 取消测试。

部署机 `/home/user2/pika_teleop_ws` 已同步并核对文件哈希；`colcon build --symlink-install --packages-up-to pika_teleop_bringup` 的 9 个包成功。隔离 `ROS_DOMAIN_ID=211` 顺序运行 Session Manager 12 项、Bag Virtual Receiver 11 项、脚踏 13 项测试均通过；Bridge 包当前没有测试，`colcon test-result --verbose` 为 0 错误/失败。安装后的 YAML 解析值：正式 Bridge 1000 ms、正式 Mapper 保留 2000 ms、正式复位开启/延迟 4000 ms/State 确认 6000 ms；Bag Bridge 2000 ms、Bag Mapper 200 ms。仓库原有混合 CRLF，`git -c core.whitespace=cr-at-eol diff --check` 通过。未启动正式 launch、未连接真实 Recorder 或触发 RealMan MoveJ；需由现场在下次启动后低风险联调。4000 ms 只作为行为树控制权交接的试验缓冲，不能证明 `arm is busy` 已消除。Git 未 add/commit/push。

## 2026-10-09：脚踏板改为踩下切换启停

现场原始 evdev 事件显示，用户持续踩住约 3 秒时，设备仍只给出约 40 ms 的 KEY_F3 按下/释放脉冲；因此原“按住运行、释放停止”会立即中断。现改为每次有效踩下切换：第一次按先左后右启动，第二次按停止；物理释放不影响启停。同批读取到按下和释放时按事件时间戳处理，以免系统调度延迟吞掉短脉冲。150 ms 重复触发保护可通过 `foot_pedal_retrigger_guard_ms` 调整；保留 30 ms 输入消抖、启动超时、迟到 START 补偿和设备失联保护。`/foot_pedal/pressed` 保留物理状态，新增 `/foot_pedal/enabled` 显示逻辑启停请求（不是双臂 ACTIVE 证明）。

两份 launch 的 `start_foot_pedal` 默认设为 `true`，与部署机已有本地改动一致；原 Bag 命令加 `start_pika_official:=true`、`pika_ros_ws:=/home/user2/pika_ros` 可同时拉起脚踏节点。`start_foot_pedal:=false` 可临时关闭。更新根、bringup、Bridge 和脚踏包 README 与隔离假服务测试。停止后 Bag 4000 ms 归位交接仍按原流程执行；脚踏松开不会停机，仍须依赖第二次踩下、手动 STOP 或硬件急停。

已同步至部署机并校验 25 个文件；`colcon build --symlink-install --packages-up-to pika_teleop_bringup` 的 9 个包成功。`ROS_DOMAIN_ID=211` 顺序运行脚踏 13 项（含同批 40 ms 脉冲）、Session Manager 2 项、Bag Virtual Receiver 11 项测试均通过，`colcon test-result` 为 0 错误/失败；两份 launch 的 `--show-args` 均显示脚踏默认 `true`、重复触发保护默认 `150.0`。首次并行运行时 Virtual Receiver 的 11 项断言通过但进程退出出现一次段错误，顺序复跑正常。未启动真机 launch、未触发 RealMan Action；用户下次启动才会加载新节点逻辑。

## 2026-10-09：单脚踏板按住控制双臂

基线为 `db79c0fcf0cb7bda3d1c9ed4003075b0043918c5`。旧本地工作副本有未提交改动，保留原样；本次在新的 `outputs/foot_pedal_repo` 实施，不执行 Git add/commit/push。

- 新增 `src/pika_foot_pedal`：一个 evdev ROS 节点，30 ms 消抖、20 Hz `/foot_pedal/pressed`、启动已踩住须先松开、非阻塞读取。按下先请求左侧，等实际 State ACTIVE 后请求右侧；松开异步 STOP 已请求侧。START 拒绝/超时后停止并要求松开重试，不形成请求风暴。设备异常使用 `pedal_device_lost`。
- Bridge `node.py` 增加默认开启的 `gesture_enabled`；脚踏 launch 时关闭夹爪双击/三击判定。手动服务在设备异常时映射 `STALE_STOP`，在未 ACTIVE 的 START 取消时映射 `START_CANCEL_STOP`；原 `reason=manual` 的正常 ACTIVE 停止仍为 `USER_STOP`。
- Session Manager `node.py` 补上 STARTING 期间收到 STOP 的取消记录；迟到 Recorder START 成功后立即请求 STOP，跳过未实际启用时的自动 MoveJ。正常已 ACTIVE 的 USER_STOP/Recorder/MoveJ 路径不变。新增延迟 START 测试。
- 两份 bringup launch 增加默认关闭的 `start_foot_pedal`，以及设备路径、键码、消抖参数，加入新包 Python 路径；Bag 已有 4000 ms handoff 与 Mapper 配置未改。
- 更新根、Bridge、bringup 和新增包 README；新增脚踏开关消抖单元测试。

部署与验证：同步至 `user2@192.168.31.97:/home/user2/pika_teleop_ws` 后，`colcon build --symlink-install` 构建涉及包成功，`ros2 pkg executables pika_foot_pedal` 能找到 `foot_pedal_node`。部署机安装 `python3-evdev`、配置仅匹配该 USB 脚踏的 udev 规则，已验证 user2 可读设备且 `active_keys()` 正常。两份 launch 的 `--show-args` 能解析脚踏参数，`gesture_enabled` 在脚踏开/关时分别求值为 false/true。

隔离测试使用 `ROS_DOMAIN_ID=211` 与假服务/假 State，未连接现场 RealMan：脚踏包 11 项测试（物理消抖、Bridge 原因映射、先左后右、双侧停止、迟到 START 补偿、缺服务、真实 ROS graph 假服务）通过；Session Manager 2 项快速松脚/第二侧待启动测试通过；Bag Virtual Receiver 原有 11 项假 Action/reset 测试通过。独立节点打开实际 USB 设备并在隔离域发布 `/foot_pedal/pressed=false`；SIGINT 退出无堆栈。未执行真实踩踏、真机遥操、Recorder 真实落盘或 MoveJ 到位验收。脚踏进程被 SIGKILL、断电或 ROS 通信完全中断时无法保证 STOP 送达，须依靠外部命令 watchdog 与硬件急停。当前运行的 launch 未重启；新代码须在下次用户启动后生效。

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
