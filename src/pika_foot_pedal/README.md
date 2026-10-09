# Pika 单脚踏板双臂控制

`foot_pedal_node` 用 Linux evdev 读取一只 USB 脚踏板。**踩一次启动，再踩一次停止**；松脚不触发停止。这款踏板实测即使持续踩住，也只发送约 40 ms 的按键脉冲。首次有效踩下调用 Bridge 左侧 `manual_enable(true)`，等左侧 `/pika_teleop/left/state` 确认 `enabled && valid` 后才请求右侧；再次有效踩下异步停止已请求的侧。不会持续重试 START。启动失败后，待前一次请求结束再踩一次重试。脚踏板不直接访问 RealMan、Recorder 或 Action。

## 安装和设备

Ubuntu 22.04 安装 `sudo apt install python3-evdev`，然后确认脚踏板：

```bash
ls -l /dev/input/by-id/usb-0483_5750-if01-event-kbd
python3 -c 'import evdev; print(evdev.__version__)'
test -r /dev/input/by-id/usb-0483_5750-if01-event-kbd && echo readable
```

运行遥操的用户必须有这个 event 设备的读取权限。部署机 user2 已安装 `python3-evdev`，并在 `/etc/udev/rules.d/99-pika-foot-pedal.rules` 用 USB VID `0483`、PID `5750`、接口 `01` 仅匹配该脚踏设备，赋予 `user2` 组读取权限；迁移机器时需按实际账号重新设置。不需以 root 运行整套遥操。设备路径按 USB 标识配置，避免 `eventN` 编号漂移。默认 `key_code=auto` 从这个专用设备的第一下按键检测并记录键码；若设备有多个按键，使用 `evtest /dev/input/...` 查出代码后以 `foot_pedal_key_code:=KEY_SPACE` 或数字指定。启动时若脚踏板已踩住，必须先松开，再重新踩下才会 START。

## 使用

```bash
# 两种模式二选一；本机两份 launch 默认 start_foot_pedal:=true
ros2 launch pika_teleop_bringup pika_bag.launch.py start_pika_official:=true pika_ros_ws:=/home/user2/pika_ros
ros2 launch pika_teleop_bringup pika_teleop.launch.py

# 临时关闭脚踏节点并恢复夹爪手势
ros2 launch pika_teleop_bringup pika_bag.launch.py start_foot_pedal:=false

# 自定义输入
ros2 launch pika_teleop_bringup pika_bag.launch.py start_foot_pedal:=true \
  foot_pedal_device:=/dev/input/by-id/usb-0483_5750-if01-event-kbd \
  foot_pedal_key_code:=auto foot_pedal_debounce_ms:=30.0 \
  foot_pedal_retrigger_guard_ms:=150.0

ros2 topic echo /foot_pedal/pressed
ros2 topic echo /foot_pedal/enabled
```

`/foot_pedal/pressed` 是 20 Hz `std_msgs/Bool` 消抖后物理按键状态，短脉冲后会回到 `false`；`/foot_pedal/enabled` 是 20 Hz 的逻辑启停请求状态，`true` 表示已经请求启动，**不等同于双臂已经 ACTIVE**。边沿会立即补发。脚踏模式下 Bridge 的夹爪双击/三击识别关闭，夹爪位置数据照常下发；`pika_start.sh`、`pika_stop.sh` 和手动服务仍可用。第二次踩下走正常 `USER_STOP`，Bag 保留每侧 4000 ms 归位交接，正式模式保留一次 Recorder STOP 与正常归位。尚未进入 ACTIVE 的 START 被取消时不会触发自动归位。设备失联使用 `STALE_STOP`，不会自动归位。

参数：`device`（evdev 路径）、`key_code`（`auto`/`KEY_*`/数字）、`debounce_ms`（默认 30）、`retrigger_guard_ms`（默认 150，忽略过快重复踩下）、`pressed_topic`（默认 `/foot_pedal/pressed`）、`enabled_topic`（默认 `/foot_pedal/enabled`）、`start_timeout_ms`（默认 10000）。设备、键码、消抖和重复触发保护可由两份 launch 的 `foot_pedal_*` 参数覆盖；独立运行时用 ROS 参数。

此脚踏板是人工启停接口，不是硬件急停。松脚不会停止；需要再次踩下、使用手动 STOP 或硬件急停。`SIGKILL`、断电或 ROS 网络完全中断时，节点可能来不及发送 STOP；运动接收器的命令 watchdog 和硬件急停仍需保持有效。停止后的归位 Goal 发出或接受也不代表机械臂已回到安全位置；再次踩下前由操作员现场确认。
