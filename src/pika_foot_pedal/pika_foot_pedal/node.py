"""Nonblocking evdev reader and one-pedal, two-arm service coordinator."""

import select
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

from pika_teleop_interfaces.msg import PikaTeleopState
from pika_teleop_interfaces.srv import SetTeleopEnabled

from .debounce import PedalDebouncer


class FootPedalNode(Node):
    SIDES = ('left', 'right')

    def __init__(self):
        super().__init__('pika_foot_pedal')
        self.declare_parameter('device', '/dev/input/by-id/usb-0483_5750-if01-event-kbd')
        self.declare_parameter('key_code', 'auto')
        self.declare_parameter('debounce_ms', 30.0)
        self.declare_parameter('retrigger_guard_ms', 150.0)
        self.declare_parameter('pressed_topic', '/foot_pedal/pressed')
        self.declare_parameter('enabled_topic', '/foot_pedal/enabled')
        self.declare_parameter('start_timeout_ms', 10000.0)
        self.device_path = str(self.get_parameter('device').value)
        self.key_setting = str(self.get_parameter('key_code').value)
        self.debouncer = PedalDebouncer(float(self.get_parameter('debounce_ms').value))
        self.retrigger_guard_ns = int(float(self.get_parameter('retrigger_guard_ms').value) * 1e6)
        if self.retrigger_guard_ns < 0:
            raise ValueError('retrigger_guard_ms must be nonnegative')
        self.start_timeout_ns = int(float(self.get_parameter('start_timeout_ms').value) * 1e6)
        if self.start_timeout_ns <= 0:
            raise ValueError('start_timeout_ms must be positive')
        self.publisher = self.create_publisher(
            Bool, str(self.get_parameter('pressed_topic').value), 1
        )
        self.enabled_publisher = self.create_publisher(
            Bool, str(self.get_parameter('enabled_topic').value), 1
        )
        self.enable_clients = {
            side: self.create_client(
                SetTeleopEnabled, f'/pika_teleop/{side}/manual_enable'
            ) for side in self.SIDES
        }
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_subscriptions = [
            self.create_subscription(
                PikaTeleopState, f'/pika_teleop/{side}/state',
                lambda msg, side=side: self._state_callback(side, msg), state_qos
            ) for side in self.SIDES
        ]
        self.device = None
        self.key_code = None
        self._next_open_ns = 0
        self.phase = 'IDLE'
        self.toggle_enabled = False
        self._last_toggle_press_ns = -self.retrigger_guard_ns
        self.generation = 0
        self.requested = set()
        self.pending = []  # (side, enable, generation, future)
        self.stop_retry = {}  # side -> (reason, deadline_ns)
        self.cancel_reason = 'pedal'
        self.active_after_ns = {side: 0 for side in self.SIDES}
        self.start_deadline_ns = 0
        self._resolve_key_setting()
        self._publish()
        self.io_timer = self.create_timer(0.01, self._tick)
        self.topic_timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(
            f'Foot pedal ready: device={self.device_path}, key={self.key_setting}; '
            'release once before the first START'
        )

    def _resolve_key_setting(self):
        if self.key_setting.lower() == 'auto':
            return
        try:
            import evdev
            self.key_code = (
                int(self.key_setting) if self.key_setting.isdecimal()
                else int(getattr(evdev.ecodes, self.key_setting))
            )
        except (ImportError, AttributeError, ValueError) as exc:
            raise ValueError(f'invalid key_code={self.key_setting}: {exc}') from exc

    def _publish(self):
        self.publisher.publish(Bool(data=bool(self.debouncer.pressed)))
        self.enabled_publisher.publish(Bool(data=self.toggle_enabled))

    def _open_device(self, now_ns):
        if self.device is not None or now_ns < self._next_open_ns:
            return
        self._next_open_ns = now_ns + 2_000_000_000
        device = None
        try:
            import evdev
            device = evdev.InputDevice(self.device_path)
            held_keys = set(device.active_keys())
            if self.key_setting.lower() == 'auto' and held_keys:
                self.key_code = min(held_keys)
            held = self.key_code in held_keys if self.key_code is not None else False
            self.debouncer.initialize(held, now_ns)
            self.device = device
            self._publish()
            self.get_logger().info(
                f'Foot pedal connected: {device.path}, key={self.key_code or "auto"}, '
                f'initially_held={held}'
            )
        except (ImportError, OSError) as exc:
            if device is not None:
                device.close()
            self.get_logger().error(
                f'Cannot read foot pedal {self.device_path}: {exc}; '
                'check python3-evdev and /dev/input permissions'
            )

    def _device_fault(self, exc):
        self.get_logger().error(f'Foot pedal disconnected/read failed: {exc}')
        if self.device is not None:
            self.device.close()
            self.device = None
        self.debouncer.disconnect()
        self._publish()
        self._stop_requested('pedal_device_lost')
        self._next_open_ns = time.monotonic_ns() + 2_000_000_000

    def _read_device(self, now_ns):
        self._open_device(now_ns)
        if self.device is None:
            return
        try:
            import evdev
            if not select.select([self.device.fd], [], [], 0)[0]:
                return
            key_events = []
            for event in self.device.read():
                if event.type != evdev.ecodes.EV_KEY or event.value not in (0, 1):
                    continue
                if self.key_code is None:
                    if event.value != 1:
                        continue
                    self.key_code = event.code
                    self.get_logger().info(f'Foot pedal auto-detected key_code={event.code}')
                if event.code == self.key_code:
                    key_events.append(event)
            if key_events:
                self._consume_key_events(key_events, now_ns)
        except (OSError, EOFError, ValueError) as exc:
            self._device_fault(exc)

    def _consume_key_events(self, events, now_ns):
        # Preserve a valid short pulse even if both key edges arrive in one read.
        last_us = events[-1].sec * 1_000_000 + events[-1].usec
        for event in events:
            event_us = event.sec * 1_000_000 + event.usec
            event_ns = now_ns - max(0, last_us - event_us) * 1000
            self._on_edge(self.debouncer.tick(event_ns), event_ns)
            self.debouncer.event(event.value, event_ns)

    def _on_edge(self, edge, now_ns):
        if not edge:
            return
        self._publish()
        if edge == 'press':
            self._handle_press(now_ns)
        else:
            self.get_logger().info('Foot pedal RELEASED')

    def _call(self, side, enable, reason, generation):
        client = self.enable_clients[side]
        if not client.service_is_ready():
            self.get_logger().warning(f'{side.upper()} manual_enable unavailable ({reason})')
            return False
        request = SetTeleopEnabled.Request()
        request.enable = enable
        request.reason = reason
        try:
            future = client.call_async(request)
        except Exception as exc:
            self.get_logger().error(f'{side.upper()} manual_enable call failed: {exc}')
            return False
        self.pending.append((side, enable, generation, future))
        return True

    def _start(self, side):
        if not self.toggle_enabled:
            return
        self.active_after_ns[side] = time.monotonic_ns()
        self.phase = f'{side.upper()}_REQUESTED'
        self.start_deadline_ns = time.monotonic_ns() + self.start_timeout_ns
        if not self._call(side, True, 'pedal', self.generation):
            self._abort(f'{side.upper()} START service unavailable')
        else:
            self.requested.add(side)
            self.get_logger().info(f'{side.upper()} START requested')

    def _state_callback(self, side, message):
        if not (message.enabled and message.valid and self.toggle_enabled):
            return
        if time.monotonic_ns() < self.active_after_ns[side]:
            return
        if side == 'left' and self.phase == 'WAIT_LEFT':
            self.get_logger().info('LEFT ACTIVE; requesting RIGHT START')
            self._start('right')
        elif side == 'right' and self.phase == 'WAIT_RIGHT':
            self.phase = 'RUNNING'
            self.get_logger().info('RIGHT ACTIVE; both sides running')

    def _poll_futures(self):
        current = self.pending
        self.pending = []
        for side, enable, generation, future in current:
            if not future.done():
                self.pending.append((side, enable, generation, future))
                continue
            try:
                response = future.result()
                success = response is not None and response.success
                detail = '' if response is None else response.message
            except Exception as exc:
                success, detail = False, str(exc)
            if enable and generation != self.generation:
                # STOP may have reached Bridge before this delayed START.
                self.get_logger().warning(f'{side.upper()} late START reply; compensating STOP')
                self._request_stop(side, self.cancel_reason, time.monotonic_ns())
                continue
            if enable and self.phase == f'{side.upper()}_REQUESTED':
                if success:
                    self.phase = f'WAIT_{side.upper()}'
                    self.get_logger().info(f'{side.upper()} START accepted; waiting for ACTIVE State')
                else:
                    self._abort(f'{side.upper()} START rejected: {detail}')
            elif not enable:
                if success or 'already idle' in detail:
                    self.stop_retry.pop(side, None)
                else:
                    self.get_logger().warning(f'{side.upper()} STOP failed: {detail}')

    def _request_stop(self, side, reason, now_ns):
        if self._call(side, False, reason, self.generation):
            self.stop_retry.pop(side, None)
        else:
            self.stop_retry[side] = (reason, now_ns + 2_000_000_000)

    def _stop_requested(self, reason):
        self.toggle_enabled = False
        self._publish()
        self.generation += 1
        self.cancel_reason = reason
        now_ns = time.monotonic_ns()
        for side in tuple(self.requested):
            self._request_stop(side, reason, now_ns)
        if self.requested:
            self.get_logger().info(f'Foot pedal STOP requested: {reason}, sides={sorted(self.requested)}')
        self.requested.clear()
        self.phase = 'IDLE'

    def _abort(self, detail):
        self.get_logger().error(detail + '; stopping requested sides; press again to retry')
        self._stop_requested('pedal')

    def _handle_press(self, now_ns):
        if now_ns - self._last_toggle_press_ns < self.retrigger_guard_ns:
            self.get_logger().warning('Foot pedal duplicate press ignored within guard interval')
            return
        self._last_toggle_press_ns = now_ns
        self.get_logger().info('Foot pedal PRESSED')
        if self.toggle_enabled:
            self._stop_requested('pedal')
        elif self.pending or self.stop_retry:
            self.get_logger().warning('Previous START/STOP still pending; press again after it completes')
        else:
            self.toggle_enabled = True
            self.generation += 1
            self._publish()
            self._start('left')

    def _tick(self):
        now_ns = time.monotonic_ns()
        self._read_device(now_ns)
        self._on_edge(self.debouncer.tick(now_ns), now_ns)
        self._poll_futures()
        if self.phase in ('LEFT_REQUESTED', 'WAIT_LEFT', 'RIGHT_REQUESTED', 'WAIT_RIGHT'):
            if now_ns >= self.start_deadline_ns:
                self._abort(f'{self.phase} timed out')
        for side, (reason, deadline_ns) in list(self.stop_retry.items()):
            if self.enable_clients[side].service_is_ready():
                self._request_stop(side, reason, now_ns)
            elif now_ns >= deadline_ns:
                self.get_logger().error(f'{side.upper()} STOP could not be delivered')
                del self.stop_retry[side]

    def close(self):
        if self.requested:
            self._stop_requested('pedal_device_lost')
        if self.device is not None:
            self.device.close()
            self.device = None


def main(args=None):
    # Keep the ROS context alive long enough to send abnormal STOP on Ctrl+C.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_term_handler = signal.getsignal(signal.SIGTERM)
    def _termination_requested(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _termination_requested)
    node = FootPedalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        # Give outstanding asynchronous STOP calls a short chance to complete.
        end = time.monotonic() + 0.5
        while rclpy.ok() and node.pending and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term_handler)


if __name__ == '__main__':
    main()
