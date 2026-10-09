"""Virtual downstream receiver for manual Pika teleoperation validation."""

from dataclasses import dataclass, field
import math
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from pika_teleop_interfaces.msg import PikaTeleopState
from pika_teleop_interfaces.srv import SetTeleopEnabled

from .reset_handoff import ResetHandoff


NANOSECONDS_PER_MILLISECOND = 1_000_000
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass
class SideStatus:
    """Independent receive and safety state for one side."""

    latest_state: Optional[PikaTeleopState] = None
    last_state_receipt_ns: Optional[int] = None
    rx_count: int = 0
    last_report_count: int = 0
    last_report_time_ns: int = 0
    expected_enabled: bool = False
    expected_change_ns: int = 0
    transition_start_rx_count: int = 0
    transition_pending: bool = False
    transition_warned: bool = False
    safe_stop: bool = True
    watchdog_active: bool = False
    anomaly_counts: Dict[str, int] = field(default_factory=dict)
    reported_anomalies: set = field(default_factory=set)
    last_logged_state: Optional[tuple] = None


class PikaTeleopVirtualReceiver(Node):
    """Accept bridge handshakes and observe state like a safe downstream node."""

    SIDES = ('left', 'right')
    QUATERNION_EPSILON = 1.0e-12

    def __init__(self, **kwargs) -> None:
        super().__init__('pika_teleop_virtual_receiver', **kwargs)
        self.declare_parameter('state_transition_warn_ms', 100.0)
        self.declare_parameter('state_timeout_ms', 100.0)
        self.declare_parameter('accept_start', True)
        self.declare_parameter('periodic_summary', False)
        self.declare_parameter('reset_on_user_stop', False)
        self.declare_parameter('left_reset_action', '/l/execute_motion')
        self.declare_parameter('right_reset_action', '/r/execute_motion')
        for side in self.SIDES:
            self.declare_parameter(
                f'{side}_reset_joint_degrees', Parameter.Type.DOUBLE_ARRAY
            )
        self.declare_parameter('reset_velocity_percent', 10)
        self.declare_parameter('reset_blend_radius_percent', 0)
        self.declare_parameter('reset_timeout_sec', 120.0)
        self.declare_parameter('reset_dispatch_delay_ms', 150.0)
        self.declare_parameter('reset_stop_confirm_timeout_ms', 1000.0)

        self.state_transition_warn_ms = self._positive_parameter(
            'state_transition_warn_ms'
        )
        self.state_timeout_ms = self._positive_parameter('state_timeout_ms')
        self.accept_start = bool(self.get_parameter('accept_start').value)
        self.periodic_summary = bool(
            self.get_parameter('periodic_summary').value
        )
        self.reset_on_user_stop = bool(
            self.get_parameter('reset_on_user_stop').value
        )
        if self.reset_on_user_stop:
            from realman_msgs.action import ExecuteMotion

            self._execute_motion_type = ExecuteMotion
            self.reset_actions = {
                side: self._string_parameter(f'{side}_reset_action')
                for side in self.SIDES
            }
            self.reset_joints = {
                side: self._array_parameter(f'{side}_reset_joint_degrees', 6)
                for side in self.SIDES
            }
            self.reset_velocity_percent = self._percentage_parameter(
                'reset_velocity_percent'
            )
            self.reset_blend_radius_percent = self._percentage_parameter(
                'reset_blend_radius_percent'
            )
            self.reset_timeout_sec = self._positive_parameter('reset_timeout_sec')
            dispatch_delay_ms = self._nonnegative_parameter(
                'reset_dispatch_delay_ms'
            )
            confirm_timeout_ms = self._positive_parameter(
                'reset_stop_confirm_timeout_ms'
            )
            self._reset_handoffs = {
                side: ResetHandoff(
                    int(confirm_timeout_ms * NANOSECONDS_PER_MILLISECOND),
                    int(dispatch_delay_ms * NANOSECONDS_PER_MILLISECOND),
                )
                for side in self.SIDES
            }
            self._reset_clients = {
                side: ActionClient(
                    self, self._execute_motion_type, self.reset_actions[side]
                )
                for side in self.SIDES
            }
        else:
            self._reset_handoffs = {}
            self._reset_clients = {}
        self._reset_goal_futures = {}
        self._transition_warn_ns = int(
            self.state_transition_warn_ms * NANOSECONDS_PER_MILLISECOND
        )
        self._state_timeout_ns = int(
            self.state_timeout_ms * NANOSECONDS_PER_MILLISECOND
        )

        start_ns = time.monotonic_ns()
        self.status = {
            side: SideStatus(
                last_report_time_ns=start_ns,
                expected_change_ns=start_ns,
            )
            for side in self.SIDES
        }
        self._start_ns = start_ns

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_subscriptions = [
            self.create_subscription(
                PikaTeleopState,
                f'/pika_teleop/{side}/state',
                lambda message, side=side: self._state_callback(side, message),
                state_qos,
            )
            for side in self.SIDES
        ]
        self._enable_services = [
            self.create_service(
                SetTeleopEnabled,
                f'/pika_teleop/{side}/set_enabled',
                lambda request, response, side=side: self._service_callback(
                    side, request, response
                ),
            )
            for side in self.SIDES
        ]
        self._summary_timer = (
            self.create_timer(1.0, self._summary_tick)
            if self.periodic_summary else None
        )
        monitor_period_s = max(
            0.01,
            min(0.05, self.state_timeout_ms / 2000.0),
        )
        self._monitor_timer = self.create_timer(
            monitor_period_s, self._monitor_tick
        )

        self.get_logger().info(
            'Virtual receiver ready: timeout=%.1f ms, transition_warn=%.1f ms, '
            'accept_start=%s'
            % (
                self.state_timeout_ms,
                self.state_transition_warn_ms,
                self.accept_start,
            )
        )
        if self.reset_on_user_stop:
            self.get_logger().warning(
                'Bag USER_STOP reset enabled: MoveJ Goal is fire-and-forget. '
                'Confirm physical motion has finished before restarting that side.'
            )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
        return value

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _percentage_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if value < 0 or value > 100:
            raise ValueError(f'{name} must be in [0, 100]')
        return value

    def _array_parameter(self, name: str, size: int):
        raw = self.get_parameter(name).value
        values = () if raw is None else tuple(float(value) for value in raw)
        if len(values) != size or not all(math.isfinite(v) for v in values):
            raise ValueError(f'{name} must contain {size} finite values')
        return values

    @staticmethod
    def _stamp_text(stamp) -> str:
        return f'{int(stamp.sec)}.{int(stamp.nanosec):09d}'

    @classmethod
    def _validate_state(cls, message: PikaTeleopState) -> List[str]:
        issues = []
        position = message.pose.position
        orientation = message.pose.orientation
        twist = message.twist

        if not all(
            math.isfinite(float(value))
            for value in (position.x, position.y, position.z)
        ):
            issues.append('position_non_finite')
        quaternion_values = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(
            math.isfinite(float(value)) for value in quaternion_values
        ):
            issues.append('quaternion_non_finite')
        else:
            norm = math.sqrt(
                sum(float(value) ** 2 for value in quaternion_values)
            )
            if norm <= cls.QUATERNION_EPSILON:
                issues.append('quaternion_not_normalizable')
        twist_values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(float(value)) for value in twist_values):
            issues.append('twist_non_finite')
        if not math.isfinite(float(message.gripper_position)):
            issues.append('gripper_non_finite')
        if not all(
            math.isfinite(float(value))
            for value in (message.pose_age_ms, message.gripper_age_ms)
        ):
            issues.append('age_non_finite')
        return issues

    def _state_callback(self, side: str, message: PikaTeleopState) -> None:
        now_ns = time.monotonic_ns()
        status = self.status[side]
        status.latest_state = message
        status.last_state_receipt_ns = now_ns
        status.rx_count += 1
        status.safe_stop = False
        if self.reset_on_user_stop:
            event = self._reset_handoffs[side].state(
                bool(message.enabled), bool(message.valid), status.rx_count, now_ns
            )
            if event == 'confirmed':
                self.get_logger().info(
                    f'{side.upper()} RESET STOP confirmed by new disabled State'
                )
            elif event:
                self.get_logger().warning(
                    f'{side.upper()} RESET CANCELED: {event}'
                )
        state_flags = (bool(message.enabled), bool(message.valid))
        if state_flags != status.last_logged_state:
            gate = 'ACCEPTED' if all(state_flags) else 'BLOCKED'
            self.get_logger().info(
                '%s STATE enabled=%s valid=%s control_gate=%s'
                % (side.upper(), *state_flags, gate)
            )
            status.last_logged_state = state_flags
        issues = (
            set(self._validate_state(message))
            if message.enabled and message.valid else set()
        )
        for issue in issues:
            status.anomaly_counts[issue] = (
                status.anomaly_counts.get(issue, 0) + 1
            )
        for issue in sorted(issues - status.reported_anomalies):
            self.get_logger().warning(
                f'{side.upper()} STATE DATA ANOMALY: {issue}'
            )
        status.reported_anomalies.update(issues)

    def _service_callback(self, side: str, request, response):
        status = self.status[side]
        if (
            request.enable and self.reset_on_user_stop
            and self._reset_handoffs[side].pending is not None
        ):
            response.success = False
            response.message = 'START rejected: STOP-to-reset handoff in progress'
            self.get_logger().warning(f'{side.upper()} {response.message}')
            return response
        if request.enable and not self.accept_start:
            response.success = False
            response.message = 'START rejected by virtual receiver'
            self.get_logger().warning(
                '%s SERVICE enable=true reason=%s -> success=false message="%s"'
                % (side.upper(), request.reason, response.message)
            )
            return response

        now_ns = time.monotonic_ns()
        status.expected_enabled = bool(request.enable)
        status.expected_change_ns = now_ns
        status.transition_start_rx_count = status.rx_count
        status.transition_pending = True
        status.transition_warned = False
        if self.reset_on_user_stop:
            handoff = self._reset_handoffs[side]
            if request.enable:
                handoff.start(status.rx_count)
            elif handoff.stop(request.reason, status.rx_count, now_ns):
                self.get_logger().info(
                    f'{side.upper()} RESET pending: waiting for new disabled State'
                )
            elif not request.enable and request.reason != 'USER_STOP':
                self.get_logger().info(
                    f'{side.upper()} RESET skipped/canceled: {request.reason}'
                )
        response.success = True
        response.message = '%s %s accepted' % (
            side.upper(),
            'START' if request.enable else 'STOP',
        )
        self.get_logger().info(
            '%s SERVICE enable=%s reason=%s -> success=true message="%s"'
            % (
                side.upper(),
                str(bool(request.enable)).lower(),
                request.reason,
                response.message,
            )
        )
        return response

    def _reset_goal(self, side: str):
        ExecuteMotion = self._execute_motion_type
        goal = ExecuteMotion.Goal()
        goal.command = ExecuteMotion.Goal.MOVEJ
        goal.reference_type = ExecuteMotion.Goal.BASE
        goal.reference_name = 'base'
        goal.joint_degrees = list(self.reset_joints[side])
        goal.pose_position_m = [0.0, 0.0, 0.0]
        goal.pose_quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]
        goal.velocity_percent = self.reset_velocity_percent
        goal.blend_radius_percent = self.reset_blend_radius_percent
        goal.connect = False
        goal.timeout_sec = float(self.reset_timeout_sec)
        return goal

    def _dispatch_reset(self, side: str) -> None:
        client = self._reset_clients[side]
        try:
            if not client.server_is_ready():
                self.get_logger().warning(
                    f'{side.upper()} RESET NOT SENT: Action server unavailable '
                    f'({self.reset_actions[side]})'
                )
                return
            future = client.send_goal_async(self._reset_goal(side))
            self._reset_goal_futures[side] = future
            future.add_done_callback(
                lambda completed, side=side: self._reset_goal_response(
                    side, completed
                )
            )
            self.get_logger().info(
                f'{side.upper()} RESET Goal sent to {self.reset_actions[side]}; '
                'motion completion is not monitored'
            )
        except Exception as exc:
            self.get_logger().error(f'{side.upper()} RESET NOT SENT: {exc}')

    def _reset_goal_response(self, side: str, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.accepted:
                self.get_logger().info(
                    f'{side.upper()} RESET Goal accepted; physical completion '
                    'is not monitored'
                )
            else:
                self.get_logger().warning(f'{side.upper()} RESET Goal rejected')
        except Exception as exc:
            self.get_logger().error(
                f'{side.upper()} RESET Goal response failed: {exc}'
            )
        finally:
            if self._reset_goal_futures.get(side) is future:
                self._reset_goal_futures.pop(side, None)

    def _monitor_tick(self) -> None:
        now_ns = time.monotonic_ns()
        for side in self.SIDES:
            status = self.status[side]
            reference_ns = status.last_state_receipt_ns
            if reference_ns is None:
                reference_ns = self._start_ns
            timed_out = now_ns - reference_ns > self._state_timeout_ns
            if timed_out:
                status.safe_stop = True
                if not status.watchdog_active:
                    status.watchdog_active = True
                    if self.reset_on_user_stop:
                        if self._reset_handoffs[side].invalidate():
                            self.get_logger().warning(
                                f'{side.upper()} RESET CANCELED: State watchdog timeout'
                            )
                    self.get_logger().warning(
                        '%s WATCHDOG TIMEOUT: no state for > %.1f ms; '
                        'safe_stop=true control_gate=BLOCKED'
                        % (side.upper(), self.state_timeout_ms)
                    )
            elif status.watchdog_active:
                status.watchdog_active = False
                status.safe_stop = False
                self.get_logger().info(
                    f'{side.upper()} WATCHDOG RECOVERED: state reception resumed'
                )

            if self.reset_on_user_stop:
                event = self._reset_handoffs[side].tick(now_ns)
                if event == 'dispatch':
                    self._dispatch_reset(side)
                elif event:
                    self.get_logger().warning(
                        f'{side.upper()} RESET CANCELED: {event}'
                    )

            if status.transition_pending:
                state = status.latest_state
                received_after_request = (
                    status.rx_count > status.transition_start_rx_count
                )
                if (
                    received_after_request
                    and state is not None
                    and bool(state.enabled) == status.expected_enabled
                ):
                    status.transition_pending = False
                elif (
                    not status.transition_warned
                    and now_ns - status.expected_change_ns
                    > self._transition_warn_ns
                ):
                    actual = 'NO_STATE' if state is None else str(state.enabled)
                    status.transition_warned = True
                    self.get_logger().warning(
                        '%s STATE TRANSITION LATE: expected_enabled=%s, '
                        'state.enabled=%s after %.1f ms'
                        % (
                            side.upper(),
                            status.expected_enabled,
                            actual,
                            self.state_transition_warn_ms,
                        )
                    )

    def _summary_tick(self) -> None:
        now_ns = time.monotonic_ns()
        for side in self.SIDES:
            status = self.status[side]
            elapsed_s = max(
                (now_ns - status.last_report_time_ns)
                / NANOSECONDS_PER_SECOND,
                1.0e-9,
            )
            received = status.rx_count - status.last_report_count
            rx_hz = received / elapsed_s
            status.last_report_count = status.rx_count
            status.last_report_time_ns = now_ns
            self.get_logger().info(self._format_summary(side, status, rx_hz))

    def _format_summary(
        self, side: str, status: SideStatus, rx_hz: float
    ) -> str:
        message = status.latest_state
        if message is None:
            return (
                '\n%s\n  rx_hz=%.1f expected_enabled=%s state=NO_STATE\n'
                '  safe_stop=%s control_gate=BLOCKED'
                % (
                    side.upper(),
                    rx_hz,
                    status.expected_enabled,
                    status.safe_stop,
                )
            )

        position = message.pose.position
        quaternion = message.pose.orientation
        quaternion_norm = math.sqrt(
            float(quaternion.x) ** 2
            + float(quaternion.y) ** 2
            + float(quaternion.z) ** 2
            + float(quaternion.w) ** 2
        )
        linear = message.twist.linear
        angular = message.twist.angular
        accepted = bool(message.enabled and message.valid and not status.safe_stop)
        gate = 'ACCEPTED' if accepted else 'BLOCKED'
        anomaly_text = (
            'none'
            if not status.anomaly_counts
            else ','.join(
                f'{name}:{count}'
                for name, count in sorted(status.anomaly_counts.items())
            )
        )
        return (
            '\n%s\n'
            '  rx_hz=%.1f expected_enabled=%s state.enabled=%s '
            'state.valid=%s velocity_valid=%s\n'
            '  safe_stop=%s control_gate=%s frame_id=%s\n'
            '  position=(%.6f, %.6f, %.6f)\n'
            '  quaternion=(%.6f, %.6f, %.6f, %.6f) norm=%.6f\n'
            '  linear_velocity=(%.6f, %.6f, %.6f) m/s\n'
            '  angular_velocity=(%.6f, %.6f, %.6f) rad/s\n'
            '  gripper_position=%.6f pose_age_ms=%.3f '
            'gripper_age_ms=%.3f\n'
            '  pose_source_stamp=%s gripper_source_stamp=%s\n'
            '  anomaly_counts=%s'
            % (
                side.upper(),
                rx_hz,
                status.expected_enabled,
                message.enabled,
                message.valid,
                message.velocity_valid,
                status.safe_stop,
                gate,
                message.header.frame_id,
                position.x,
                position.y,
                position.z,
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
                quaternion_norm,
                linear.x,
                linear.y,
                linear.z,
                angular.x,
                angular.y,
                angular.z,
                message.gripper_position,
                message.pose_age_ms,
                message.gripper_age_ms,
                self._stamp_text(message.pose_source_stamp),
                self._stamp_text(message.gripper_source_stamp),
                anomaly_text,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PikaTeleopVirtualReceiver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
