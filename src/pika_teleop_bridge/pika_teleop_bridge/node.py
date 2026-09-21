"""100 Hz robot-agnostic Pika teleoperation state bridge."""

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from pika_teleop_interfaces.msg import PikaTeleopState
from pika_teleop_interfaces.srv import SetTeleopEnabled
from std_msgs.msg import Bool, Empty

from .frame_transform import PikaFrameTransform
from .gesture import GestureDetector
from .pika_input import PikaInputAdapter, TimedSample
from .safety import PoseJumpGuard
from .velocity import VelocityEstimator


class PikaTeleopPublisher(Node):
    """Publish independent left/right state and service handshakes."""

    SIDES = ('left', 'right')
    IDLE = 'IDLE'
    PENDING_START = 'PENDING_START'
    ACTIVE = 'ACTIVE'

    def __init__(self) -> None:
        super().__init__('pika_teleop_publisher')
        self.declare_parameter('state_rate_hz', 100.0)
        self.declare_parameter('stale_stop_ms', 50.0)
        self.declare_parameter('gripper_open_threshold', 0.075)
        self.declare_parameter('gripper_close_threshold', 0.025)
        self.declare_parameter('click_max_interval_ms', 450.0)
        self.declare_parameter('gesture_reset_timeout_ms', 1200.0)
        self.declare_parameter('max_position_jump_m', 0.08)
        self.declare_parameter('max_rotation_jump_deg', 45.0)
        self.declare_parameter('start_service_timeout_ms', 10000.0)
        self.declare_parameter('velocity_filter_cutoff_hz', 10.0)
        self.declare_parameter('velocity_max_dt_ms', 50.0)
        self.declare_parameter('use_session_gate', True)

        self.state_rate_hz = self._positive_parameter('state_rate_hz')
        self.stale_stop_ms = self._positive_parameter('stale_stop_ms')
        self.start_service_timeout_ms = self._positive_parameter(
            'start_service_timeout_ms'
        )
        self.use_session_gate = bool(
            self.get_parameter('use_session_gate').value
        )
        self.start_allowed = not self.use_session_gate
        self._last_gate_warning_ns = 0
        open_threshold = float(
            self.get_parameter('gripper_open_threshold').value
        )
        close_threshold = float(
            self.get_parameter('gripper_close_threshold').value
        )
        click_interval_ms = self._positive_parameter('click_max_interval_ms')
        reset_timeout_ms = self._positive_parameter(
            'gesture_reset_timeout_ms'
        )
        max_position_jump_m = self._positive_parameter(
            'max_position_jump_m'
        )
        max_rotation_jump_deg = self._positive_parameter(
            'max_rotation_jump_deg'
        )
        velocity_filter_cutoff_hz = self._positive_parameter(
            'velocity_filter_cutoff_hz'
        )
        velocity_max_dt_ms = self._positive_parameter(
            'velocity_max_dt_ms'
        )

        self.adapter = PikaInputAdapter(self)
        self.mode: Dict[str, str] = {side: self.IDLE for side in self.SIDES}
        self.detectors = {
            side: GestureDetector(
                open_threshold,
                close_threshold,
                click_interval_ms,
                reset_timeout_ms,
            )
            for side in self.SIDES
        }
        self.pose_guards = {
            side: PoseJumpGuard(
                max_position_jump_m,
                max_rotation_jump_deg,
            )
            for side in self.SIDES
        }
        self.velocity_estimators = {
            side: VelocityEstimator(
                velocity_filter_cutoff_hz,
                velocity_max_dt_ms,
            )
            for side in self.SIDES
        }

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_publishers = {
            side: self.create_publisher(
                PikaTeleopState,
                f'/pika_teleop/{side}/state',
                state_qos,
            )
            for side in self.SIDES
        }
        self.enable_clients = {
            side: self.create_client(
                SetTeleopEnabled,
                f'/pika_teleop/{side}/set_enabled',
            )
            for side in self.SIDES
        }
        gate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        force_stop_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._start_gate_subscription = self.create_subscription(
            Bool,
            '/pika_session/start_allowed',
            self._start_allowed_callback,
            gate_qos,
        )
        self._force_stop_subscription = self.create_subscription(
            Empty,
            '/pika_session/force_stop_all',
            self._force_stop_all_callback,
            force_stop_qos,
        )
        self.pending_start_futures: Dict[str, Optional[object]] = {
            side: None for side in self.SIDES
        }
        self.pending_start_deadline_ns: Dict[str, Optional[int]] = {
            side: None for side in self.SIDES
        }
        self.disable_futures = []

        self.timer = self.create_timer(
            1.0 / self.state_rate_hz, self._control_tick
        )
        self.get_logger().info(
            'Pika teleop bridge ready: state=%.1f Hz, stale=%.1f ms, '
            'start_timeout=%.1f ms, velocity=(%.1f Hz, %.1f ms), '
            'pose_jump=(%.3f m, %.1f deg), session_gate=%s'
            % (
                self.state_rate_hz,
                self.stale_stop_ms,
                self.start_service_timeout_ms,
                velocity_filter_cutoff_hz,
                velocity_max_dt_ms,
                max_position_jump_m,
                max_rotation_jump_deg,
                self.use_session_gate,
            )
        )

    def _start_allowed_callback(self, message: Bool) -> None:
        allowed = bool(message.data)
        if allowed != self.start_allowed:
            self.get_logger().info(f'SESSION START ALLOWED={allowed}')
        self.start_allowed = allowed

    def _force_stop_all_callback(self, _message: Empty) -> None:
        for side in self.SIDES:
            future = self.pending_start_futures[side]
            if future is not None and not future.done():
                future.cancel()
            self.mode[side] = self.IDLE
            self._clear_pending_start(side)
            self.detectors[side].reset()
            self.pose_guards[side].reset()
            self.velocity_estimators[side].reset()
        self.get_logger().warning('SESSION FORCE STOP ALL')

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _freshness_age_ms(sample: TimedSample) -> float:
        if sample.age_ms is not None:
            return sample.age_ms
        if sample.receipt_age_ms is not None:
            return sample.receipt_age_ms
        return math.inf

    @staticmethod
    def _pose_finite(sample: TimedSample) -> bool:
        if not sample.valid:
            return False
        pose = sample.msg.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return (
            all(math.isfinite(float(value)) for value in values)
            and PoseJumpGuard.quaternion_is_normalizable(pose)
        )

    @staticmethod
    def _gripper_value(sample: TimedSample) -> float:
        if not sample.valid or not sample.msg.position:
            return math.nan
        return float(sample.msg.position[0])

    def _side_samples(self, snapshot, side: str) -> Tuple[TimedSample, TimedSample]:
        return (
            getattr(snapshot, f'{side}_pose'),
            getattr(snapshot, f'{side}_gripper'),
        )

    def _data_usable(self, pose: TimedSample, gripper: TimedSample) -> bool:
        pose_age = self._freshness_age_ms(pose)
        gripper_age = self._freshness_age_ms(gripper)
        return (
            self._pose_finite(pose)
            and math.isfinite(self._gripper_value(gripper))
            and 0.0 <= pose_age <= self.stale_stop_ms
            and 0.0 <= gripper_age <= self.stale_stop_ms
        )

    def _clear_pending_start(self, side: str) -> None:
        self.pending_start_futures[side] = None
        self.pending_start_deadline_ns[side] = None

    def _call_enable_service(self, side: str, enable: bool, reason: str):
        client = self.enable_clients[side]
        if not client.service_is_ready():
            return None
        request = SetTeleopEnabled.Request()
        request.enable = enable
        request.reason = reason
        return client.call_async(request)

    def _try_send_pending_start(self, side: str) -> None:
        if self.pending_start_futures[side] is not None:
            return
        future = self._call_enable_service(side, True, 'USER_START')
        if future is not None:
            self.pending_start_futures[side] = future
            self.get_logger().info(f'{side.upper()} USER_START requested')

    def _request_start(self, side: str, now_ns: int) -> None:
        self.mode[side] = self.PENDING_START
        self.pose_guards[side].reset()
        self.velocity_estimators[side].reset()
        self.pending_start_deadline_ns[side] = now_ns + int(
            self.start_service_timeout_ms * 1.0e6
        )
        self.pending_start_futures[side] = None
        self._try_send_pending_start(side)

    def _send_disable(self, side: str, reason: str) -> None:
        future = self._call_enable_service(side, False, reason)
        if future is None:
            self.get_logger().warning(
                f'{side.upper()} disable service unavailable: {reason}'
            )
            return
        self.disable_futures.append((side, reason, future))

    def _deactivate(self, side: str, reason: str) -> None:
        """Stop locally first; downstream notification is deliberately async."""
        self.mode[side] = self.IDLE
        self._clear_pending_start(side)
        self.detectors[side].reset()
        self.pose_guards[side].reset()
        self.velocity_estimators[side].reset()
        self.get_logger().info(f'{side.upper()} {reason}')
        self._send_disable(side, reason)

    def _process_pending_start(
        self,
        side: str,
        now_ns: int,
        pose: TimedSample,
        data_usable: bool,
    ) -> None:
        if self.mode[side] != self.PENDING_START:
            return
        deadline_ns = self.pending_start_deadline_ns[side]
        if deadline_ns is not None and now_ns >= deadline_ns:
            future = self.pending_start_futures[side]
            if future is not None:
                future.cancel()
            self.mode[side] = self.IDLE
            self._clear_pending_start(side)
            self.detectors[side].reset()
            self.velocity_estimators[side].reset()
            self.get_logger().warning(f'{side.upper()} USER_START timed out')
            return

        self._try_send_pending_start(side)
        future = self.pending_start_futures[side]
        if future is None or not future.done():
            return
        try:
            response = future.result()
        except Exception as exc:  # rclpy futures surface transport failures here
            self.get_logger().warning(
                f'{side.upper()} USER_START service failed: {exc}'
            )
            self.mode[side] = self.IDLE
            self._clear_pending_start(side)
            self.detectors[side].reset()
            return
        self._clear_pending_start(side)
        if response is None or not response.success:
            message = '' if response is None else response.message
            self.mode[side] = self.IDLE
            self.detectors[side].reset()
            self.get_logger().warning(
                f'{side.upper()} USER_START rejected: {message}'
            )
            return
        if not data_usable:
            self.mode[side] = self.IDLE
            self.detectors[side].reset()
            self.get_logger().warning(
                f'{side.upper()} USER_START acknowledged after data became invalid'
            )
            self._send_disable(side, 'STALE_STOP')
            return

        self.mode[side] = self.ACTIVE
        self.pose_guards[side].initialize(pose.msg.pose)
        self.velocity_estimators[side].reset()
        self.get_logger().info(f'{side.upper()} ACTIVE: {response.message}')

    def _check_pose_jump(self, side: str, pose: TimedSample) -> bool:
        if not pose.new_sample:
            return False
        result = self.pose_guards[side].check(pose.msg.pose)
        if not result.triggered:
            return False
        self.get_logger().warning(
            '%s POSE_JUMP_STOP: position_jump=%.3f m, '
            'rotation_jump=%.1f deg, thresholds=(%.3f m, %.1f deg)'
            % (
                side.upper(),
                result.position_jump_m,
                result.rotation_jump_deg,
                self.pose_guards[side].max_position_jump_m,
                self.pose_guards[side].max_rotation_jump_deg,
            )
        )
        self._deactivate(side, 'POSE_JUMP_STOP')
        return True

    def _build_state(
        self,
        control_stamp,
        pose: TimedSample,
        gripper: TimedSample,
        transformed_pose,
        enabled: bool,
        valid: bool,
        side: str,
    ) -> PikaTeleopState:
        output = PikaTeleopState()
        output.header.stamp = control_stamp
        output.header.frame_id = PikaFrameTransform.FRAME_ID
        if transformed_pose is not None:
            output.pose = transformed_pose
        if pose.valid:
            output.pose_source_stamp = pose.msg.header.stamp
        if gripper.valid:
            output.gripper_source_stamp = gripper.msg.header.stamp
        output.gripper_position = self._gripper_value(gripper)
        output.enabled = enabled
        output.valid = valid
        estimator = self.velocity_estimators[side]
        if enabled and valid and estimator.valid:
            output.twist = estimator.twist
            output.velocity_valid = True
        output.pose_age_ms = float(self._freshness_age_ms(pose))
        output.gripper_age_ms = float(self._freshness_age_ms(gripper))
        return output

    def _process_gesture(
        self,
        side: str,
        pose: TimedSample,
        gripper: TimedSample,
        data_usable: bool,
    ) -> None:
        if self.mode[side] == self.PENDING_START or not gripper.new_sample:
            return
        gripper_value = self._gripper_value(gripper)
        sample_time_ns = gripper.receipt_time_ns
        if sample_time_ns is None:
            return
        target_clicks = 2 if self.mode[side] == self.IDLE else 3
        triggered = self.detectors[side].update(
            gripper_value,
            sample_time_ns,
            target_clicks,
        )
        if not triggered:
            return
        if self.mode[side] == self.IDLE:
            if not data_usable:
                self.get_logger().warning(
                    f'{side.upper()} USER_START rejected locally: data unusable'
                )
                return
            if self.use_session_gate and not self.start_allowed:
                now_ns = time.monotonic_ns()
                if now_ns - self._last_gate_warning_ns >= 1_000_000_000:
                    self._last_gate_warning_ns = now_ns
                    self.get_logger().warning(
                        f'{side.upper()} USER_START blocked: session not ready'
                    )
                return
            self._request_start(side, sample_time_ns)
            return
        self._deactivate(side, 'USER_STOP')

    def _clean_disable_futures(self) -> None:
        remaining = []
        for side, reason, future in self.disable_futures:
            if not future.done():
                remaining.append((side, reason, future))
                continue
            try:
                response = future.result()
                if response is None or not response.success:
                    message = '' if response is None else response.message
                    self.get_logger().warning(
                        f'{side.upper()} {reason} not acknowledged: {message}'
                    )
            except Exception as exc:
                self.get_logger().warning(
                    f'{side.upper()} {reason} service failed: {exc}'
                )
        self.disable_futures = remaining

    def _control_tick(self) -> None:
        control_time = self.get_clock().now()
        snapshot = self.adapter.get_snapshot(control_time.nanoseconds)
        side_data = {}

        for side in self.SIDES:
            pose, gripper = self._side_samples(snapshot, side)
            usable = self._data_usable(pose, gripper)
            self._process_pending_start(
                side, control_time.nanoseconds, pose, usable
            )
            if self.mode[side] == self.ACTIVE and not usable:
                self._deactivate(side, 'STALE_STOP')
            elif self.mode[side] == self.ACTIVE:
                self._check_pose_jump(side, pose)

            transformed_pose = None
            if self._pose_finite(pose):
                transformed_pose = PikaFrameTransform.transform_pose(
                    pose.msg.pose
                )
            enabled = self.mode[side] == self.ACTIVE
            valid = enabled and usable
            estimator = self.velocity_estimators[side]
            if enabled and usable and pose.new_sample:
                estimator.update(transformed_pose, pose.source_time_ns)

            state = self._build_state(
                control_time.to_msg(),
                pose,
                gripper,
                transformed_pose,
                enabled,
                valid,
                side,
            )
            self.state_publishers[side].publish(state)
            side_data[side] = (pose, gripper, usable)

        # Publish the gesture-completing sample before applying its transition.
        for side in self.SIDES:
            self._process_gesture(side, *side_data[side])
        self._clean_disable_futures()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PikaTeleopPublisher()
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
