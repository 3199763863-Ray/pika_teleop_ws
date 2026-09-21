"""Coordinate one global recording episode and parallel RealMan resets."""

import math
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Empty, String

from pika_teleop_interfaces.srv import SetTeleopEnabled
from realman_msgs.action import ExecuteMotion
from realman_recording_msgs.srv import ManageRecording


class PikaSessionManager(Node):
    """Own global episode state without embedding recorder logic in Publisher."""

    PREPARING = 'PREPARING'
    READY = 'READY'
    STARTING = 'STARTING'
    RECORDING = 'RECORDING'
    STOPPING = 'STOPPING'
    RESETTING = 'RESETTING'
    FAILED = 'FAILED'
    SIDES = ('left', 'right')
    ABNORMAL_REASONS = ('STALE_STOP', 'POSE_JUMP_STOP')

    def __init__(self) -> None:
        super().__init__('pika_session_manager')
        self._declare_parameters()
        self.recording_service = self._string_parameter('recording_service')
        self.recording_status_topic = self._string_parameter(
            'recording_status_topic'
        )
        self.recording_profile = str(
            self.get_parameter('recording_profile').value
        )
        self.recording_task = str(self.get_parameter('recording_task').value)
        self.recording_duration_sec = int(
            self.get_parameter('recording_duration_sec').value
        )
        if self.recording_duration_sec < 0:
            raise ValueError('recording_duration_sec must be non-negative')
        self.record_cameras = bool(
            self.get_parameter('record_cameras').value
        )
        self.prepare_retry_sec = self._positive_parameter(
            'prepare_retry_sec'
        )
        self.reset_velocity_percent = self._percentage_parameter(
            'reset_velocity_percent'
        )
        self.reset_blend_radius_percent = self._percentage_parameter(
            'reset_blend_radius_percent'
        )
        self.reset_timeout_sec = self._positive_parameter('reset_timeout_sec')
        self.reset_joints = {
            side: self._array_parameter(
                f'{side}_reset_joint_degrees', 6
            )
            for side in self.SIDES
        }
        self._array_parameter('middle_reset_joint_degrees', 6)

        self._lock = threading.RLock()
        self._callback_group = ReentrantCallbackGroup()
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.start_allowed_publisher = self.create_publisher(
            Bool, '/pika_session/start_allowed', latched_qos
        )
        self.force_stop_publisher = self.create_publisher(
            Empty, '/pika_session/force_stop_all', event_qos
        )
        self.state_publisher = self.create_publisher(
            String, '/pika_session/state', latched_qos
        )
        self.recording_client = self.create_client(
            ManageRecording,
            self.recording_service,
            callback_group=self._callback_group,
        )
        self.action_clients = {
            side: ActionClient(
                self,
                ExecuteMotion,
                self._string_parameter(f'{side}_reset_action'),
                callback_group=self._callback_group,
            )
            for side in self.SIDES
        }
        self._left_service = self.create_service(
            SetTeleopEnabled,
            '/pika_teleop/left/set_enabled',
            self._left_set_enabled,
            callback_group=self._callback_group,
        )
        self._right_service = self.create_service(
            SetTeleopEnabled,
            '/pika_teleop/right/set_enabled',
            self._right_set_enabled,
            callback_group=self._callback_group,
        )

        self.state = self.PREPARING
        self.active = {side: False for side in self.SIDES}
        self.session_id = ''
        self._prepare_future = None
        self._prepare_retry_at_ns = 0
        self._stop_future = None
        self._abnormal_stop = False
        self._stop_reason = ''
        self._reset_goal_futures: Dict[str, object] = {}
        self._reset_result_futures: Dict[str, object] = {}
        self._reset_failures: Dict[str, str] = {}
        self._workflow_timer = self.create_timer(
            0.02,
            self._workflow_tick,
            callback_group=self._callback_group,
        )
        self._publish_state()
        self.get_logger().info(
            'Pika session manager started: PREPARING recorder; no robot motion'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('recording_service', '/recording/manage')
        self.declare_parameter('recording_status_topic', '/recording/status')
        self.declare_parameter('recording_profile', 'teleop_v1')
        self.declare_parameter('recording_task', 'pick_and_place')
        self.declare_parameter('recording_duration_sec', 0)
        self.declare_parameter('record_cameras', True)
        self.declare_parameter('prepare_retry_sec', 2.0)
        self.declare_parameter('left_reset_action', '/l/execute_motion')
        self.declare_parameter('right_reset_action', '/r/execute_motion')
        for side in ('left', 'middle', 'right'):
            self.declare_parameter(
                f'{side}_reset_joint_degrees',
                Parameter.Type.DOUBLE_ARRAY,
            )
        self.declare_parameter('reset_velocity_percent', 10)
        self.declare_parameter('reset_blend_radius_percent', 0)
        self.declare_parameter('reset_timeout_sec', 120.0)

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _percentage_parameter(self, name: str) -> int:
        value = int(self.get_parameter(name).value)
        if value < 0 or value > 100:
            raise ValueError(f'{name} must be in [0, 100]')
        return value

    def _array_parameter(self, name: str, size: int):
        raw = self.get_parameter(name).value
        values = () if raw is None else tuple(float(value) for value in raw)
        if len(values) != size or not all(
            math.isfinite(value) for value in values
        ):
            raise ValueError(f'{name} must contain {size} finite values')
        return values

    def _publish_state(self) -> None:
        allowed = self.state in (self.READY, self.RECORDING)
        self.start_allowed_publisher.publish(Bool(data=allowed))
        self.state_publisher.publish(String(data=self.state))

    def _set_state(self, state: str) -> None:
        with self._lock:
            if state != self.state:
                self.get_logger().info(f'SESSION {self.state} -> {state}')
            self.state = state
            self._publish_state()

    @staticmethod
    def _recording_request(command: int) -> ManageRecording.Request:
        request = ManageRecording.Request()
        request.command = command
        request.session_id = ''
        request.profile = ''
        request.task = ''
        request.duration_sec = 0
        request.record_cameras = False
        request.start_at_walltime_ns = 0
        return request

    def _start_request(self) -> ManageRecording.Request:
        request = self._recording_request(ManageRecording.Request.START)
        request.profile = self.recording_profile
        request.task = self.recording_task
        request.duration_sec = self.recording_duration_sec
        request.record_cameras = self.record_cameras
        return request

    async def _left_set_enabled(self, request, response):
        return await self._set_enabled('left', request, response)

    async def _right_set_enabled(self, request, response):
        return await self._set_enabled('right', request, response)

    async def _set_enabled(self, side: str, request, response):
        if request.enable:
            return await self._handle_start(side, request.reason, response)
        return self._handle_stop(side, request.reason, response)

    async def _handle_start(self, side: str, reason: str, response):
        if reason != 'USER_START':
            response.success = False
            response.message = f'unsupported enable reason: {reason}'
            return response
        with self._lock:
            if self.state == self.RECORDING:
                self.active[side] = True
                response.success = True
                response.message = (
                    f'{side.upper()} joined recording; session_id={self.session_id}'
                )
                self.get_logger().info(response.message)
                return response
            if self.state != self.READY:
                response.success = False
                response.message = f'session not ready: state={self.state}'
                return response
            self._set_state(self.STARTING)

        if not self.recording_client.service_is_ready():
            self._set_state(self.READY)
            response.success = False
            response.message = 'recording service unavailable'
            return response
        try:
            recording_response = await self.recording_client.call_async(
                self._start_request()
            )
        except Exception as exc:
            self._set_state(self.READY)
            response.success = False
            response.message = f'recording START call failed: {exc}'
            return response
        if not recording_response.success:
            self._set_state(self.READY)
            response.success = False
            response.message = (
                f'recording START rejected: {recording_response.message}'
            )
            return response
        if recording_response.state != self.RECORDING:
            self._set_state(self.FAILED)
            response.success = False
            response.message = (
                'recording START returned unexpected state: '
                f'{recording_response.state}'
            )
            return response
        with self._lock:
            if self.state != self.STARTING:
                response.success = False
                response.message = f'START superseded by state={self.state}'
                return response
            self.session_id = recording_response.session_id
            self.active[side] = True
            self._set_state(self.RECORDING)
        response.success = True
        response.message = (
            f'recording started; {side.upper()} teleop allowed; '
            f'session_id={self.session_id}'
        )
        self.get_logger().info(response.message)
        return response

    def _handle_stop(self, side: str, reason: str, response):
        if reason not in ('USER_STOP', *self.ABNORMAL_REASONS):
            response.success = False
            response.message = f'unsupported disable reason: {reason}'
            return response
        with self._lock:
            if self.state in (self.STOPPING, self.RESETTING, self.PREPARING):
                response.success = True
                response.message = f'episode stop already in progress: {self.state}'
                return response
            if self.state != self.RECORDING:
                response.success = True
                response.message = f'no recording episode active: {self.state}'
                return response
            if not self.active[side]:
                response.success = True
                response.message = f'{side.upper()} teleop is not active'
                return response
            self.active = {name: False for name in self.SIDES}
            self._abnormal_stop = reason in self.ABNORMAL_REASONS
            self._stop_reason = reason
            self._set_state(self.STOPPING)
            self.force_stop_publisher.publish(Empty())
            if not self.recording_client.service_is_ready():
                self._set_state(self.FAILED)
                response.success = False
                response.message = 'recording STOP unavailable; session FAILED'
                return response
            request = self._recording_request(ManageRecording.Request.STOP)
            self._stop_future = self.recording_client.call_async(request)
        response.success = True
        response.message = 'episode stop accepted; recording stop/reset in progress'
        self.get_logger().warning(
            f'{side.upper()} {reason}: {response.message}'
        )
        return response

    def _begin_prepare(self) -> None:
        self._prepare_future = None
        self._prepare_retry_at_ns = 0
        self._set_state(self.PREPARING)

    def _poll_prepare(self, now_ns: int) -> None:
        if self._prepare_future is None:
            if now_ns < self._prepare_retry_at_ns:
                return
            if not self.recording_client.service_is_ready():
                self._prepare_retry_at_ns = now_ns + int(
                    self.prepare_retry_sec * 1.0e9
                )
                return
            request = self._recording_request(ManageRecording.Request.PREPARE)
            request.record_cameras = self.record_cameras
            self._prepare_future = self.recording_client.call_async(request)
            return
        if not self._prepare_future.done():
            return
        try:
            result = self._prepare_future.result()
        except Exception as exc:
            result = None
            self.get_logger().warning(f'Recording PREPARE call failed: {exc}')
        self._prepare_future = None
        if result is not None and result.success:
            self.session_id = ''
            self.active = {side: False for side in self.SIDES}
            self._set_state(self.READY)
            self.get_logger().info('Recording PREPARE succeeded; START allowed')
            return
        message = 'no response' if result is None else result.message
        self.get_logger().warning(
            f'Recording PREPARE failed; retrying: {message}'
        )
        self._prepare_retry_at_ns = now_ns + int(
            self.prepare_retry_sec * 1.0e9
        )

    def _poll_stop(self) -> None:
        if self._stop_future is None or not self._stop_future.done():
            return
        try:
            result = self._stop_future.result()
        except Exception as exc:
            result = None
            self.get_logger().error(f'Recording STOP call failed: {exc}')
        self._stop_future = None
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            self.get_logger().error(f'Recording STOP failed: {message}')
            self._set_state(self.FAILED)
            return
        self.get_logger().info('Recording STOP succeeded')
        if self._abnormal_stop:
            self.get_logger().error(
                f'Abnormal {self._stop_reason}: automatic MoveJ is forbidden'
            )
            self._set_state(self.FAILED)
            return
        self._begin_reset()

    def _reset_goal(self, side: str) -> ExecuteMotion.Goal:
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

    def _begin_reset(self) -> None:
        self._set_state(self.RESETTING)
        self._reset_goal_futures = {}
        self._reset_result_futures = {}
        self._reset_failures = {}
        for side in self.SIDES:
            client = self.action_clients[side]
            if not client.server_is_ready():
                self._reset_failures[side] = 'Action server unavailable'
                continue
            self._reset_goal_futures[side] = client.send_goal_async(
                self._reset_goal(side)
            )

    def _poll_reset(self) -> None:
        for side in self.SIDES:
            if side in self._reset_failures:
                continue
            goal_future = self._reset_goal_futures.get(side)
            if goal_future is None or not goal_future.done():
                continue
            if side in self._reset_result_futures:
                continue
            try:
                goal_handle = goal_future.result()
                if not goal_handle.accepted:
                    self._reset_failures[side] = 'goal rejected'
                    continue
                self._reset_result_futures[side] = (
                    goal_handle.get_result_async()
                )
            except Exception as exc:
                self._reset_failures[side] = f'goal send failed: {exc}'

        complete = True
        results = {}
        for side in self.SIDES:
            if side in self._reset_failures:
                continue
            result_future = self._reset_result_futures.get(side)
            if result_future is None or not result_future.done():
                complete = False
                continue
            try:
                results[side] = result_future.result().result
            except Exception as exc:
                self._reset_failures[side] = f'result failed: {exc}'
        if not complete:
            return
        for side, result in results.items():
            if not (
                result.success
                and result.terminal_state == ExecuteMotion.Result.SUCCEEDED
            ):
                self._reset_failures[side] = (
                    'success=%s terminal_state=%s api2_status=%s '
                    'final_joint_degrees=%s message=%s'
                    % (
                        result.success,
                        result.terminal_state,
                        result.api2_status,
                        list(result.final_joint_degrees),
                        result.message,
                    )
                )
        if self._reset_failures:
            for side, detail in self._reset_failures.items():
                self.get_logger().error(
                    f'{side.upper()} reset failed: {detail}'
                )
            self._set_state(self.FAILED)
            return
        self.get_logger().info('LEFT and RIGHT reset Actions succeeded')
        self._begin_prepare()

    def _workflow_tick(self) -> None:
        now_ns = time.monotonic_ns()
        if self.state == self.PREPARING:
            self._poll_prepare(now_ns)
        elif self.state == self.STOPPING:
            self._poll_stop()
        elif self.state == self.RESETTING:
            self._poll_reset()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PikaSessionManager()
    executor = MultiThreadedExecutor(num_threads=4)
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
