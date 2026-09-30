"""Map Pika motion through a calibrated shared base into RealMan targets."""

from dataclasses import dataclass
import math
import time
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from pika_teleop_interfaces.msg import PikaTeleopState

from .pika_base import PikaBaseCalibrator
from .pose_mapper import (
    PoseMapper,
    gripper_percentage,
    make_pose,
    pose_values,
)
from .velocity import TargetVelocityEstimator


NANOSECONDS_PER_MILLISECOND = 1_000_000
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass
class SideRuntime:
    """Configuration and independent command session for one arm."""

    side: str
    short_name: str
    base_frame: str
    tcp_frame: str
    pika_base_frame: str
    velocity_frame: str
    default_tcp_pose: Pose
    mapper: PoseMapper
    velocity: TargetVelocityEstimator
    gripper_closed: float
    gripper_open: float
    pose_publisher: Any
    velocity_publisher: Any
    gripper_publisher: Any
    latest_state: Optional[PikaTeleopState] = None
    last_receipt_ns: Optional[int] = None
    invalid_input_active: bool = False
    watchdog_active: bool = False
    rearm_required: bool = False
    last_processed_pose_stamp_ns: Optional[int] = None
    target_pose: Optional[Pose] = None
    gripper_filtered: Optional[float] = None
    gripper_last_publish_ns: int = 0
    pika_base_translation: Optional[Tuple[float, float, float]] = None
    tf_waiting: bool = False

    @property
    def session_initialized(self) -> bool:
        return self.mapper.initialized


class PikaRealManMapper(Node):
    """Generate session-relative RealMan targets from a shared Pika base."""

    SIDES = ('left', 'right')
    DEFAULT_MAP = [0.0, 0.0, 0.0, 1.0]

    def __init__(self) -> None:
        super().__init__('pika_realman_mapper')
        self._declare_parameters()

        self.command_rate_hz = self._positive_parameter('command_rate_hz')
        self.state_timeout_ms = self._positive_parameter('state_timeout_ms')
        self.require_realman_start_tf = self._bool_parameter(
            'require_realman_start_tf'
        )
        self._pika_pair_max_stamp_delta_ns = int(
            self._positive_parameter('pika_base_pair_max_stamp_delta_ms')
            * NANOSECONDS_PER_MILLISECOND
        )
        self._pika_base = PikaBaseCalibrator(
            stable_samples=self._integer_parameter(
                'pika_base_calibration_stable_samples', minimum=2
            ),
            max_position_spread_m=self._positive_parameter(
                'pika_base_calibration_max_spread_m'
            ),
            min_controller_separation_m=self._positive_parameter(
                'pika_base_min_controller_separation_m'
            ),
            vertical_axis_in_teleop_frame=self._array_parameter(
                'pika_vertical_axis_in_teleop_frame_xyz', 3
            ),
        )
        gripper_filter_cutoff_hz = self._positive_parameter(
            'gripper_filter_cutoff_hz'
        )
        self._gripper_publish_period_ns = int(
            1e9 / self._positive_parameter('gripper_publish_rate_hz')
        )
        command_dt = 1.0 / self.command_rate_hz
        gripper_tau = 1.0 / (2.0 * math.pi * gripper_filter_cutoff_hz)
        self._gripper_alpha = command_dt / (gripper_tau + command_dt)
        velocity_filter_cutoff_hz = self._positive_parameter(
            'velocity_filter_cutoff_hz'
        )
        velocity_min_dt_ms = self._positive_parameter(
            'velocity_min_dt_ms'
        )
        velocity_max_dt_ms = self._positive_parameter(
            'velocity_max_dt_ms'
        )
        if velocity_min_dt_ms >= velocity_max_dt_ms:
            raise ValueError(
                'velocity_min_dt_ms must be less than velocity_max_dt_ms'
            )
        velocity_derivative_window_samples = self._integer_parameter(
            'velocity_derivative_window_samples', minimum=2
        )
        velocity_kalman_enabled = self._bool_parameter(
            'velocity_kalman_enabled'
        )
        linear_velocity_kalman_process_variance = (
            self._positive_parameter(
                'linear_velocity_kalman_process_variance'
            )
        )
        linear_velocity_kalman_measurement_variance = (
            self._positive_parameter(
                'linear_velocity_kalman_measurement_variance'
            )
        )
        angular_velocity_kalman_process_variance = (
            self._positive_parameter(
                'angular_velocity_kalman_process_variance'
            )
        )
        angular_velocity_kalman_measurement_variance = (
            self._positive_parameter(
                'angular_velocity_kalman_measurement_variance'
            )
        )
        orientation_mapping_mode = self._choice_parameter(
            'orientation_mapping_mode', PoseMapper.VALID_ORIENTATION_MODES
        )
        linear_velocity_deadband_mps = self._nonnegative_parameter(
            'linear_velocity_deadband_mps'
        )
        angular_velocity_deadband_radps = self._nonnegative_parameter(
            'angular_velocity_deadband_radps'
        )
        self._state_timeout_ns = int(
            self.state_timeout_ms * NANOSECONDS_PER_MILLISECOND
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.runtime: Dict[str, SideRuntime] = {}
        for side, short_name in (('left', 'l'), ('right', 'r')):
            base_frame = self._string_parameter(f'{side}_base_frame')
            tcp_frame = self._string_parameter(f'{side}_tcp_frame')
            pika_base_frame = self._string_parameter(
                f'{side}_pika_base_frame'
            )
            velocity_frame = self._string_parameter(f'{side}_velocity_frame')
            mapping = self._quaternion_parameter(
                f'{side}_base_from_pika_quaternion_xyzw'
            )
            scale = self._positive_parameter(f'translation_scale_{side}')
            closed = self._finite_parameter(
                f'{side}_gripper_closed_position'
            )
            opened = self._finite_parameter(
                f'{side}_gripper_open_position'
            )
            if opened <= closed:
                raise ValueError(
                    f'{side}_gripper_open_position must exceed '
                    f'{side}_gripper_closed_position'
                )
            default_tcp = self._default_tcp_pose(side)
            self.runtime[side] = SideRuntime(
                side=side,
                short_name=short_name,
                base_frame=base_frame,
                tcp_frame=tcp_frame,
                pika_base_frame=pika_base_frame,
                velocity_frame=velocity_frame,
                default_tcp_pose=default_tcp,
                mapper=PoseMapper(mapping, scale, orientation_mapping_mode),
                velocity=TargetVelocityEstimator(
                    cutoff_hz=velocity_filter_cutoff_hz,
                    min_dt_ms=velocity_min_dt_ms,
                    max_dt_ms=velocity_max_dt_ms,
                    derivative_window_samples=(
                        velocity_derivative_window_samples
                    ),
                    kalman_enabled=velocity_kalman_enabled,
                    linear_kalman_process_variance=(
                        linear_velocity_kalman_process_variance
                    ),
                    linear_kalman_measurement_variance=(
                        linear_velocity_kalman_measurement_variance
                    ),
                    angular_kalman_process_variance=(
                        angular_velocity_kalman_process_variance
                    ),
                    angular_kalman_measurement_variance=(
                        angular_velocity_kalman_measurement_variance
                    ),
                    linear_deadband_mps=linear_velocity_deadband_mps,
                    angular_deadband_radps=angular_velocity_deadband_radps,
                ),
                gripper_closed=closed,
                gripper_open=opened,
                pose_publisher=self.create_publisher(
                    PoseStamped,
                    f'/pika/{short_name}/cartesian_pose',
                    command_qos,
                ),
                velocity_publisher=self.create_publisher(
                    TwistStamped,
                    f'/pika/{short_name}/cartesian_velocity',
                    command_qos,
                ),
                gripper_publisher=self.create_publisher(
                    Float32,
                    f'/pika/{short_name}/gripper_percentage',
                    command_qos,
                ),
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
        self._command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self._command_tick,
        )
        self.get_logger().info(
            'Pika RealMan mapper ready: command_rate=%.1f Hz, '
            'state_timeout=%.1f ms, command_qos=RELIABLE, '
            'start_tf=%s, tcp_frames=(%s,%s), pika_base_frames=(%s,%s), '
            'orientation_mapping_mode=%s, velocity_frames=(%s,%s), '
            'velocity_dt=(%.1f..%.1f ms, window=%d), kalman=%s, '
            'velocity_deadband=(%.3f m/s, %.3f rad/s), '
            'configured pose mapping'
            % (
                self.command_rate_hz,
                self.state_timeout_ms,
                'required' if self.require_realman_start_tf else 'fallback',
                self.runtime['left'].tcp_frame,
                self.runtime['right'].tcp_frame,
                self.runtime['left'].pika_base_frame,
                self.runtime['right'].pika_base_frame,
                orientation_mapping_mode,
                self.runtime['left'].velocity_frame,
                self.runtime['right'].velocity_frame,
                velocity_min_dt_ms,
                velocity_max_dt_ms,
                velocity_derivative_window_samples,
                velocity_kalman_enabled,
                linear_velocity_deadband_mps,
                angular_velocity_deadband_radps,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('state_timeout_ms', 100.0)
        self.declare_parameter('require_realman_start_tf', True)
        self.declare_parameter('left_tcp_frame', 'l/link_6')
        self.declare_parameter('right_tcp_frame', 'r/link_6')
        self.declare_parameter('left_pika_base_frame', 'l_pika_base_link')
        self.declare_parameter('right_pika_base_frame', 'r_pika_base_link')
        self.declare_parameter('pika_base_calibration_stable_samples', 10)
        self.declare_parameter('pika_base_calibration_max_spread_m', 0.01)
        self.declare_parameter('pika_base_min_controller_separation_m', 0.10)
        self.declare_parameter('pika_base_pair_max_stamp_delta_ms', 100.0)
        # The bridge maps raw Pika (x,y,z) to (-z,y,x), so raw base +Z is
        # the -X direction in pika_teleop_frame.
        self.declare_parameter(
            'pika_vertical_axis_in_teleop_frame_xyz', [-1.0, 0.0, 0.0]
        )
        self.declare_parameter('gripper_filter_cutoff_hz', 10.0)
        # The Changingtek gripper restarts its motion profile on every trigger,
        # so targets faster than ~4 Hz stall it; publish the gripper at this
        # rate.
        self.declare_parameter('gripper_publish_rate_hz', 4.0)
        self.declare_parameter('left_base_frame', 'l/base_link')
        self.declare_parameter('right_base_frame', 'r/base_link')
        self.declare_parameter('left_velocity_frame', 'l/work/pikabase')
        self.declare_parameter('right_velocity_frame', 'r/work/pikabase')
        self.declare_parameter('translation_scale_left', 1.0)
        self.declare_parameter('translation_scale_right', 1.0)
        self.declare_parameter('orientation_mapping_mode', 'relative')
        self.declare_parameter(
            'left_base_from_pika_quaternion_xyzw', self.DEFAULT_MAP
        )
        self.declare_parameter(
            'right_base_from_pika_quaternion_xyzw', self.DEFAULT_MAP
        )
        self.declare_parameter('velocity_filter_cutoff_hz', 10.0)
        self.declare_parameter('velocity_min_dt_ms', 5.0)
        self.declare_parameter('velocity_max_dt_ms', 50.0)
        self.declare_parameter('velocity_derivative_window_samples', 2)
        self.declare_parameter('velocity_kalman_enabled', False)
        self.declare_parameter(
            'linear_velocity_kalman_process_variance', 0.2
        )
        self.declare_parameter(
            'linear_velocity_kalman_measurement_variance', 0.05
        )
        self.declare_parameter(
            'angular_velocity_kalman_process_variance', 0.5
        )
        self.declare_parameter(
            'angular_velocity_kalman_measurement_variance', 0.1
        )
        self.declare_parameter('linear_velocity_deadband_mps', 0.02)
        self.declare_parameter('angular_velocity_deadband_radps', 0.03)
        self.declare_parameter('left_gripper_closed_position', 0.0)
        self.declare_parameter('left_gripper_open_position', 0.1)
        self.declare_parameter('right_gripper_closed_position', 0.0)
        self.declare_parameter('right_gripper_open_position', 0.1)
        for side in self.SIDES:
            self.declare_parameter(
                f'{side}_default_tcp_position_m',
                Parameter.Type.DOUBLE_ARRAY,
            )
            self.declare_parameter(
                f'{side}_default_tcp_orientation_xyzw',
                Parameter.Type.DOUBLE_ARRAY,
            )

    def _finite_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        return value

    def _positive_parameter(self, name: str) -> float:
        value = self._finite_parameter(name)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = self._finite_parameter(name)
        if value < 0.0:
            raise ValueError(f'{name} must be non-negative')
        return value

    def _integer_parameter(self, name: str, minimum: int) -> int:
        raw = self.get_parameter(name).value
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ValueError(f'{name} must be an integer')
        if raw < minimum:
            raise ValueError(f'{name} must be at least {minimum}')
        return raw

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise ValueError(f'{name} must be true or false')
        return value

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _choice_parameter(
        self, name: str, choices: Tuple[str, ...]
    ) -> str:
        value = self._string_parameter(name).lower()
        if value not in choices:
            raise ValueError(
                f'{name} must be one of: {", ".join(choices)}'
            )
        return value

    def _array_parameter(self, name: str, size: int) -> Tuple[float, ...]:
        raw = self.get_parameter(name).value
        values = () if raw is None else tuple(float(value) for value in raw)
        if len(values) != size:
            raise ValueError(f'{name} must contain exactly {size} values')
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain only finite values')
        return values

    def _quaternion_parameter(self, name: str) -> Tuple[float, ...]:
        return self._array_parameter(name, 4)

    def _default_tcp_pose(self, side: str) -> Pose:
        position = self._array_parameter(
            f'{side}_default_tcp_position_m', 3
        )
        orientation = self._array_parameter(
            f'{side}_default_tcp_orientation_xyzw', 4
        )
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = orientation
        normalized_position, normalized_orientation = pose_values(pose)
        pose.position.x, pose.position.y, pose.position.z = normalized_position
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = normalized_orientation
        return pose

    @staticmethod
    def _source_stamp_ns(message: PikaTeleopState) -> int:
        stamp = message.pose_source_stamp
        return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)

    def _update_pika_base_calibration(self, now_ns: int) -> None:
        if self._pika_base.calibrated:
            return
        left = self.runtime['left']
        right = self.runtime['right']
        if (
            left.latest_state is None
            or right.latest_state is None
            or left.last_receipt_ns is None
            or right.last_receipt_ns is None
        ):
            return
        if (
            now_ns - left.last_receipt_ns > self._state_timeout_ns
            or now_ns - right.last_receipt_ns > self._state_timeout_ns
        ):
            return
        left_stamp = self._source_stamp_ns(left.latest_state)
        right_stamp = self._source_stamp_ns(right.latest_state)
        if (
            left_stamp <= 0
            or right_stamp <= 0
            or abs(left_stamp - right_stamp)
            > self._pika_pair_max_stamp_delta_ns
        ):
            return
        try:
            calibrated_now = self._pika_base.add_sample(
                left.latest_state.pose,
                right.latest_state.pose,
                (left_stamp, right_stamp),
            )
        except ValueError:
            return
        if calibrated_now:
            self.get_logger().info(
                'PIKA BASE CALIBRATED: origin=(%.4f,%.4f,%.4f) '
                'X=(%.4f,%.4f,%.4f) Y=(%.4f,%.4f,%.4f) '
                'Z=(%.4f,%.4f,%.4f)'
                % (
                    *self._pika_base.origin,
                    *self._pika_base.x_axis,
                    *self._pika_base.y_axis,
                    *self._pika_base.z_axis,
                )
            )

    def _pika_base_pose(self, message: PikaTeleopState) -> Pose:
        return self._pika_base.transform_pose(message.pose)

    def _realman_start_pose(self, runtime: SideRuntime) -> Optional[Pose]:
        if not self.require_realman_start_tf:
            return runtime.default_tcp_pose
        try:
            transform = self._tf_buffer.lookup_transform(
                runtime.base_frame,
                runtime.tcp_frame,
                Time(),
            )
        except TransformException as exc:
            if not runtime.tf_waiting:
                self.get_logger().warning(
                    '%s START WAITING: TF %s <- %s unavailable: %s'
                    % (
                        runtime.side.upper(),
                        runtime.base_frame,
                        runtime.tcp_frame,
                        exc,
                    )
                )
                runtime.tf_waiting = True
            return None
        if runtime.tf_waiting:
            self.get_logger().info(
                '%s START TF RECOVERED: %s <- %s'
                % (
                    runtime.side.upper(),
                    runtime.base_frame,
                    runtime.tcp_frame,
                )
            )
            runtime.tf_waiting = False
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        try:
            return make_pose(pose_values(pose))
        except ValueError as exc:
            if not runtime.tf_waiting:
                self.get_logger().warning(
                    '%s START WAITING: invalid TCP TF: %s'
                    % (runtime.side.upper(), exc)
                )
                runtime.tf_waiting = True
            return None

    @staticmethod
    def _velocity_pose(target_pose: Pose, pika_base_pose: Pose) -> Pose:
        target_position, _ = pose_values(target_pose)
        _, pika_orientation = pose_values(pika_base_pose)
        return make_pose((target_position, pika_orientation))

    def _broadcast_pika_base(
        self,
        runtime: SideRuntime,
        command_stamp,
    ) -> None:
        if runtime.pika_base_translation is None:
            return
        transform = TransformStamped()
        transform.header.stamp = command_stamp
        transform.header.frame_id = runtime.base_frame
        transform.child_frame_id = runtime.pika_base_frame
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ) = runtime.pika_base_translation
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _state_values_valid(message: PikaTeleopState) -> bool:
        try:
            pose_values(message.pose)
        except ValueError:
            return False
        return math.isfinite(float(message.gripper_position))

    @staticmethod
    def _pose_text(values) -> str:
        position, orientation = values
        return 'p=(%.4f,%.4f,%.4f) q=(%.4f,%.4f,%.4f,%.4f)' % (
            *position,
            *orientation,
        )

    def _state_callback(self, side: str, message: PikaTeleopState) -> None:
        runtime = self.runtime[side]
        now_ns = time.monotonic_ns()
        runtime.latest_state = message
        runtime.last_receipt_ns = now_ns
        invalid_input = (
            bool(message.enabled and message.valid)
            and not self._state_values_valid(message)
        )
        if invalid_input and not runtime.invalid_input_active:
            self.get_logger().warning(
                f'{side.upper()} invalid Pose/Gripper; command blocked'
            )
        elif not invalid_input and runtime.invalid_input_active:
            self.get_logger().info(f'{side.upper()} Pose/Gripper recovered')
        runtime.invalid_input_active = invalid_input
        if runtime.watchdog_active:
            runtime.watchdog_active = False
            suffix = (
                '; waiting for enabled=false before re-arm'
                if runtime.rearm_required
                else ''
            )
            self.get_logger().info(
                f'{side.upper()} state stream recovered{suffix}'
            )
        if not message.enabled:
            self._reset_session(runtime, 'state disabled')
            runtime.rearm_required = False
            return
        if not message.valid:
            self._reset_session(
                runtime, 'state invalid', require_rearm=True
            )
            return
        if invalid_input:
            self._reset_session(
                runtime,
                'non-finite/invalid input',
                require_rearm=True,
            )

    def _reset_session(
        self,
        runtime: SideRuntime,
        reason: str,
        *,
        require_rearm: bool = False,
        warning: bool = False,
    ) -> None:
        had_session = runtime.session_initialized
        runtime.mapper.reset()
        runtime.velocity.reset()
        runtime.last_processed_pose_stamp_ns = None
        runtime.target_pose = None
        runtime.pika_base_translation = None
        runtime.tf_waiting = False
        runtime.rearm_required = runtime.rearm_required or require_rearm
        if had_session:
            text = f'{runtime.side.upper()} SESSION STOPPED: {reason}'
            if warning:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    def _initialize_session(self, runtime: SideRuntime) -> None:
        message = runtime.latest_state
        if (
            message is None
            or runtime.rearm_required
            or not self._pika_base.calibrated
        ):
            return
        rm_start = self._realman_start_pose(runtime)
        if rm_start is None:
            return
        try:
            pika_start = self._pika_base_pose(message)
            runtime.mapper.initialize(pika_start, rm_start)
            runtime.target_pose = runtime.mapper.map_pose(pika_start)
            pika_start_position, _ = pose_values(pika_start)
            rm_start_position, _ = pose_values(rm_start)
            runtime.pika_base_translation = tuple(
                rm_value - pika_value
                for rm_value, pika_value in zip(
                    rm_start_position,
                    pika_start_position,
                )
            )
        except (ValueError, RuntimeError) as exc:
            self._reset_session(
                runtime,
                f'reference error: {exc}',
                require_rearm=True,
                warning=True,
            )
            return
        runtime.velocity.reset()
        source_stamp_ns = self._source_stamp_ns(message)
        runtime.velocity.update(
            self._velocity_pose(runtime.target_pose, pika_start),
            source_stamp_ns,
        )
        runtime.last_processed_pose_stamp_ns = source_stamp_ns
        self.get_logger().info(
            '\n%s SESSION STARTED\n  orientation_mapping_mode=%s\n'
            '  pika_start=%s\n  rm_tcp_start=%s\n  tf=%s -> %s'
            % (
                runtime.side.upper(),
                runtime.mapper.orientation_mode,
                self._pose_text(runtime.mapper.pika_start),
                self._pose_text(runtime.mapper.rm_start),
                runtime.base_frame,
                runtime.pika_base_frame,
            )
        )

    def _watchdog_ok(self, runtime: SideRuntime, now_ns: int) -> bool:
        if runtime.last_receipt_ns is None:
            return False
        if now_ns - runtime.last_receipt_ns <= self._state_timeout_ns:
            return True
        if runtime.session_initialized:
            self._reset_session(
                runtime,
                'state watchdog timeout',
                require_rearm=True,
                warning=True,
            )
            runtime.watchdog_active = True
        return False

    def _publish_commands(
        self,
        runtime: SideRuntime,
        message: PikaTeleopState,
        command_stamp,
    ) -> None:
        try:
            pika_pose = self._pika_base_pose(message)
            target = runtime.mapper.map_pose(pika_pose)
            percentage = gripper_percentage(
                message.gripper_position,
                runtime.gripper_closed,
                runtime.gripper_open,
            )
        except (ValueError, RuntimeError) as exc:
            self._reset_session(
                runtime,
                f'mapping error: {exc}',
                require_rearm=True,
                warning=True,
            )
            return
        source_stamp_ns = self._source_stamp_ns(message)
        if source_stamp_ns != runtime.last_processed_pose_stamp_ns:
            runtime.velocity.update(
                self._velocity_pose(target, pika_pose),
                source_stamp_ns,
            )
            runtime.last_processed_pose_stamp_ns = source_stamp_ns
        runtime.target_pose = target

        if runtime.gripper_filtered is None:
            runtime.gripper_filtered = percentage
        else:
            runtime.gripper_filtered += self._gripper_alpha * (
                percentage - runtime.gripper_filtered
            )

        pose_message = PoseStamped()
        pose_message.header.stamp = command_stamp
        pose_message.header.frame_id = runtime.base_frame
        pose_message.pose = target
        velocity_message = TwistStamped()
        velocity_message.header.stamp = command_stamp
        velocity_message.header.frame_id = runtime.velocity_frame
        velocity_message.twist = runtime.velocity.twist
        runtime.pose_publisher.publish(pose_message)
        runtime.velocity_publisher.publish(velocity_message)
        self._broadcast_pika_base(runtime, command_stamp)
        now_ns = time.monotonic_ns()
        if (
            now_ns - runtime.gripper_last_publish_ns
            >= self._gripper_publish_period_ns
        ):
            runtime.gripper_publisher.publish(
                Float32(data=runtime.gripper_filtered)
            )
            runtime.gripper_last_publish_ns = now_ns

    def _command_tick(self) -> None:
        now_ns = time.monotonic_ns()
        command_stamp = self.get_clock().now().to_msg()
        self._update_pika_base_calibration(now_ns)
        for side in self.SIDES:
            runtime = self.runtime[side]
            if not self._watchdog_ok(runtime, now_ns):
                continue
            message = runtime.latest_state
            if message is None or not message.enabled or not message.valid:
                continue
            if not self._state_values_valid(message):
                continue
            if not runtime.session_initialized:
                self._initialize_session(runtime)
            if not runtime.session_initialized:
                continue
            self._publish_commands(runtime, message, command_stamp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PikaRealManMapper()
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
