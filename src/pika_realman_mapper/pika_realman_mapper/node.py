"""Map Pika session-relative motion from configured RealMan TCP zero poses."""

from dataclasses import dataclass
import math
import time
from typing import Any, Dict, Optional, Tuple

from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32

from pika_teleop_interfaces.msg import PikaTeleopState

from .pose_mapper import PoseMapper, gripper_percentage, pose_values
from .velocity import TargetVelocityEstimator


NANOSECONDS_PER_MILLISECOND = 1_000_000
NANOSECONDS_PER_SECOND = 1_000_000_000


@dataclass
class SideRuntime:
    """Configuration and independent command session for one arm."""

    side: str
    short_name: str
    base_frame: str
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
    last_input_warning_ns: int = 0
    watchdog_active: bool = False
    rearm_required: bool = False
    last_processed_pose_stamp_ns: Optional[int] = None
    target_pose: Optional[Pose] = None

    @property
    def session_initialized(self) -> bool:
        return self.mapper.initialized


class PikaRealManMapper(Node):
    """Generate fixed-zero RealMan targets without reading RealMan TF."""

    SIDES = ('left', 'right')
    DEFAULT_MAP = [0.70710678, 0.0, -0.70710678, 0.0]

    def __init__(self) -> None:
        super().__init__('pika_realman_mapper')
        self._declare_parameters()

        self.command_rate_hz = self._positive_parameter('command_rate_hz')
        self.state_timeout_ms = self._positive_parameter('state_timeout_ms')
        velocity_filter_cutoff_hz = self._positive_parameter(
            'velocity_filter_cutoff_hz'
        )
        velocity_max_dt_ms = self._positive_parameter(
            'velocity_max_dt_ms'
        )
        self._state_timeout_ns = int(
            self.state_timeout_ms * NANOSECONDS_PER_MILLISECOND
        )

        io_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.runtime: Dict[str, SideRuntime] = {}
        for side, short_name in (('left', 'l'), ('right', 'r')):
            base_frame = self._string_parameter(f'{side}_base_frame')
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
                default_tcp_pose=default_tcp,
                mapper=PoseMapper(mapping, scale),
                velocity=TargetVelocityEstimator(
                    velocity_filter_cutoff_hz,
                    velocity_max_dt_ms,
                ),
                gripper_closed=closed,
                gripper_open=opened,
                pose_publisher=self.create_publisher(
                    PoseStamped,
                    f'/pika/{short_name}/cartesian_pose',
                    io_qos,
                ),
                velocity_publisher=self.create_publisher(
                    TwistStamped,
                    f'/pika/{short_name}/cartesian_velocity',
                    io_qos,
                ),
                gripper_publisher=self.create_publisher(
                    Float32,
                    f'/pika/{short_name}/gripper_percentage',
                    io_qos,
                ),
            )

        self._state_subscriptions = [
            self.create_subscription(
                PikaTeleopState,
                f'/pika_teleop/{side}/state',
                lambda message, side=side: self._state_callback(side, message),
                io_qos,
            )
            for side in self.SIDES
        ]
        self._command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self._command_tick,
        )
        self.get_logger().info(
            'Pika RealMan mapper ready: command_rate=%.1f Hz, '
            'state_timeout=%.1f ms, fixed configured TCP zero poses'
            % (self.command_rate_hz, self.state_timeout_ms)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('state_timeout_ms', 100.0)
        self.declare_parameter('left_base_frame', 'l/base_link')
        self.declare_parameter('right_base_frame', 'r/base_link')
        self.declare_parameter('translation_scale_left', 1.0)
        self.declare_parameter('translation_scale_right', 1.0)
        self.declare_parameter(
            'left_base_from_pika_quaternion_xyzw', self.DEFAULT_MAP
        )
        self.declare_parameter(
            'right_base_from_pika_quaternion_xyzw', self.DEFAULT_MAP
        )
        self.declare_parameter('velocity_filter_cutoff_hz', 10.0)
        self.declare_parameter('velocity_max_dt_ms', 50.0)
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

    def _string_parameter(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
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
        if not self._state_values_valid(message):
            self._reset_session(
                runtime,
                'non-finite/invalid input',
                require_rearm=True,
            )
            if now_ns - runtime.last_input_warning_ns >= NANOSECONDS_PER_SECOND:
                runtime.last_input_warning_ns = now_ns
                self.get_logger().warning(
                    f'{side.upper()} invalid Pose/Gripper; command blocked'
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
        runtime.rearm_required = runtime.rearm_required or require_rearm
        if had_session:
            text = f'{runtime.side.upper()} SESSION STOPPED: {reason}'
            if warning:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    def _initialize_session(self, runtime: SideRuntime) -> None:
        message = runtime.latest_state
        if message is None or runtime.rearm_required:
            return
        try:
            runtime.mapper.initialize(message.pose, runtime.default_tcp_pose)
            runtime.target_pose = runtime.mapper.map_pose(message.pose)
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
        runtime.velocity.update(runtime.target_pose, source_stamp_ns)
        runtime.last_processed_pose_stamp_ns = source_stamp_ns
        self.get_logger().info(
            '\n%s SESSION STARTED\n  pika_start=%s\n  rm_default=%s\n'
            '  base_frame=%s'
            % (
                runtime.side.upper(),
                self._pose_text(runtime.mapper.pika_start),
                self._pose_text(runtime.mapper.rm_start),
                runtime.base_frame,
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
            target = runtime.mapper.map_pose(message.pose)
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
            runtime.velocity.update(target, source_stamp_ns)
            runtime.last_processed_pose_stamp_ns = source_stamp_ns
        runtime.target_pose = target

        pose_message = PoseStamped()
        pose_message.header.stamp = command_stamp
        pose_message.header.frame_id = runtime.base_frame
        pose_message.pose = target
        velocity_message = TwistStamped()
        velocity_message.header.stamp = command_stamp
        velocity_message.header.frame_id = runtime.base_frame
        velocity_message.twist = runtime.velocity.twist
        runtime.pose_publisher.publish(pose_message)
        runtime.velocity_publisher.publish(velocity_message)
        runtime.gripper_publisher.publish(Float32(data=percentage))

    def _command_tick(self) -> None:
        now_ns = time.monotonic_ns()
        command_stamp = self.get_clock().now().to_msg()
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
