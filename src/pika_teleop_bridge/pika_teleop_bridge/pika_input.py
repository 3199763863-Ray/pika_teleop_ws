"""Thread-safe latest-value adapter for the four official Pika topics."""

from dataclasses import dataclass
import threading
from typing import Any, Dict, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState


NANOSECONDS_PER_SECOND = 1_000_000_000
MILLISECONDS_PER_NANOSECOND = 1.0e-6


@dataclass(frozen=True)
class TimedSample:
    """One latest message as observed at a control tick."""

    msg: Optional[Any]
    source_time_ns: Optional[int]
    receipt_time_ns: Optional[int]
    age_ms: Optional[float]
    receipt_age_ms: Optional[float]
    valid: bool
    source_stamp_valid: bool
    sequence: int
    new_sample: bool


@dataclass(frozen=True)
class PikaSnapshot:
    """Latest four-channel Pika state at one control time."""

    control_time_ns: int
    left_pose: TimedSample
    right_pose: TimedSample
    left_gripper: TimedSample
    right_gripper: TimedSample


@dataclass
class _CachedSample:
    msg: Optional[Any] = None
    source_time_ns: Optional[int] = None
    receipt_time_ns: Optional[int] = None
    source_stamp_valid: bool = False
    sequence: int = 0


class PikaInputAdapter:
    """Cache latest Pika inputs without synchronization or bridge semantics."""

    SOURCE_NAMES = (
        'left_pose',
        'right_pose',
        'left_gripper',
        'right_gripper',
    )

    DEFAULT_TOPICS = {
        'left_pose': '/pika_pose_l',
        'right_pose': '/pika_pose_r',
        'left_gripper': '/gripper_l/joint_state',
        'right_gripper': '/gripper_r/joint_state',
    }

    def __init__(
        self,
        node: Node,
        *,
        topics: Optional[Dict[str, str]] = None,
        qos_profile: Optional[QoSProfile] = None,
        callback_group: Optional[CallbackGroup] = None,
    ) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._latest = {name: _CachedSample() for name in self.SOURCE_NAMES}
        self._last_snapshot_seen_sequence = {
            name: 0 for name in self.SOURCE_NAMES
        }

        resolved_topics = dict(self.DEFAULT_TOPICS)
        if topics:
            unknown = set(topics) - set(self.SOURCE_NAMES)
            if unknown:
                raise ValueError(f'Unknown Pika sources: {sorted(unknown)}')
            resolved_topics.update(topics)
        self.topics = resolved_topics

        qos = qos_profile or QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        subscription_kwargs = {}
        if callback_group is not None:
            subscription_kwargs['callback_group'] = callback_group

        self._subscriptions = [
            node.create_subscription(
                PoseStamped,
                self.topics['left_pose'],
                self._left_pose_callback,
                qos,
                **subscription_kwargs,
            ),
            node.create_subscription(
                PoseStamped,
                self.topics['right_pose'],
                self._right_pose_callback,
                qos,
                **subscription_kwargs,
            ),
            node.create_subscription(
                JointState,
                self.topics['left_gripper'],
                self._left_gripper_callback,
                qos,
                **subscription_kwargs,
            ),
            node.create_subscription(
                JointState,
                self.topics['right_gripper'],
                self._right_gripper_callback,
                qos,
                **subscription_kwargs,
            ),
        ]

    @staticmethod
    def _extract_source_time_ns(msg: Any) -> tuple[Optional[int], bool]:
        try:
            stamp = msg.header.stamp
            source_time_ns = (
                int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)
            )
        except (AttributeError, TypeError, ValueError):
            return None, False
        return source_time_ns, source_time_ns > 0

    def _store(self, source_name: str, msg: Any) -> None:
        receipt_time_ns = self._node.get_clock().now().nanoseconds
        source_time_ns, source_stamp_valid = self._extract_source_time_ns(msg)
        with self._lock:
            cached = self._latest[source_name]
            cached.msg = msg
            cached.source_time_ns = source_time_ns
            cached.receipt_time_ns = receipt_time_ns
            cached.source_stamp_valid = source_stamp_valid
            cached.sequence += 1

    def _left_pose_callback(self, msg: PoseStamped) -> None:
        self._store('left_pose', msg)

    def _right_pose_callback(self, msg: PoseStamped) -> None:
        self._store('right_pose', msg)

    def _left_gripper_callback(self, msg: JointState) -> None:
        self._store('left_gripper', msg)

    def _right_gripper_callback(self, msg: JointState) -> None:
        self._store('right_gripper', msg)

    @staticmethod
    def _make_timed_sample(
        cached: _CachedSample,
        control_time_ns: int,
        new_sample: bool,
    ) -> TimedSample:
        age_ms = None
        if cached.source_stamp_valid and cached.source_time_ns is not None:
            age_ms = (
                control_time_ns - cached.source_time_ns
            ) * MILLISECONDS_PER_NANOSECOND
        receipt_age_ms = None
        if cached.receipt_time_ns is not None:
            receipt_age_ms = (
                control_time_ns - cached.receipt_time_ns
            ) * MILLISECONDS_PER_NANOSECOND
        return TimedSample(
            msg=cached.msg,
            source_time_ns=cached.source_time_ns,
            receipt_time_ns=cached.receipt_time_ns,
            age_ms=age_ms,
            receipt_age_ms=receipt_age_ms,
            valid=cached.msg is not None,
            source_stamp_valid=cached.source_stamp_valid,
            sequence=cached.sequence,
            new_sample=new_sample,
        )

    def get_snapshot(self, control_time_ns: Optional[int] = None) -> PikaSnapshot:
        """Return latest four samples immediately, without waiting for pairing."""
        if control_time_ns is None:
            control_time_ns = self._node.get_clock().now().nanoseconds
        with self._lock:
            cached_copy = {
                name: _CachedSample(
                    msg=value.msg,
                    source_time_ns=value.source_time_ns,
                    receipt_time_ns=value.receipt_time_ns,
                    source_stamp_valid=value.source_stamp_valid,
                    sequence=value.sequence,
                )
                for name, value in self._latest.items()
            }
            is_new = {}
            for name, value in cached_copy.items():
                is_new[name] = (
                    value.sequence > self._last_snapshot_seen_sequence[name]
                )
                self._last_snapshot_seen_sequence[name] = value.sequence

        samples = {
            name: self._make_timed_sample(
                cached_copy[name], control_time_ns, is_new[name]
            )
            for name in self.SOURCE_NAMES
        }
        return PikaSnapshot(
            control_time_ns=control_time_ns,
            left_pose=samples['left_pose'],
            right_pose=samples['right_pose'],
            left_gripper=samples['left_gripper'],
            right_gripper=samples['right_gripper'],
        )
