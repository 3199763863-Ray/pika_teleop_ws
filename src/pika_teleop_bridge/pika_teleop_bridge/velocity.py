"""Timestamp-based Cartesian velocity estimation for teleoperation poses."""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Pose, Twist


QuaternionTuple = Tuple[float, float, float, float]


class VelocityEstimator:
    """Estimate and low-pass-filter twist from consecutive new pose samples."""

    def __init__(self, cutoff_hz: float, max_dt_ms: float) -> None:
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0.0:
            raise ValueError('cutoff_hz must be finite and positive')
        if not math.isfinite(max_dt_ms) or max_dt_ms <= 0.0:
            raise ValueError('max_dt_ms must be finite and positive')
        self.cutoff_hz = float(cutoff_hz)
        self.max_dt_ns = int(max_dt_ms * 1.0e6)
        self._tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        self._previous_pose: Optional[Pose] = None
        self._previous_stamp_ns: Optional[int] = None
        self._filtered = [0.0] * 6
        self._twist = Twist()
        self._valid = False

    @property
    def twist(self) -> Twist:
        return self._twist

    @property
    def valid(self) -> bool:
        return self._valid

    @staticmethod
    def _quaternion(pose: Pose) -> QuaternionTuple:
        values = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in values))
        if not math.isfinite(norm) or norm <= 1.0e-12:
            raise ValueError('Quaternion is not normalizable')
        return tuple(value / norm for value in values)

    @staticmethod
    def _multiply(left: QuaternionTuple, right: QuaternionTuple) -> QuaternionTuple:
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def _conjugate(quaternion: QuaternionTuple) -> QuaternionTuple:
        x, y, z, w = quaternion
        return (-x, -y, -z, w)

    @classmethod
    def _rotate_vector(
        cls,
        quaternion: QuaternionTuple,
        vector: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        rotated = cls._multiply(
            cls._multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
            cls._conjugate(quaternion),
        )
        return rotated[:3]

    @staticmethod
    def _pose_finite(pose: Pose) -> bool:
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(float(value)) for value in values)

    def _invalidate_and_rebaseline(self, pose: Optional[Pose], stamp_ns: Optional[int]) -> None:
        self._twist = Twist()
        self._filtered = [0.0] * 6
        self._valid = False
        self._previous_pose = pose
        self._previous_stamp_ns = stamp_ns

    def reset(self) -> None:
        """Discard the session baseline and output an invalid zero twist."""
        self._invalidate_and_rebaseline(None, None)

    def update(self, pose: Pose, source_stamp_ns: Optional[int]) -> bool:
        """Consume exactly one new transformed pose sample."""
        if (
            source_stamp_ns is None
            or source_stamp_ns <= 0
            or not self._pose_finite(pose)
        ):
            self.reset()
            return False
        try:
            current_quaternion = self._quaternion(pose)
        except ValueError:
            self.reset()
            return False

        if self._previous_pose is None or self._previous_stamp_ns is None:
            self._invalidate_and_rebaseline(pose, source_stamp_ns)
            return False

        dt_ns = source_stamp_ns - self._previous_stamp_ns
        if dt_ns <= 0 or dt_ns > self.max_dt_ns:
            self._invalidate_and_rebaseline(pose, source_stamp_ns)
            return False
        dt = dt_ns * 1.0e-9
        previous = self._previous_pose
        try:
            previous_quaternion = self._quaternion(previous)
        except ValueError:
            self._invalidate_and_rebaseline(pose, source_stamp_ns)
            return False

        if sum(a * b for a, b in zip(previous_quaternion, current_quaternion)) < 0.0:
            current_quaternion = tuple(-value for value in current_quaternion)
        delta = self._multiply(
            self._conjugate(previous_quaternion), current_quaternion
        )
        if delta[3] < 0.0:
            delta = tuple(-value for value in delta)
        vector_norm = math.sqrt(sum(value * value for value in delta[:3]))
        if vector_norm <= 1.0e-12:
            angular = (0.0, 0.0, 0.0)
        else:
            angle = 2.0 * math.atan2(vector_norm, max(0.0, delta[3]))
            body_axis = tuple(value / vector_norm for value in delta[:3])
            teleop_axis = self._rotate_vector(previous_quaternion, body_axis)
            angular = tuple(value * angle / dt for value in teleop_axis)
        raw = [
            (float(pose.position.x) - float(previous.position.x)) / dt,
            (float(pose.position.y) - float(previous.position.y)) / dt,
            (float(pose.position.z) - float(previous.position.z)) / dt,
            angular[0],
            angular[1],
            angular[2],
        ]
        if not all(math.isfinite(value) for value in raw):
            self._invalidate_and_rebaseline(pose, source_stamp_ns)
            return False
        alpha = dt / (self._tau + dt)
        self._filtered = [
            old + alpha * (new - old)
            for old, new in zip(self._filtered, raw)
        ]
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self._filtered[:3]
        twist.angular.x, twist.angular.y, twist.angular.z = self._filtered[3:]
        self._twist = twist
        self._valid = True
        self._previous_pose = pose
        self._previous_stamp_ns = source_stamp_ns
        return True
