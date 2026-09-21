"""Causal Base-frame target velocity estimator."""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Pose, Twist

from .pose_mapper import pose_values
from .quaternion_utils import inverse, multiply, shortest


class TargetVelocityEstimator:
    """Differentiate consecutive new target poses using source timestamps."""

    def __init__(self, cutoff_hz: float, max_dt_ms: float) -> None:
        self.cutoff_hz = float(cutoff_hz)
        max_dt_ms = float(max_dt_ms)
        if not math.isfinite(self.cutoff_hz) or self.cutoff_hz <= 0.0:
            raise ValueError('cutoff_hz must be finite and positive')
        if not math.isfinite(max_dt_ms) or max_dt_ms <= 0.0:
            raise ValueError('max_dt_ms must be finite and positive')
        self.max_dt_ns = int(max_dt_ms * 1.0e6)
        self._tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        self.reset()

    @property
    def twist(self) -> Twist:
        return self._twist

    @property
    def valid(self) -> bool:
        return self._valid

    def reset(self) -> None:
        self._previous_pose = None
        self._previous_stamp_ns: Optional[int] = None
        self._filtered = [0.0] * 6
        self._twist = Twist()
        self._valid = False

    def _rebaseline(self, pose_values_now, stamp_ns: int) -> bool:
        self._previous_pose = pose_values_now
        self._previous_stamp_ns = stamp_ns
        self._filtered = [0.0] * 6
        self._twist = Twist()
        self._valid = False
        return False

    def update(self, target_pose: Pose, source_stamp_ns: int) -> bool:
        try:
            current = pose_values(target_pose)
        except ValueError:
            self.reset()
            return False
        if source_stamp_ns is None or int(source_stamp_ns) <= 0:
            self.reset()
            return False
        stamp_ns = int(source_stamp_ns)
        if self._previous_pose is None or self._previous_stamp_ns is None:
            return self._rebaseline(current, stamp_ns)

        dt_ns = stamp_ns - self._previous_stamp_ns
        if dt_ns <= 0 or dt_ns > self.max_dt_ns:
            return self._rebaseline(current, stamp_ns)
        dt = dt_ns * 1.0e-9
        current_position, current_orientation = current
        previous_position, previous_orientation = self._previous_pose

        linear = tuple(
            (now - before) / dt
            for now, before in zip(current_position, previous_position)
        )
        rotation_delta = shortest(
            multiply(current_orientation, inverse(previous_orientation))
        )
        vector = rotation_delta[:3]
        vector_norm = math.sqrt(sum(value * value for value in vector))
        if vector_norm <= 1.0e-12:
            angular = (0.0, 0.0, 0.0)
        else:
            angle = 2.0 * math.atan2(vector_norm, rotation_delta[3])
            angular = tuple(
                component / vector_norm * angle / dt
                for component in vector
            )
        raw = (*linear, *angular)
        if not all(math.isfinite(value) for value in raw):
            self.reset()
            return False

        alpha = dt / (self._tau + dt)
        self._filtered = [
            previous + alpha * (current_value - previous)
            for previous, current_value in zip(self._filtered, raw)
        ]
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self._filtered[:3]
        twist.angular.x, twist.angular.y, twist.angular.z = self._filtered[3:]
        self._twist = twist
        self._valid = True
        self._previous_pose = current
        self._previous_stamp_ns = stamp_ns
        return True
