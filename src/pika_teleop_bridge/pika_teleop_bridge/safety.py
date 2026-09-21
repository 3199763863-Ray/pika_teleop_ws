"""Constant-time pose discontinuity protection for active teleoperation."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Pose


@dataclass(frozen=True)
class PoseJumpResult:
    """Distance and shortest rotation between two consecutive poses."""

    triggered: bool
    position_jump_m: float
    rotation_jump_deg: float


class PoseJumpGuard:
    """Detect excessive position or rotation changes between new poses."""

    def __init__(
        self,
        max_position_jump_m: float,
        max_rotation_jump_deg: float,
    ) -> None:
        if not math.isfinite(max_position_jump_m) or max_position_jump_m <= 0:
            raise ValueError('max_position_jump_m must be finite and positive')
        if not math.isfinite(max_rotation_jump_deg) or max_rotation_jump_deg <= 0:
            raise ValueError('max_rotation_jump_deg must be finite and positive')
        self.max_position_jump_m = max_position_jump_m
        self.max_rotation_jump_deg = max_rotation_jump_deg
        self._previous_position: Optional[Tuple[float, float, float]] = None
        self._previous_orientation: Optional[Tuple[float, float, float, float]] = None

    @property
    def has_baseline(self) -> bool:
        return self._previous_position is not None

    @staticmethod
    def quaternion_is_normalizable(pose: Pose) -> bool:
        quaternion = pose.orientation
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not all(math.isfinite(float(value)) for value in values):
            return False
        norm = math.sqrt(sum(float(value) ** 2 for value in values))
        return norm > 1.0e-12

    @staticmethod
    def _values(
        pose: Pose,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        position = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        quaternion = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 1.0e-12 or not math.isfinite(norm):
            raise ValueError('Pose quaternion is not normalizable')
        normalized = tuple(value / norm for value in quaternion)
        return position, normalized

    def reset(self) -> None:
        """Discard the ACTIVE-session baseline."""
        self._previous_position = None
        self._previous_orientation = None

    def initialize(self, pose: Pose) -> None:
        """Use the current valid START pose as a new baseline."""
        position, orientation = self._values(pose)
        self._previous_position = position
        self._previous_orientation = orientation

    def check(self, pose: Pose) -> PoseJumpResult:
        """Compare one new pose with the baseline and update it if accepted."""
        current_position, current_orientation = self._values(pose)
        if self._previous_position is None or self._previous_orientation is None:
            self._previous_position = current_position
            self._previous_orientation = current_orientation
            return PoseJumpResult(False, 0.0, 0.0)

        position_jump_m = math.sqrt(
            sum(
                (current - previous) ** 2
                for current, previous in zip(
                    current_position,
                    self._previous_position,
                )
            )
        )
        dot = abs(
            sum(
                current * previous
                for current, previous in zip(
                    current_orientation,
                    self._previous_orientation,
                )
            )
        )
        dot = min(1.0, max(0.0, dot))
        rotation_jump_deg = math.degrees(2.0 * math.acos(dot))
        triggered = (
            position_jump_m > self.max_position_jump_m
            or rotation_jump_deg > self.max_rotation_jump_deg
        )
        if not triggered:
            self._previous_position = current_position
            self._previous_orientation = current_orientation
        return PoseJumpResult(
            triggered,
            position_jump_m,
            rotation_jump_deg,
        )
