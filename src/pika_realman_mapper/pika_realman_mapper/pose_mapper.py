"""Session-relative Pika pose and gripper mapping."""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import Pose

from .quaternion_utils import (
    Quaternion,
    inverse,
    map_fixed_delta,
    multiply,
    normalize,
    rotate_vector,
)


PoseValues = Tuple[Tuple[float, float, float], Quaternion]


def pose_values(pose: Pose) -> PoseValues:
    position = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    )
    if not all(math.isfinite(value) for value in position):
        raise ValueError('Pose position contains non-finite values')
    orientation = normalize(
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
    )
    return position, orientation


def make_pose(values: PoseValues) -> Pose:
    position, orientation = values
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = orientation
    return pose


def gripper_percentage(value: float, closed: float, opened: float) -> float:
    value = float(value)
    closed = float(closed)
    opened = float(opened)
    if not all(math.isfinite(item) for item in (value, closed, opened)):
        raise ValueError('Gripper mapping values must be finite')
    if opened <= closed:
        raise ValueError('Gripper open position must exceed closed position')
    return min(1.0, max(0.0, (value - closed) / (opened - closed)))


class PoseMapper:
    """Map Pika poses into the configured robot base frame."""

    VALID_ORIENTATION_MODES = ('relative', 'absolute')

    def __init__(
        self,
        base_from_pika_quaternion: Quaternion,
        translation_scale: float,
        orientation_mode: str = 'relative',
    ) -> None:
        self.mapping = normalize(base_from_pika_quaternion)
        self.translation_scale = float(translation_scale)
        self.orientation_mode = str(orientation_mode).strip().lower()
        if (
            not math.isfinite(self.translation_scale)
            or self.translation_scale <= 0.0
        ):
            raise ValueError('translation_scale must be finite and positive')
        if self.orientation_mode not in self.VALID_ORIENTATION_MODES:
            raise ValueError(
                'orientation_mode must be one of: '
                + ', '.join(self.VALID_ORIENTATION_MODES)
            )
        self._pika_start: Optional[PoseValues] = None
        self._rm_start: Optional[PoseValues] = None

    @property
    def initialized(self) -> bool:
        return self._pika_start is not None and self._rm_start is not None

    @property
    def pika_start(self) -> Optional[PoseValues]:
        return self._pika_start

    @property
    def rm_start(self) -> Optional[PoseValues]:
        return self._rm_start

    def reset(self) -> None:
        self._pika_start = None
        self._rm_start = None

    def initialize(self, pika_start: Pose, rm_start: Pose) -> None:
        pika_values = pose_values(pika_start)
        rm_values = pose_values(rm_start)
        self._pika_start = pika_values
        self._rm_start = rm_values

    def map_pose(self, pika_current: Pose) -> Pose:
        if not self.initialized:
            raise RuntimeError('PoseMapper has no active session reference')
        current_position, current_orientation = pose_values(pika_current)
        pika_start_position, pika_start_orientation = self._pika_start
        rm_start_position, rm_start_orientation = self._rm_start

        pika_delta_fixed = tuple(
            current - start
            for current, start in zip(
                current_position,
                pika_start_position,
            )
        )
        pika_delta_start_frame = rotate_vector(
            inverse(pika_start_orientation),
            pika_delta_fixed,
        )
        base_delta = tuple(
            value * self.translation_scale
            for value in rotate_vector(
                self.mapping,
                pika_delta_start_frame,
            )
        )
        target_position = tuple(
            start + delta
            for start, delta in zip(rm_start_position, base_delta)
        )

        if self.orientation_mode == 'relative':
            pika_rotation_delta = normalize(
                multiply(
                    inverse(pika_start_orientation),
                    current_orientation,
                )
            )
            base_rotation_delta = map_fixed_delta(
                pika_rotation_delta,
                self.mapping,
            )
            target_orientation = normalize(
                multiply(rm_start_orientation, base_rotation_delta)
            )
        else:
            target_orientation = map_fixed_delta(
                current_orientation,
                self.mapping,
            )
        return make_pose((target_position, target_orientation))
