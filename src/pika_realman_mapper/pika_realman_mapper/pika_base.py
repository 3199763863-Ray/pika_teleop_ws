"""Build a stable shared base frame from the two Pika origins."""

from collections import deque
import math
from typing import Optional

from geometry_msgs.msg import Pose

from .pose_mapper import make_pose, pose_values
from .quaternion_utils import (
    Quaternion,
    Vector3,
    from_rotation_matrix,
    multiply,
    normalize,
    rotate_vector,
)


def _add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(left, right))


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(left, right))


def _scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in vector)


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(a * b for a, b in zip(left, right))


def _cross(left: Vector3, right: Vector3) -> Vector3:
    ax, ay, az = left
    bx, by, bz = right
    return (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Vector3) -> Vector3:
    length = _norm(vector)
    if not math.isfinite(length) or length <= 1.0e-9:
        raise ValueError('Cannot normalize a zero-length axis')
    return _scale(vector, 1.0 / length)


def _mean(vectors) -> Vector3:
    count = len(vectors)
    if count == 0:
        raise ValueError('Cannot average an empty sample set')
    return tuple(
        sum(vector[index] for vector in vectors) / count
        for index in range(3)
    )


class PikaBaseCalibrator:
    """Freeze a shared Pika base after a stable paired-sample window."""

    def __init__(
        self,
        stable_samples: int,
        max_position_spread_m: float,
        min_controller_separation_m: float,
        vertical_axis_in_teleop_frame: Vector3,
    ) -> None:
        self.stable_samples = int(stable_samples)
        self.max_position_spread_m = float(max_position_spread_m)
        self.min_controller_separation_m = float(min_controller_separation_m)
        if self.stable_samples < 2:
            raise ValueError('stable_samples must be at least 2')
        if self.max_position_spread_m <= 0.0:
            raise ValueError('max_position_spread_m must be positive')
        if self.min_controller_separation_m <= 0.0:
            raise ValueError('min_controller_separation_m must be positive')
        self.vertical_axis = _unit(tuple(vertical_axis_in_teleop_frame))
        self._samples = deque(maxlen=self.stable_samples)
        self._last_pair_key = None
        self.origin: Optional[Vector3] = None
        self.x_axis: Optional[Vector3] = None
        self.y_axis: Optional[Vector3] = None
        self.z_axis: Optional[Vector3] = None
        self.base_from_teleop: Optional[Quaternion] = None

    @property
    def calibrated(self) -> bool:
        return self.base_from_teleop is not None

    def add_sample(
        self,
        left_pose: Pose,
        right_pose: Pose,
        pair_key,
    ) -> bool:
        """Add a unique paired sample and return True on first calibration."""
        if self.calibrated or pair_key == self._last_pair_key:
            return False
        self._last_pair_key = pair_key
        left_position, _ = pose_values(left_pose)
        right_position, _ = pose_values(right_pose)
        separation = _subtract(right_position, left_position)
        horizontal = _subtract(
            separation,
            _scale(self.vertical_axis, _dot(separation, self.vertical_axis)),
        )
        if _norm(horizontal) < self.min_controller_separation_m:
            self._samples.clear()
            return False
        self._samples.append((left_position, right_position))
        if len(self._samples) < self.stable_samples:
            return False

        mean_left = _mean([sample[0] for sample in self._samples])
        mean_right = _mean([sample[1] for sample in self._samples])
        max_spread = max(
            _norm(_subtract(position, mean))
            for side_index, mean in ((0, mean_left), (1, mean_right))
            for position in [sample[side_index] for sample in self._samples]
        )
        if max_spread > self.max_position_spread_m:
            newest = self._samples[-1]
            self._samples.clear()
            self._samples.append(newest)
            return False

        separation = _subtract(mean_right, mean_left)
        y_axis = _unit(
            _subtract(
                separation,
                _scale(
                    self.vertical_axis,
                    _dot(separation, self.vertical_axis),
                ),
            )
        )
        x_axis = _unit(_cross(y_axis, self.vertical_axis))
        z_axis = _unit(_cross(x_axis, y_axis))
        # Rows project teleop-frame vectors onto the new X/Y/Z axes.
        self.base_from_teleop = from_rotation_matrix((x_axis, y_axis, z_axis))
        self.origin = _scale(_add(mean_left, mean_right), 0.5)
        self.x_axis = x_axis
        self.y_axis = y_axis
        self.z_axis = z_axis
        self._samples.clear()
        return True

    def transform_pose(self, pose: Pose) -> Pose:
        if not self.calibrated:
            raise RuntimeError('Pika base frame has not been calibrated')
        position, orientation = pose_values(pose)
        centered = _subtract(position, self.origin)
        mapped_position = rotate_vector(self.base_from_teleop, centered)
        mapped_orientation = normalize(
            multiply(self.base_from_teleop, orientation)
        )
        return make_pose((mapped_position, mapped_orientation))
