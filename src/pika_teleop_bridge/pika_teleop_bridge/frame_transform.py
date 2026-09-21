"""Fixed right-handed transform from the Pika frame to the teleop frame."""

import math
from typing import Tuple

from geometry_msgs.msg import Pose, Vector3


QuaternionTuple = Tuple[float, float, float, float]


class PikaFrameTransform:
    """Apply x_new=-z_old, y_new=y_old, z_new=x_old."""

    FRAME_ID = 'pika_teleop_frame'
    _SQRT_HALF = math.sqrt(0.5)
    _FRAME_QUATERNION = (0.0, -_SQRT_HALF, 0.0, _SQRT_HALF)

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

    @staticmethod
    def _normalized(quaternion: QuaternionTuple) -> QuaternionTuple:
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isfinite(norm) or norm <= 1.0e-12:
            raise ValueError('Quaternion is not normalizable')
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def transform_vector_xyz(x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Transform vector components without translation."""
        return (-float(z), float(y), float(x))

    @classmethod
    def transform_vector(cls, source: Vector3) -> Vector3:
        target = Vector3()
        target.x, target.y, target.z = cls.transform_vector_xyz(
            source.x, source.y, source.z
        )
        return target

    @classmethod
    def transform_pose(cls, source: Pose) -> Pose:
        """Transform position and orientation into the fixed teleop basis."""
        target = Pose()
        target.position.x, target.position.y, target.position.z = (
            cls.transform_vector_xyz(
                source.position.x,
                source.position.y,
                source.position.z,
            )
        )
        source_quaternion = cls._normalized(
            (
                float(source.orientation.x),
                float(source.orientation.y),
                float(source.orientation.z),
                float(source.orientation.w),
            )
        )
        frame = cls._FRAME_QUATERNION
        transformed = cls._multiply(
            cls._multiply(frame, source_quaternion),
            cls._conjugate(frame),
        )
        transformed = cls._normalized(transformed)
        target.orientation.x = transformed[0]
        target.orientation.y = transformed[1]
        target.orientation.z = transformed[2]
        target.orientation.w = transformed[3]
        return target
