"""Small dependency-free quaternion helpers using ROS xyzw ordering."""

import math
from typing import Iterable, Tuple


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


def normalize(values: Iterable[float]) -> Quaternion:
    quaternion = tuple(float(value) for value in values)
    if len(quaternion) != 4:
        raise ValueError('Quaternion must contain exactly four values')
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError('Quaternion contains non-finite values')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError('Quaternion is not normalizable')
    return tuple(value / norm for value in quaternion)


def conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def inverse(quaternion: Quaternion) -> Quaternion:
    return conjugate(normalize(quaternion))


def multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def shortest(quaternion: Quaternion) -> Quaternion:
    normalized = normalize(quaternion)
    if normalized[3] < 0.0:
        return tuple(-value for value in normalized)
    return normalized


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    q = normalize(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    if not all(math.isfinite(value) for value in (vx, vy, vz)):
        raise ValueError('Vector contains non-finite values')
    result = multiply(multiply(q, (vx, vy, vz, 0.0)), conjugate(q))
    return result[:3]


def relative_fixed(current: Quaternion, start: Quaternion) -> Quaternion:
    """Return current * inverse(start), expressed in the fixed frame."""
    return shortest(multiply(normalize(current), inverse(start)))


def map_fixed_delta(delta: Quaternion, mapping: Quaternion) -> Quaternion:
    """Change a fixed-frame rotation delta into the mapped frame."""
    q_map = normalize(mapping)
    return shortest(multiply(multiply(q_map, delta), inverse(q_map)))


def from_rotation_matrix(matrix: Matrix3) -> Quaternion:
    """Convert a right-handed 3x3 rotation matrix to ROS xyzw order."""
    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError('Rotation matrix must be 3x3')
    if not all(math.isfinite(value) for row in rows for value in row):
        raise ValueError('Rotation matrix contains non-finite values')
    m00, m01, m02 = rows[0]
    m10, m11, m12 = rows[1]
    m20, m21, m22 = rows[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    return shortest(quaternion)
