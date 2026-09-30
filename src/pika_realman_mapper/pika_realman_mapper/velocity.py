"""Estimate target velocity expressed in the configured command frame."""

from collections import deque
import math

from geometry_msgs.msg import Pose, Twist

from .pose_mapper import pose_values
from .quaternion_utils import inverse, multiply, shortest


class TargetVelocityEstimator:
    """Differentiate and filter target poses in their command frame."""

    def __init__(
        self,
        cutoff_hz: float,
        min_dt_ms: float,
        max_dt_ms: float,
        derivative_window_samples: int,
        kalman_enabled: bool,
        linear_kalman_process_variance: float,
        linear_kalman_measurement_variance: float,
        angular_kalman_process_variance: float,
        angular_kalman_measurement_variance: float,
        linear_deadband_mps: float,
        angular_deadband_radps: float,
    ) -> None:
        self.cutoff_hz = float(cutoff_hz)
        min_dt_ms = float(min_dt_ms)
        max_dt_ms = float(max_dt_ms)
        self.derivative_window_samples = int(derivative_window_samples)
        self.kalman_enabled = bool(kalman_enabled)
        self.linear_kalman_process_variance = float(
            linear_kalman_process_variance
        )
        self.linear_kalman_measurement_variance = float(
            linear_kalman_measurement_variance
        )
        self.angular_kalman_process_variance = float(
            angular_kalman_process_variance
        )
        self.angular_kalman_measurement_variance = float(
            angular_kalman_measurement_variance
        )
        self.linear_deadband_mps = float(linear_deadband_mps)
        self.angular_deadband_radps = float(angular_deadband_radps)
        if not math.isfinite(self.cutoff_hz) or self.cutoff_hz <= 0.0:
            raise ValueError('cutoff_hz must be finite and positive')
        if not math.isfinite(min_dt_ms) or min_dt_ms <= 0.0:
            raise ValueError('min_dt_ms must be finite and positive')
        if not math.isfinite(max_dt_ms) or max_dt_ms <= 0.0:
            raise ValueError('max_dt_ms must be finite and positive')
        if min_dt_ms >= max_dt_ms:
            raise ValueError('min_dt_ms must be less than max_dt_ms')
        if self.derivative_window_samples < 2:
            raise ValueError('derivative_window_samples must be at least 2')
        for name, value in (
            (
                'linear_kalman_process_variance',
                self.linear_kalman_process_variance,
            ),
            (
                'linear_kalman_measurement_variance',
                self.linear_kalman_measurement_variance,
            ),
            (
                'angular_kalman_process_variance',
                self.angular_kalman_process_variance,
            ),
            (
                'angular_kalman_measurement_variance',
                self.angular_kalman_measurement_variance,
            ),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if (
            not math.isfinite(self.linear_deadband_mps)
            or self.linear_deadband_mps < 0.0
        ):
            raise ValueError(
                'linear_deadband_mps must be finite and non-negative'
            )
        if (
            not math.isfinite(self.angular_deadband_radps)
            or self.angular_deadband_radps < 0.0
        ):
            raise ValueError(
                'angular_deadband_radps must be finite and non-negative'
            )
        self.min_dt_ns = int(min_dt_ms * 1.0e6)
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
        self._samples = deque(maxlen=self.derivative_window_samples)
        self._filtered = [0.0] * 6
        self._kalman_state = [0.0] * 6
        self._kalman_covariance = [0.0] * 6
        self._twist = Twist()
        self._valid = False

    def _rebaseline(self, pose_values_now, stamp_ns: int) -> bool:
        self._samples.clear()
        self._samples.append((pose_values_now, stamp_ns))
        self._filtered = [0.0] * 6
        self._kalman_state = [0.0] * 6
        self._kalman_covariance = [0.0] * 6
        self._twist = Twist()
        self._valid = False
        return False

    def _kalman_update(self, raw, dt: float):
        if not self.kalman_enabled:
            return raw
        result = []
        for index, measurement in enumerate(raw):
            if index < 3:
                process_variance = self.linear_kalman_process_variance
                measurement_variance = (
                    self.linear_kalman_measurement_variance
                )
            else:
                process_variance = self.angular_kalman_process_variance
                measurement_variance = (
                    self.angular_kalman_measurement_variance
                )
            predicted_covariance = (
                self._kalman_covariance[index]
                + process_variance * dt
            )
            gain = predicted_covariance / (
                predicted_covariance + measurement_variance
            )
            estimate = self._kalman_state[index] + gain * (
                measurement - self._kalman_state[index]
            )
            self._kalman_state[index] = estimate
            self._kalman_covariance[index] = (
                1.0 - gain
            ) * predicted_covariance
            result.append(estimate)
        return tuple(result)

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
        if not self._samples:
            return self._rebaseline(current, stamp_ns)

        step_dt_ns = stamp_ns - self._samples[-1][1]
        if step_dt_ns <= 0 or step_dt_ns > self.max_dt_ns:
            return self._rebaseline(current, stamp_ns)
        if step_dt_ns < self.min_dt_ns:
            return False
        self._samples.append((current, stamp_ns))
        if len(self._samples) < self.derivative_window_samples:
            self._twist = Twist()
            self._valid = False
            return False

        previous, previous_stamp_ns = self._samples[0]
        dt_ns = stamp_ns - previous_stamp_ns
        derivative_dt = dt_ns * 1.0e-9
        sample_dt = step_dt_ns * 1.0e-9
        current_position, current_orientation = current
        previous_position, previous_orientation = previous

        linear = tuple(
            (now - before) / derivative_dt
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
                component / vector_norm * angle / derivative_dt
                for component in vector
            )
        raw = (*linear, *angular)
        if not all(math.isfinite(value) for value in raw):
            self.reset()
            return False

        filtered_input = self._kalman_update(raw, sample_dt)

        alpha = sample_dt / (self._tau + sample_dt)
        self._filtered = [
            previous + alpha * (current_value - previous)
            for previous, current_value in zip(
                self._filtered, filtered_input
            )
        ]
        linear_norm = math.sqrt(sum(value * value for value in self._filtered[:3]))
        angular_norm = math.sqrt(sum(value * value for value in self._filtered[3:]))
        if linear_norm < self.linear_deadband_mps:
            self._filtered[:3] = [0.0, 0.0, 0.0]
            self._kalman_state[:3] = [0.0, 0.0, 0.0]
            self._kalman_covariance[:3] = [0.0, 0.0, 0.0]
        if angular_norm < self.angular_deadband_radps:
            self._filtered[3:] = [0.0, 0.0, 0.0]
            self._kalman_state[3:] = [0.0, 0.0, 0.0]
            self._kalman_covariance[3:] = [0.0, 0.0, 0.0]
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self._filtered[:3]
        twist.angular.x, twist.angular.y, twist.angular.z = self._filtered[3:]
        self._twist = twist
        self._valid = True
        return True
