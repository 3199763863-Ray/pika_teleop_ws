"""Hysteretic gripper click and multi-click gesture detection."""

import math
from typing import Optional


class GestureDetector:
    """Detect complete open-close-open clicks from one gripper stream."""

    OPEN = 'OPEN'
    CLOSED = 'CLOSED'

    def __init__(
        self,
        open_threshold: float,
        close_threshold: float,
        click_max_interval_ms: float,
        gesture_reset_timeout_ms: float,
    ) -> None:
        if not math.isfinite(open_threshold) or not math.isfinite(close_threshold):
            raise ValueError('Gripper thresholds must be finite')
        if open_threshold <= close_threshold:
            raise ValueError('open_threshold must be greater than close_threshold')
        if click_max_interval_ms <= 0 or gesture_reset_timeout_ms <= 0:
            raise ValueError('Gesture timeouts must be positive')
        self.open_threshold = open_threshold
        self.close_threshold = close_threshold
        self.click_max_interval_ns = int(click_max_interval_ms * 1.0e6)
        self.gesture_reset_timeout_ns = int(gesture_reset_timeout_ms * 1.0e6)
        self._region: Optional[str] = None
        self._pressed_time_ns: Optional[int] = None
        self._last_click_time_ns: Optional[int] = None
        self._last_transition_time_ns: Optional[int] = None
        self._click_count = 0

    @property
    def click_count(self) -> int:
        return self._click_count

    def reset(self, *, preserve_region: bool = True) -> None:
        """Clear partial gesture state, optionally retaining hysteresis region."""
        if not preserve_region:
            self._region = None
        self._pressed_time_ns = None
        self._last_click_time_ns = None
        self._click_count = 0

    def _expire(self, time_ns: int) -> None:
        if self._last_transition_time_ns is None:
            return
        if time_ns - self._last_transition_time_ns > self.gesture_reset_timeout_ns:
            self.reset()

    def update(self, position: float, time_ns: int, target_clicks: int) -> bool:
        """Consume one new sample and return true when target clicks complete."""
        if target_clicks < 1:
            raise ValueError('target_clicks must be at least one')
        if not math.isfinite(position):
            self.reset(preserve_region=False)
            return False

        self._expire(time_ns)
        if self._region is None:
            if position >= self.open_threshold:
                self._region = self.OPEN
            elif position <= self.close_threshold:
                self._region = self.CLOSED
            self._last_transition_time_ns = time_ns
            return False

        new_region = self._region
        if self._region == self.OPEN and position <= self.close_threshold:
            new_region = self.CLOSED
        elif self._region == self.CLOSED and position >= self.open_threshold:
            new_region = self.OPEN
        if new_region == self._region:
            return False

        previous_region = self._region
        self._region = new_region
        self._last_transition_time_ns = time_ns
        if previous_region == self.OPEN and new_region == self.CLOSED:
            self._pressed_time_ns = time_ns
            return False

        if self._pressed_time_ns is None:
            return False
        press_duration_ns = time_ns - self._pressed_time_ns
        self._pressed_time_ns = None
        if press_duration_ns > self.click_max_interval_ns:
            self._click_count = 0
            self._last_click_time_ns = None
            return False

        if (
            self._last_click_time_ns is None
            or time_ns - self._last_click_time_ns <= self.click_max_interval_ns
        ):
            self._click_count += 1
        else:
            self._click_count = 1
        self._last_click_time_ns = time_ns
        if self._click_count < target_clicks:
            return False

        self.reset()
        return True
