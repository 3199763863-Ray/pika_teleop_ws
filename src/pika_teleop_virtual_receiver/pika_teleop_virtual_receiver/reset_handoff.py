"""Per-side, fail-closed STOP-to-MoveJ handoff without ROS dependencies."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class PendingReset:
    requested_ns: int
    request_rx_count: int
    confirmed_ns: Optional[int] = None


class ResetHandoff:
    """Require an active session and a fresh disabled State for one reset."""

    def __init__(self, confirm_timeout_ns: int, dispatch_delay_ns: int):
        self.confirm_timeout_ns = confirm_timeout_ns
        self.dispatch_delay_ns = dispatch_delay_ns
        self.start_rx_count: Optional[int] = None
        self.active = False
        self.pending: Optional[PendingReset] = None

    def start(self, rx_count: int) -> bool:
        if self.pending is not None:
            return False
        self.start_rx_count = rx_count
        self.active = False
        return True

    def stop(self, reason: str, rx_count: int, now_ns: int) -> bool:
        """Return true only when this STOP creates a new reset handoff."""
        if self.pending is not None:
            if reason != 'USER_STOP':
                self.invalidate()
            return False
        should_reset = reason == 'USER_STOP' and self.active
        self.active = False
        self.start_rx_count = None
        if should_reset:
            self.pending = PendingReset(now_ns, rx_count)
        return should_reset

    def state(self, enabled: bool, valid: bool, rx_count: int, now_ns: int) -> str:
        pending = self.pending
        if pending is not None:
            if rx_count <= pending.request_rx_count:
                return ''
            if (
                pending.confirmed_ns is None
                and now_ns - pending.requested_ns >= self.confirm_timeout_ns
            ):
                self.pending = None
                return 'confirmation timeout'
            if enabled or valid:
                self.pending = None
                return 'contradictory State after STOP'
            if pending.confirmed_ns is None:
                pending.confirmed_ns = now_ns
                return 'confirmed'
            return ''

        if (
            self.start_rx_count is not None
            and rx_count > self.start_rx_count
            and enabled and valid
        ):
            self.active = True
        elif not enabled or not valid:
            if self.active:
                self.start_rx_count = None
            self.active = False
        return ''

    def invalidate(self) -> bool:
        """Disarm on watchdog failure; cancel an unsubmitted reset."""
        had_pending = self.pending is not None
        self.pending = None
        self.active = False
        self.start_rx_count = None
        return had_pending

    def tick(self, now_ns: int) -> str:
        pending = self.pending
        if pending is None:
            return ''
        if pending.confirmed_ns is None:
            if now_ns - pending.requested_ns >= self.confirm_timeout_ns:
                self.pending = None
                return 'confirmation timeout'
            return ''
        if now_ns - pending.confirmed_ns >= self.dispatch_delay_ns:
            self.pending = None
            return 'dispatch'
        return ''
