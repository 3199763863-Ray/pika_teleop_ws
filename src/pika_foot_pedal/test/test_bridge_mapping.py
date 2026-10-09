"""Bridge preserves normal USER_STOP and maps pedal faults to STALE_STOP."""

from types import SimpleNamespace
import unittest

from pika_teleop_bridge.node import PikaTeleopPublisher


def case_reason(mode, reason):
    bridge = PikaTeleopPublisher.__new__(PikaTeleopPublisher)
    bridge.mode = {'left': mode}
    bridge.pending_start_futures = {'left': None}
    reasons = []
    bridge._deactivate = lambda side, stop_reason: reasons.append((side, stop_reason))
    response = bridge._manual_enable(
        'left', SimpleNamespace(enable=False, reason=reason),
        SimpleNamespace(success=False, message='')
    )
    assert response.success
    return reasons[0][1]


class TestBridgeReason(unittest.TestCase):
    def test_active_release(self):
        assert case_reason(PikaTeleopPublisher.ACTIVE, 'pedal') == 'USER_STOP'

    def test_pending_release(self):
        assert case_reason(PikaTeleopPublisher.PENDING_START, 'pedal') == 'START_CANCEL_STOP'

    def test_device_lost(self):
        assert case_reason(PikaTeleopPublisher.ACTIVE, 'pedal_device_lost') == 'STALE_STOP'
