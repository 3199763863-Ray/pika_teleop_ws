"""Safety checks for the Bag-only per-side reset handoff."""

import unittest

from pika_teleop_virtual_receiver.reset_handoff import ResetHandoff


MS = 1_000_000


def handoff():
    return ResetHandoff(1000 * MS, 150 * MS)


def activate(flow, baseline=2):
    assert flow.start(baseline)
    flow.state(False, False, baseline + 1, 10 * MS)
    flow.state(True, True, baseline + 2, 20 * MS)
    assert flow.active


def case_only_active_user_stop_dispatches_once_after_fresh_disabled_state():
    flow = handoff()
    assert not flow.stop('USER_STOP', 2, 0)
    activate(flow)
    assert flow.stop('USER_STOP', 4, 30 * MS)
    assert not flow.stop('USER_STOP', 4, 31 * MS)
    assert not flow.start(4)
    assert flow.state(False, False, 4, 40 * MS) == ''
    assert flow.tick(100 * MS) == ''
    assert flow.state(False, False, 5, 50 * MS) == 'confirmed'
    assert flow.tick(199 * MS) == ''
    assert flow.tick(200 * MS) == 'dispatch'
    assert flow.tick(300 * MS) == ''
    assert flow.start(5)


def case_pending_start_and_abnormal_stop_never_dispatch():
    for reason in ('STALE_STOP', 'POSE_JUMP_STOP', 'WATCHDOG_TIMEOUT'):
        flow = handoff()
        assert flow.start(2)
        assert not flow.stop('USER_STOP', 2, 30 * MS)
        activate(flow)
        assert not flow.stop(reason, 4, 30 * MS)
        flow.state(False, False, 5, 40 * MS)
        assert flow.tick(500 * MS) == ''


def case_timeout_and_contradictory_state_cancel_without_late_dispatch():
    flow = handoff()
    activate(flow)
    assert flow.stop('USER_STOP', 4, 30 * MS)
    assert flow.tick(1030 * MS) == 'confirmation timeout'
    assert flow.state(False, False, 5, 1040 * MS) == ''
    assert flow.tick(1200 * MS) == ''

    assert flow.start(5)
    flow.state(True, True, 6, 1300 * MS)
    assert flow.stop('USER_STOP', 6, 1310 * MS)
    assert flow.state(True, True, 7, 1320 * MS) == 'contradictory State after STOP'
    assert flow.tick(2000 * MS) == ''

    assert flow.start(7)
    flow.state(True, True, 8, 2100 * MS)
    assert flow.stop('USER_STOP', 8, 2110 * MS)
    assert flow.state(False, False, 9, 3110 * MS) == 'confirmation timeout'
    assert flow.tick(3300 * MS) == ''


def case_watchdog_and_abnormal_stop_cancel_pending():
    flow = handoff()
    activate(flow)
    assert flow.stop('USER_STOP', 4, 30 * MS)
    assert flow.state(False, False, 5, 40 * MS) == 'confirmed'
    assert flow.invalidate()
    assert flow.tick(200 * MS) == ''

    activate(flow, 5)
    assert flow.stop('USER_STOP', 7, 300 * MS)
    assert not flow.stop('STALE_STOP', 7, 310 * MS)
    assert flow.tick(500 * MS) == ''


def case_two_sides_are_independent():
    left, right = handoff(), handoff()
    activate(left)
    activate(right)
    assert left.stop('USER_STOP', 4, 30 * MS)
    assert right.active
    assert left.state(False, False, 5, 50 * MS) == 'confirmed'
    assert left.tick(200 * MS) == 'dispatch'
    assert right.tick(200 * MS) == ''


class TestResetHandoff(unittest.TestCase):
    def test_active_user_stop(self):
        case_only_active_user_stop_dispatches_once_after_fresh_disabled_state()

    def test_pending_start_and_abnormal_stop(self):
        case_pending_start_and_abnormal_stop_never_dispatch()

    def test_timeout_and_contradictory_state(self):
        case_timeout_and_contradictory_state_cancel_without_late_dispatch()

    def test_watchdog_and_abnormal_stop_cancel(self):
        case_watchdog_and_abnormal_stop_cancel_pending()

    def test_two_sides_independent(self):
        case_two_sides_are_independent()
