"""Formal STOP handoff tests; fake Recorder, State and Actions only."""

import asyncio
from concurrent.futures import Future
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from pika_session_manager.node import PikaSessionManager
from pika_teleop_interfaces.msg import PikaTeleopState
from realman_msgs.action import ExecuteMotion


T0 = 1_000_000_000_000
SECOND = 1_000_000_000


def completed(value=None, error=None):
    future = Future()
    if error is None:
        future.set_result(value)
    else:
        future.set_exception(error)
    return future


class Recorder:
    def __init__(self, ready=True, call_error=None):
        self.ready = ready
        self.call_error = call_error
        self.future = Future()
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.call_error:
            raise self.call_error
        self.calls.append(request.command)
        return self.future


class Action:
    def __init__(self, ready=True, accepted=True, success=True, send_error=None,
                 pending_result=False):
        self.ready = ready
        self.accepted = accepted
        self.success = success
        self.send_error = send_error
        self.pending_result = pending_result
        self.goals = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        if self.send_error:
            raise self.send_error
        self.goals.append(goal)
        result = SimpleNamespace(
            success=self.success,
            terminal_state=ExecuteMotion.Result.SUCCEEDED if self.success else 0,
            api2_status=0,
            final_joint_degrees=[],
            message='fake result',
        )
        handle = SimpleNamespace(
            accepted=self.accepted,
            get_result_async=lambda: (
                Future() if self.pending_result
                else completed(SimpleNamespace(result=result))
            ),
        )
        return completed(handle)


def make_manager(*, recorder=None, reset_enabled=True, reason='USER_STOP', actions=None):
    manager = PikaSessionManager.__new__(PikaSessionManager)
    manager._lock = threading.RLock()
    manager.state = manager.RECORDING
    manager.active = {'left': True, 'right': True}
    manager.session_id = 'fake'
    manager._start_cancel_reason = ''
    manager._abnormal_stop = False
    manager._skip_reset_after_stop = False
    manager.reset_on_user_stop = reset_enabled
    manager.reset_dispatch_delay_ns = 4 * SECOND
    manager.reset_timeout_sec = 120.0
    manager.stop_state_timeout_ns = 6 * SECOND
    manager.recording_client = recorder or Recorder()
    manager.action_clients = actions or {'left': Action(), 'right': Action()}
    manager._reset_goal = lambda side: side
    manager._reset_goal_futures = {}
    manager._reset_result_futures = {}
    manager._reset_failures = {}
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=SECOND)
    )
    manager.messages = []
    manager.get_logger = lambda: SimpleNamespace(
        info=lambda msg: manager.messages.append(('info', msg)),
        warning=lambda msg: manager.messages.append(('warning', msg)),
        error=lambda msg: manager.messages.append(('error', msg)),
    )
    manager._set_state = lambda state: setattr(manager, 'state', state)
    manager.force_stops = []
    manager.force_stop_publisher = SimpleNamespace(
        publish=lambda msg: manager.force_stops.append(msg)
    )
    response = manager._handle_stop(
        'left', reason, SimpleNamespace(success=False, message='')
    )
    assert response.success and manager.state == manager.STOPPING
    assert not any(manager.active.values()) and len(manager.force_stops) == 1
    manager._stop_started_ns = T0
    return manager


def state_message(*, stamp=2 * SECOND, enabled=False, valid=False):
    msg = PikaTeleopState()
    msg.header.stamp.sec = stamp // SECOND
    msg.header.stamp.nanosec = stamp % SECOND
    msg.enabled = enabled
    msg.valid = valid
    return msg


def confirm_disabled(manager, when=T0 + SECOND):
    with patch('pika_session_manager.node.time.monotonic_ns', return_value=when):
        for side in manager.SIDES:
            manager._teleop_state_callback(side, state_message())
    assert manager._both_disabled_ns == when


def tick(manager, when):
    manager._poll_recorder_stop(when)
    if manager.state == manager.STOPPING:
        manager._poll_stop(when)
    elif manager.state == manager.RESETTING:
        manager._poll_reset(when)


def goals(manager):
    return {side: len(client.goals) for side, client in manager.action_clients.items()}


class TestFormalReset(unittest.TestCase):
    def test_delay_parameter_accepts_zero_and_rejects_invalid(self):
        for value in (0.0, 4000.0):
            node = SimpleNamespace(
                get_parameter=lambda name: SimpleNamespace(value=value)
            )
            self.assertEqual(
                PikaSessionManager._nonnegative_parameter(node, 'reset_dispatch_delay_ms'),
                value,
            )
        for value in (-1.0, float('nan'), float('inf')):
            node = SimpleNamespace(
                get_parameter=lambda name: SimpleNamespace(value=value)
            )
            with self.assertRaises(ValueError):
                PikaSessionManager._nonnegative_parameter(node, 'reset_dispatch_delay_ms')

    def test_success_waits_for_later_of_recorder_and_disabled(self):
        manager = make_manager()
        manager.recording_client.future.set_result(SimpleNamespace(success=True))
        tick(manager, T0 + SECOND // 2)
        confirm_disabled(manager)
        tick(manager, T0 + 5 * SECOND - 1)
        self.assertEqual(goals(manager), {'left': 0, 'right': 0})
        tick(manager, T0 + 5 * SECOND)
        self.assertEqual(goals(manager), {'left': 1, 'right': 1})
        tick(manager, T0 + 5 * SECOND + 1)
        self.assertEqual(manager.state, manager.PREPARING)

        later = make_manager()
        confirm_disabled(later)
        later.recording_client.future.set_result(SimpleNamespace(success=True))
        tick(later, T0 + 2 * SECOND)
        tick(later, T0 + 6 * SECOND - 1)
        self.assertEqual(goals(later), {'left': 0, 'right': 0})
        tick(later, T0 + 6 * SECOND)
        self.assertEqual(goals(later), {'left': 1, 'right': 1})

    def test_recorder_failure_or_no_reply_still_resets_then_fails(self):
        recorders = [
            Recorder(ready=False),
            Recorder(call_error=RuntimeError('call error')),
            Recorder(),
            Recorder(),
            Recorder(),
        ]
        recorders[2].future.set_result(SimpleNamespace(success=False, message='rejected'))
        recorders[3].future.set_exception(RuntimeError('reply exception'))
        for recorder in recorders:
            with self.subTest(recorder=recorder):
                manager = make_manager(recorder=recorder)
                confirm_disabled(manager)
                tick(manager, T0 + 5 * SECOND - 1)
                self.assertEqual(goals(manager), {'left': 0, 'right': 0})
                tick(manager, T0 + 5 * SECOND)
                self.assertEqual(goals(manager), {'left': 1, 'right': 1})
                tick(manager, T0 + 5 * SECOND + 1)
                self.assertEqual(manager.state, manager.FAILED)
                self.assertTrue(any('RECORDER_STOP_' in text for _, text in manager.messages))

    def test_late_reply_and_duplicate_stop_never_reset_twice(self):
        for success in (True, False):
            with self.subTest(late_success=success):
                manager = make_manager()
                confirm_disabled(manager)
                duplicate = manager._handle_stop(
                    'left', 'USER_STOP', SimpleNamespace(success=False, message='')
                )
                self.assertTrue(duplicate.success)
                tick(manager, T0 + 5 * SECOND)
                manager.recording_client.future.set_result(SimpleNamespace(
                    success=success, message='late reply'
                ))
                tick(manager, T0 + 5 * SECOND + 1)
                self.assertEqual(manager.state, manager.FAILED)
                self.assertEqual(manager._recorder_stop_status, 'UNCONFIRMED')
                self.assertEqual(goals(manager), {'left': 1, 'right': 1})
                tick(manager, T0 + 6 * SECOND)
                self.assertEqual(goals(manager), {'left': 1, 'right': 1})

    def test_old_or_enabled_state_cannot_confirm_handoff(self):
        manager = make_manager()
        manager.recording_client.future.set_result(SimpleNamespace(success=True))
        tick(manager, T0 + SECOND)
        for side in manager.SIDES:
            manager._teleop_state_callback(side, state_message(stamp=SECOND))
            manager._teleop_state_callback(side, state_message(enabled=True, valid=True))
        tick(manager, T0 + 5 * SECOND)
        self.assertEqual(goals(manager), {'left': 0, 'right': 0})
        tick(manager, T0 + 6 * SECOND)
        self.assertEqual(manager.state, manager.FAILED)
        self.assertEqual(goals(manager), {'left': 0, 'right': 0})

    def test_reset_disabled_and_abnormal_stop_never_move(self):
        success = make_manager(reset_enabled=False)
        success.recording_client.future.set_result(SimpleNamespace(success=True))
        tick(success, T0 + SECOND)
        self.assertEqual(success.state, success.PREPARING)
        self.assertEqual(goals(success), {'left': 0, 'right': 0})

        missing = make_manager(recorder=Recorder(ready=False), reset_enabled=False)
        tick(missing, T0 + SECOND)
        self.assertEqual(missing.state, missing.FAILED)

        pending = make_manager(reset_enabled=False)
        tick(pending, T0 + 4 * SECOND)
        self.assertEqual(pending.state, pending.FAILED)

        for reason in ('STALE_STOP', 'POSE_JUMP_STOP'):
            abnormal = make_manager(reason=reason)
            abnormal.recording_client.future.set_result(SimpleNamespace(success=True))
            tick(abnormal, T0 + SECOND)
            self.assertEqual(abnormal.state, abnormal.FAILED)
            self.assertEqual(goals(abnormal), {'left': 0, 'right': 0})

    def test_action_failures_are_not_retried(self):
        cases = [
            {'left': Action(ready=False), 'right': Action()},
            {'left': Action(accepted=False), 'right': Action()},
            {'left': Action(success=False), 'right': Action()},
            {'left': Action(send_error=RuntimeError('send error')), 'right': Action()},
        ]
        for actions in cases:
            with self.subTest(actions=actions):
                manager = make_manager(actions=actions)
                manager.recording_client.future.set_result(SimpleNamespace(success=True))
                tick(manager, T0 + SECOND // 2)
                confirm_disabled(manager)
                tick(manager, T0 + 5 * SECOND)
                tick(manager, T0 + 5 * SECOND + 1)
                self.assertEqual(manager.state, manager.FAILED)
                self.assertTrue(all(len(action.goals) <= 1 for action in actions.values()))

    def test_missing_action_result_times_out(self):
        manager = make_manager(actions={
            'left': Action(pending_result=True), 'right': Action()
        })
        manager.recording_client.future.set_result(SimpleNamespace(success=True))
        tick(manager, T0 + SECOND // 2)
        confirm_disabled(manager)
        tick(manager, T0 + 5 * SECOND)
        manager._reset_deadline_ns = T0 + 7 * SECOND
        tick(manager, T0 + 7 * SECOND)
        self.assertEqual(manager.state, manager.FAILED)
        self.assertEqual(goals(manager), {'left': 1, 'right': 1})

    def test_pending_right_cancel_preserves_left_normal_stop(self):
        manager = PikaSessionManager.__new__(PikaSessionManager)
        manager._lock = threading.RLock()
        manager.state = manager.RECORDING
        manager.active = {'left': True, 'right': False}
        manager._start_cancel_reason = ''
        manager._abnormal_stop = False
        manager._skip_reset_after_stop = False
        manager.reset_on_user_stop = True
        manager.reset_dispatch_delay_ns = 4 * SECOND
        manager.stop_state_timeout_ns = 6 * SECOND
        manager.reset_timeout_sec = 120.0
        manager.recording_client = Recorder()
        manager.action_clients = {'left': Action(), 'right': Action()}
        manager._reset_goal = lambda side: side
        manager.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=SECOND)
        )
        manager._set_state = lambda state: setattr(manager, 'state', state)
        manager.get_logger = lambda: SimpleNamespace(
            info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
        )
        manager.force_stop_publisher = SimpleNamespace(publish=lambda _: None)
        canceled = manager._handle_stop(
            'right', 'START_CANCEL_STOP', SimpleNamespace(success=False, message='')
        )
        self.assertTrue(canceled.success)
        self.assertEqual(manager.state, manager.RECORDING)
        stopped = manager._handle_stop(
            'left', 'USER_STOP', SimpleNamespace(success=False, message='')
        )
        self.assertTrue(stopped.success)
        self.assertEqual(manager.state, manager.STOPPING)
        self.assertFalse(manager._skip_reset_after_stop)

    def test_start_rejected_during_stop(self):
        manager = make_manager()
        result = asyncio.run(manager._handle_start(
            'left', 'USER_START', SimpleNamespace(success=False, message='')
        ))
        self.assertFalse(result.success)
        self.assertIn('STOPPING', result.message)


if __name__ == '__main__':
    unittest.main()
