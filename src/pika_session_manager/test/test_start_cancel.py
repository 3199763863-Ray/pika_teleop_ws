"""Delayed Recorder START must not outlive a released pedal."""

import asyncio
from concurrent.futures import Future
import threading
import time
import unittest
from types import SimpleNamespace

from pika_session_manager.node import PikaSessionManager
from realman_recording_msgs.srv import ManageRecording


class Recorder:
    def __init__(self):
        self.calls = []
        self.start = asyncio.Future()
        self.stop = asyncio.Future()

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append(request.command)
        return self.start if request.command == ManageRecording.Request.START else self.stop


def test_stop_during_starting_compensates_without_movej():
    async def scenario():
        manager = PikaSessionManager.__new__(PikaSessionManager)
        manager._lock = threading.RLock()
        manager.state = manager.READY
        manager.active = {'left': False, 'right': False}
        manager.session_id = ''
        manager._start_cancel_reason = ''
        manager._abnormal_stop = False
        manager._skip_reset_after_stop = False
        manager.reset_on_user_stop = True
        manager.reset_dispatch_delay_ns = 0
        manager.stop_state_timeout_ns = 6_000_000_000
        manager.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100)
        )
        manager.recording_client = Recorder()
        manager.force_stop_publisher = SimpleNamespace(publish=lambda _: None)
        manager._set_state = lambda state: setattr(manager, 'state', state)
        manager.get_logger = lambda: SimpleNamespace(
            info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
        )
        manager._start_request = lambda: PikaSessionManager._recording_request(
            ManageRecording.Request.START
        )
        manager._begin_reset = lambda: (_ for _ in ()).throw(
            AssertionError('unexpected MoveJ reset')
        )
        response = SimpleNamespace(success=False, message='')
        task = asyncio.create_task(manager._handle_start('left', 'USER_START', response))
        await asyncio.sleep(0)
        assert manager.state == manager.STARTING
        stop = manager._handle_stop(
            'left', 'START_CANCEL_STOP', SimpleNamespace(success=False, message='')
        )
        assert stop.success
        manager.recording_client.start.set_result(SimpleNamespace(
            success=True, state=manager.RECORDING, session_id='test'
        ))
        result = await task
        assert not result.success
        assert manager.state == manager.STOPPING
        assert manager.recording_client.calls == [
            ManageRecording.Request.START, ManageRecording.Request.STOP
        ]
        manager.recording_client.stop.set_result(SimpleNamespace(success=True))
        now_ns = time.monotonic_ns()
        manager._poll_recorder_stop(now_ns)
        manager._poll_stop(now_ns)
        assert manager.state == manager.PREPARING
    asyncio.run(scenario())


class TestStartCancel(unittest.TestCase):
    def test_delayed_recorder_start(self):
        test_stop_during_starting_compensates_without_movej()

    def test_pending_right_preserves_left_normal_reset(self):
        manager = PikaSessionManager.__new__(PikaSessionManager)
        manager._lock = threading.RLock()
        manager.state = manager.RECORDING
        manager.active = {'left': True, 'right': True}
        manager._start_cancel_reason = ''
        manager._abnormal_stop = False
        manager._skip_reset_after_stop = False
        manager.reset_on_user_stop = True
        manager.reset_dispatch_delay_ns = 0
        manager.stop_state_timeout_ns = 6_000_000_000
        manager.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100)
        )
        stop_future = Future()
        manager.recording_client = SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: stop_future,
        )
        manager.force_stop_publisher = SimpleNamespace(publish=lambda _: None)
        manager._set_state = lambda state: setattr(manager, 'state', state)
        manager.get_logger = lambda: SimpleNamespace(
            info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
        )
        reset = []
        manager._begin_reset = lambda: reset.append(True)
        response = manager._handle_stop(
            'right', 'START_CANCEL_STOP', SimpleNamespace(success=False, message='')
        )
        assert response.success
        stop_future.set_result(SimpleNamespace(success=True))
        now_ns = time.monotonic_ns()
        manager._both_disabled_ns = now_ns
        manager._poll_recorder_stop(now_ns)
        manager._poll_stop(now_ns)
        assert reset == [True]
