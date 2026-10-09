"""Service ordering tests with fake clients; no ROS graph or robot motion."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest

from pika_foot_pedal.node import FootPedalNode
from pika_foot_pedal.debounce import PedalDebouncer


class Client:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        future = Future()
        self.calls.append((request, future))
        return future


def make_node():
    node = FootPedalNode.__new__(FootPedalNode)
    node.enable_clients = {'left': Client(), 'right': Client()}
    node.debouncer = PedalDebouncer()
    node.debouncer.initialize(False, 0)
    node.phase = 'IDLE'
    node.toggle_enabled = False
    node.retrigger_guard_ns = 150_000_000
    node._last_toggle_press_ns = -node.retrigger_guard_ns
    node.generation = 0
    node._publish = lambda: None
    node.requested = set()
    node.pending = []
    node.stop_retry = {}
    node.cancel_reason = 'pedal'
    node.active_after_ns = {'left': 0, 'right': 0}
    node.start_timeout_ns = 10_000_000_000
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *_: None, warning=lambda *_: None, error=lambda *_: None
    )
    return node


def test_left_state_gates_right_and_second_press_stops_both():
    node = make_node()
    node._handle_press(1_000_000_000)
    assert len(node.enable_clients['left'].calls) == 1
    assert not node.enable_clients['right'].calls
    node.enable_clients['left'].calls[0][1].set_result(SimpleNamespace(success=True, message='accepted'))
    node._poll_futures()
    assert node.phase == 'WAIT_LEFT'
    assert not node.enable_clients['right'].calls
    node._state_callback('left', SimpleNamespace(enabled=True, valid=True))
    assert len(node.enable_clients['right'].calls) == 1
    node.enable_clients['right'].calls[0][1].set_result(SimpleNamespace(success=True, message='accepted'))
    node._poll_futures()
    node._state_callback('right', SimpleNamespace(enabled=True, valid=True))
    assert node.phase == 'RUNNING'
    node.debouncer.pressed = False  # Physical release must not stop teleop.
    assert node.toggle_enabled and node.phase == 'RUNNING'
    node._handle_press(1_200_000_000)
    assert node.phase == 'IDLE'
    assert not node.toggle_enabled
    for side in ('left', 'right'):
        assert [call.enable for call, _ in node.enable_clients[side].calls] == [True, False]


def test_second_press_before_start_ack_sends_compensating_stop():
    node = make_node()
    node._handle_press(1_000_000_000)
    node._handle_press(1_200_000_000)
    assert [call.enable for call, _ in node.enable_clients['left'].calls] == [True, False]
    node.enable_clients['left'].calls[0][1].set_result(SimpleNamespace(success=True, message='late'))
    node._poll_futures()
    assert [call.enable for call, _ in node.enable_clients['left'].calls] == [True, False, False]
    assert not node.enable_clients['right'].calls


def test_missing_right_service_stops_left_and_can_retry():
    node = make_node()
    node.enable_clients['right'].ready = False
    node._handle_press(1_000_000_000)
    node.enable_clients['left'].calls[0][1].set_result(SimpleNamespace(success=True, message='accepted'))
    node._poll_futures()
    node._state_callback('left', SimpleNamespace(enabled=True, valid=True))
    assert node.phase == 'IDLE'
    assert not node.toggle_enabled
    assert [call.enable for call, _ in node.enable_clients['left'].calls] == [True, False]
    assert len(node.enable_clients['right'].calls) == 0


def test_quick_duplicate_press_is_ignored():
    node = make_node()
    node._handle_press(1_000_000_000)
    node._handle_press(1_040_000_000)
    assert node.toggle_enabled
    assert [call.enable for call, _ in node.enable_clients['left'].calls] == [True]


def test_batched_40ms_pulse_still_starts():
    node = make_node()
    node._consume_key_events([
        SimpleNamespace(sec=1, usec=0, value=1),
        SimpleNamespace(sec=1, usec=40_000, value=0),
    ], 1_050_000_000)
    node._on_edge(node.debouncer.tick(1_080_000_000), 1_080_000_000)
    assert node.toggle_enabled
    assert [call.enable for call, _ in node.enable_clients['left'].calls] == [True]


class TestCoordinator(unittest.TestCase):
    def test_sequential_start_and_parallel_stop(self):
        test_left_state_gates_right_and_second_press_stops_both()

    def test_late_start_reply(self):
        test_second_press_before_start_ack_sends_compensating_stop()

    def test_missing_service(self):
        test_missing_right_service_stops_left_and_can_retry()

    def test_guard(self):
        test_quick_duplicate_press_is_ignored()

    def test_batched_pulse(self):
        test_batched_40ms_pulse_still_starts()
