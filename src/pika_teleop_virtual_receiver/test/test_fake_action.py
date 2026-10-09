"""ROS integration check using an isolated domain and a fake Action Server."""

import os
import threading
import time
import unittest
from types import SimpleNamespace

try:
    import rclpy
    from rclpy.action import ActionServer, GoalResponse
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from pika_teleop_interfaces.msg import PikaTeleopState
    from pika_teleop_interfaces.srv import SetTeleopEnabled
    from realman_msgs.action import ExecuteMotion
    from pika_teleop_virtual_receiver.receiver_node import PikaTeleopVirtualReceiver
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS 2 Python packages are unavailable')
class TestFakeAction(unittest.TestCase):
    def setUp(self):
        self.old_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
        rclpy.init()
        self.goal_received = threading.Event()
        self.release_goal = threading.Event()
        self.rejection_seen = threading.Event()
        self.reject_goal = False
        self.received_goal = None
        joints = [12.172, 25.223, 73.054, -16.703, 80.307, 14.455]
        params = [
            Parameter('reset_on_user_stop', value=True),
            Parameter('state_timeout_ms', value=2000.0),
            Parameter('left_reset_action', value='/test/l/execute_motion'),
            Parameter('right_reset_action', value='/test/r/execute_motion'),
            Parameter('left_reset_joint_degrees', value=joints),
            Parameter('right_reset_joint_degrees', value=[0.0] * 6),
            Parameter('reset_dispatch_delay_ms', value=150.0),
            Parameter('reset_stop_confirm_timeout_ms', value=1000.0),
        ]
        self.receiver = PikaTeleopVirtualReceiver(parameter_overrides=params)
        self.fake_node = Node('fake_movej_action_server')
        self.fake_server = ActionServer(
            self.fake_node, ExecuteMotion, '/test/l/execute_motion',
            execute_callback=self._execute_goal,
            goal_callback=self._goal_callback,
        )
        self.test_node = Node('bag_reset_test_driver')
        self.services = {
            side: self.test_node.create_client(
                SetTeleopEnabled, f'/pika_teleop/{side}/set_enabled'
            )
            for side in ('left', 'right')
        }
        self.left_state = self.test_node.create_publisher(
            PikaTeleopState, '/pika_teleop/left/state',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.right_state = self.test_node.create_publisher(
            PikaTeleopState, '/pika_teleop/right/state',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.receiver_executor = SingleThreadedExecutor()
        self.receiver_executor.add_node(self.receiver)
        self.fake_executor = MultiThreadedExecutor(num_threads=2)
        self.fake_executor.add_node(self.fake_node)
        self.receiver_thread = threading.Thread(
            target=self.receiver_executor.spin, daemon=True
        )
        self.fake_thread = threading.Thread(
            target=self.fake_executor.spin, daemon=True
        )
        self.receiver_thread.start()
        self.fake_thread.start()
        self.assertTrue(self.services['left'].wait_for_service(timeout_sec=3.0))
        self.assertTrue(self.services['right'].wait_for_service(timeout_sec=3.0))
        self.assertTrue(self._wait_until(
            lambda: self.left_state.get_subscription_count() > 0, 3.0
        ))

    def tearDown(self):
        self.release_goal.set()
        self.fake_server.destroy()
        self.receiver_executor.shutdown()
        self.fake_executor.shutdown()
        self.receiver_thread.join(timeout=3.0)
        self.fake_thread.join(timeout=3.0)
        self.test_node.destroy_node()
        self.fake_node.destroy_node()
        self.receiver.destroy_node()
        rclpy.shutdown()
        if self.old_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = self.old_domain

    def _execute_goal(self, goal_handle):
        self.received_goal = goal_handle.request
        self.goal_received.set()
        self.release_goal.wait(timeout=5.0)
        goal_handle.succeed()
        return ExecuteMotion.Result()

    def _goal_callback(self, request):
        if self.reject_goal:
            self.rejection_seen.set()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _wait_until(predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def _service_call(self, enable, reason, side='left'):
        request = SetTeleopEnabled.Request()
        request.enable = enable
        request.reason = reason
        future = self.services[side].call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=2.0)
        self.assertTrue(future.done(), 'STOP/START service blocked on Action')
        return future.result()

    @staticmethod
    def _state(enabled):
        state = PikaTeleopState()
        state.enabled = enabled
        state.valid = enabled
        state.pose.orientation.w = 1.0
        return state

    def _publish_until(self, publisher, message, predicate, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            publisher.publish(message)
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    def test_goal_does_not_block_stop_or_state_callbacks(self):
        self.assertTrue(self._service_call(True, 'USER_START').success)
        self.assertTrue(self._publish_until(
            self.left_state, self._state(True),
            lambda: self.receiver._reset_handoffs['left'].active,
        ))
        start = time.monotonic()
        self.assertTrue(self._service_call(False, 'USER_STOP').success)
        self.assertLess(time.monotonic() - start, 1.0)
        self.assertFalse(self._service_call(True, 'USER_START').success)
        self.assertTrue(self._service_call(True, 'USER_START', 'right').success)
        time.sleep(0.25)
        self.assertFalse(self.goal_received.is_set(), 'Goal preceded disabled State')
        self.assertTrue(self._publish_until(
            self.left_state, self._state(False), self.goal_received.is_set,
        ))
        self.assertEqual(self.received_goal.command, ExecuteMotion.Goal.MOVEJ)
        self.assertEqual(self.received_goal.reference_type, ExecuteMotion.Goal.BASE)
        self.assertEqual(
            list(self.received_goal.joint_degrees),
            [12.172, 25.223, 73.054, -16.703, 80.307, 14.455],
        )
        self.assertEqual(self.received_goal.velocity_percent, 10)
        self.assertFalse(self.release_goal.is_set())
        self.assertTrue(self._wait_until(
            lambda: 'left' not in self.receiver._reset_goal_futures, 1.0
        ), 'Goal acceptance callback did not finish')
        before = self.receiver.status['right'].rx_count
        self.assertTrue(self._publish_until(
            self.right_state, self._state(False),
            lambda: self.receiver.status['right'].rx_count > before,
        ), 'other-side State reception blocked by unfinished MoveJ')

    def test_missing_server_keeps_stop_successful(self):
        self.assertTrue(self._service_call(True, 'USER_START', 'right').success)
        self.assertTrue(self._publish_until(
            self.right_state, self._state(True),
            lambda: self.receiver._reset_handoffs['right'].active,
        ))
        self.assertTrue(self._service_call(False, 'USER_STOP', 'right').success)
        self.assertTrue(self._publish_until(
            self.right_state, self._state(False),
            lambda: self.receiver._reset_handoffs['right'].pending is None,
        ))
        self.assertNotIn('right', self.receiver._reset_goal_futures)
        self.assertFalse(self.goal_received.is_set())

    def test_rejected_goal_keeps_stop_successful(self):
        self.reject_goal = True
        self.assertTrue(self._service_call(True, 'USER_START').success)
        self.assertTrue(self._publish_until(
            self.left_state, self._state(True),
            lambda: self.receiver._reset_handoffs['left'].active,
        ))
        self.assertTrue(self._service_call(False, 'USER_STOP').success)
        self.assertTrue(self._publish_until(
            self.left_state, self._state(False), self.rejection_seen.is_set,
        ))
        self.assertTrue(self._wait_until(
            lambda: 'left' not in self.receiver._reset_goal_futures, 1.0
        ))
        self.assertFalse(self.goal_received.is_set())

    def test_send_exception_keeps_stop_successful(self):
        class FailingClient:
            @staticmethod
            def server_is_ready():
                return True

            @staticmethod
            def send_goal_async(_goal):
                raise RuntimeError('simulated send failure')

        self.receiver._reset_clients['left'] = FailingClient()
        self.assertTrue(self._service_call(True, 'USER_START').success)
        self.assertTrue(self._publish_until(
            self.left_state, self._state(True),
            lambda: self.receiver._reset_handoffs['left'].active,
        ))
        self.assertTrue(self._service_call(False, 'USER_STOP').success)
        self.assertTrue(self._publish_until(
            self.left_state, self._state(False),
            lambda: self.receiver._reset_handoffs['left'].pending is None,
        ))
        self.assertNotIn('left', self.receiver._reset_goal_futures)
        self.assertFalse(self.goal_received.is_set())

    def test_invalid_joint_configuration_is_rejected(self):
        fake_node = SimpleNamespace(
            get_parameter=lambda _name: SimpleNamespace(value=[float('nan')] * 6)
        )
        with self.assertRaisesRegex(ValueError, 'six_bad must contain 6 finite'):
            PikaTeleopVirtualReceiver._array_parameter(fake_node, 'six_bad', 6)


@unittest.skipIf(rclpy is None, 'ROS 2 Python packages are unavailable')
class TestFeatureDisabled(unittest.TestCase):
    def test_switch_off_creates_no_action_client(self):
        old_domain = os.environ.get('ROS_DOMAIN_ID')
        os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
        rclpy.init()
        node = None
        try:
            node = PikaTeleopVirtualReceiver(parameter_overrides=[
                Parameter('reset_on_user_stop', value=False),
            ])
            self.assertEqual(node._reset_clients, {})
            self.assertEqual(node._reset_handoffs, {})
            request = SetTeleopEnabled.Request()
            request.enable = False
            request.reason = 'USER_STOP'
            response = node._service_callback(
                'left', request, SetTeleopEnabled.Response()
            )
            self.assertTrue(response.success)
            self.assertEqual(node._reset_goal_futures, {})
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
            if old_domain is None:
                os.environ.pop('ROS_DOMAIN_ID', None)
            else:
                os.environ['ROS_DOMAIN_ID'] = old_domain


if __name__ == '__main__':
    unittest.main()
