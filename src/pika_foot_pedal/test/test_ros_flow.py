"""Isolated ROS graph with fake Bridge services and State; no robot nodes."""

import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from pika_foot_pedal.node import FootPedalNode
from pika_teleop_interfaces.msg import PikaTeleopState
from pika_teleop_interfaces.srv import SetTeleopEnabled
from std_msgs.msg import Bool


class TestRosFlow(unittest.TestCase):
    def test_left_active_before_right_start_and_both_stop(self):
        # Domain 211 is deliberately independent of the deployment domain 65.
        rclpy.init(domain_id=211, args=['--ros-args',
            '-r', '/pika_teleop/left/manual_enable:=/test_foot/left/manual_enable',
            '-r', '/pika_teleop/right/manual_enable:=/test_foot/right/manual_enable',
            '-r', '/pika_teleop/left/state:=/test_foot/left/state',
            '-r', '/pika_teleop/right/state:=/test_foot/right/state',
            '-r', '/foot_pedal/pressed:=/test_foot/pressed',
            '-r', '/foot_pedal/enabled:=/test_foot/enabled'])
        pedal = FootPedalNode()
        pedal._read_device = lambda _now_ns: None
        pedal.debouncer.initialize(False, time.monotonic_ns())
        fake = Node('test_foot_bridge')
        calls = []
        left_enabled = False
        right_enabled = False
        left_state_published = False
        pressed = []
        enabled_messages = []

        def service(side):
            def callback(request, response):
                nonlocal left_enabled, right_enabled
                calls.append((side, request.enable, left_state_published))
                if side == 'left':
                    left_enabled = request.enable
                else:
                    right_enabled = request.enable
                response.success = True
                response.message = 'fake accepted'
                return response
            return callback

        services = [fake.create_service(
            SetTeleopEnabled, f'/test_foot/{side}/manual_enable', service(side)
        ) for side in ('left', 'right')]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        states = {
            side: fake.create_publisher(PikaTeleopState, f'/test_foot/{side}/state', qos)
            for side in ('left', 'right')
        }
        def publish_states():
            nonlocal left_state_published
            for side, enabled in (('left', left_enabled), ('right', right_enabled)):
                state = PikaTeleopState()
                state.enabled = enabled
                state.valid = enabled
                states[side].publish(state)
                if side == 'left' and enabled:
                    left_state_published = True
        timer = fake.create_timer(0.02, publish_states)
        subscriber = fake.create_subscription(
            Bool, '/test_foot/pressed', lambda msg: pressed.append(msg.data), 1
        )
        enabled_subscriber = fake.create_subscription(
            Bool, '/test_foot/enabled', lambda msg: enabled_messages.append(msg.data), 1
        )
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(fake)
        executor.add_node(pedal)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        try:
            end = time.monotonic() + 3
            while time.monotonic() < end and not all(
                client.service_is_ready() for client in pedal.enable_clients.values()
            ):
                time.sleep(0.01)
            self.assertTrue(all(
                client.service_is_ready() for client in pedal.enable_clients.values()
            ))
            time.sleep(0.1)
            pedal.debouncer.event(1, time.monotonic_ns())
            end = time.monotonic() + 3
            while time.monotonic() < end and pedal.phase != 'RUNNING':
                time.sleep(0.01)
            self.assertEqual(pedal.phase, 'RUNNING')
            self.assertEqual(calls[:2], [
                ('left', True, False), ('right', True, True)
            ])
            pedal.debouncer.event(0, time.monotonic_ns())
            time.sleep(0.15)
            self.assertEqual(pedal.phase, 'RUNNING')
            self.assertFalse(any(not enable for _, enable, _ in calls))
            pedal.debouncer.event(1, time.monotonic_ns())
            end = time.monotonic() + 3
            while time.monotonic() < end and sum(not enable for _, enable, _ in calls) < 2:
                time.sleep(0.01)
            self.assertEqual({side for side, enable, _ in calls if not enable},
                             {'left', 'right'})
            self.assertIn(True, pressed)
            self.assertIn(False, pressed)
            self.assertIn(True, enabled_messages)
            end = time.monotonic() + 1
            while time.monotonic() < end and (not enabled_messages or enabled_messages[-1]):
                time.sleep(0.01)
            self.assertFalse(enabled_messages[-1])
        finally:
            pedal.close()
            executor.shutdown()
            worker.join(timeout=2)
            pedal.destroy_node()
            fake.destroy_node()
            rclpy.shutdown()
