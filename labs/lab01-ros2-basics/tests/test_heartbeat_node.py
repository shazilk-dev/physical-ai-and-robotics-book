#!/usr/bin/env python3
"""
Test suite for Lab 1: Heartbeat Publisher Node
Tests node lifecycle, topic creation, message format, and publishing rate.
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TestHeartbeatNode(unittest.TestCase):
    """Test cases for the heartbeat_node implementation."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 before all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create a test node for each test case."""
        self.test_node = rclpy.create_node('test_heartbeat_node')

    def tearDown(self):
        """Clean up test node after each test."""
        self.test_node.destroy_node()

    def test_topic_exists(self):
        """Test that /robot/ready topic is created."""
        # Wait for topic to appear (up to 5 seconds)
        timeout = 5.0
        start_time = time.time()
        
        topic_found = False
        while time.time() - start_time < timeout:
            topic_list = self.test_node.get_topic_names_and_types()
            if '/robot/ready' in [topic[0] for topic in topic_list]:
                topic_found = True
                break
            time.sleep(0.1)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
        
        self.assertTrue(topic_found, "Topic /robot/ready not found")

    def test_message_format(self):
        """Test that messages follow the expected format: 'READY: Heartbeat #N at TIMESTAMP'."""
        received_messages = []

        def callback(msg):
            received_messages.append(msg.data)

        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            callback,
            10
        )

        # Collect messages for 2.5 seconds (should get 2-3 messages at 1 Hz)
        timeout = 2.5
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertGreater(len(received_messages), 0, "No messages received")

        # Validate format
        for msg_data in received_messages:
            self.assertTrue(msg_data.startswith('READY: Heartbeat #'),
                            f"Message format incorrect: {msg_data}")
            self.assertIn(' at ', msg_data, f"Timestamp missing in message: {msg_data}")

        # Clean up
        self.test_node.destroy_subscription(subscription)

    def test_publishing_rate(self):
        """Test that messages are published at approximately 1 Hz (±10%)."""
        received_times = []

        def callback(msg):
            received_times.append(time.time())

        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            callback,
            10
        )

        # Collect messages for 5 seconds (expect ~5 messages)
        timeout = 5.5
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertGreaterEqual(len(received_times), 3,
                                "Not enough messages received to measure rate")

        # Calculate intervals between messages
        intervals = [received_times[i] - received_times[i - 1]
                     for i in range(1, len(received_times))]

        # Average interval should be close to 1.0 second (±10% = 0.9 to 1.1 seconds)
        avg_interval = sum(intervals) / len(intervals)
        self.assertGreaterEqual(avg_interval, 0.9,
                                f"Publishing too fast: {avg_interval:.2f}s average interval")
        self.assertLessEqual(avg_interval, 1.1,
                             f"Publishing too slow: {avg_interval:.2f}s average interval")

        # Clean up
        self.test_node.destroy_subscription(subscription)

    def test_counter_increments(self):
        """Test that the heartbeat counter increments correctly."""
        received_counters = []

        def callback(msg):
            # Extract counter from message: "READY: Heartbeat #N at ..."
            try:
                counter_str = msg.data.split('#')[1].split(' ')[0]
                received_counters.append(int(counter_str))
            except (IndexError, ValueError):
                pass  # Ignore malformed messages

        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            callback,
            10
        )

        # Collect messages for 3 seconds
        timeout = 3.5
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertGreaterEqual(len(received_counters), 2,
                                "Not enough messages to verify counter")

        # Verify counters increment by 1
        for i in range(1, len(received_counters)):
            self.assertEqual(received_counters[i], received_counters[i - 1] + 1,
                             f"Counter did not increment correctly: {received_counters}")

        # Clean up
        self.test_node.destroy_subscription(subscription)


if __name__ == '__main__':
    unittest.main()
