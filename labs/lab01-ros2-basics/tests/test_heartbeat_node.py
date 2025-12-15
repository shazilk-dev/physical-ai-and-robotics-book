#!/usr/bin/env python3
"""
Automated Tests for Lab 1: Heartbeat Publisher Node
Physical AI & Humanoid Robotics Textbook

Tests verify:
1. Node publishes to correct topic
2. Message format is correct
3. Publish rate is approximately 1Hz
4. Counter increments correctly
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import re


class TestHeartbeatNode(unittest.TestCase):
    """
    Test suite for heartbeat publisher node.

    These tests validate the lab implementation meets all requirements.
    """

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create a test node before each test."""
        self.test_node = Node('test_heartbeat_node')
        self.messages_received = []
        self.receive_times = []

    def tearDown(self):
        """Cleanup after each test."""
        self.test_node.destroy_node()

    def test_topic_exists(self):
        """Test 1: Verify /robot/ready topic exists."""
        # Create subscriber to /robot/ready
        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            lambda msg: None,  # Dummy callback
            10
        )

        # Spin briefly to allow discovery
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if topic appears in topic list
        topic_names_and_types = self.test_node.get_topic_names_and_types()
        topic_names = [name for name, _ in topic_names_and_types]

        self.assertIn(
            '/robot/ready',
            topic_names,
            "❌ Topic '/robot/ready' does not exist. Did you create the publisher?"
        )

        subscription.destroy()

    def test_message_format(self):
        """Test 2: Verify message format is correct."""

        def message_callback(msg):
            self.messages_received.append(msg.data)
            self.receive_times.append(time.time())

        # Subscribe to heartbeat topic
        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            message_callback,
            10
        )

        # Collect messages for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Verify we received at least 2 messages
        self.assertGreaterEqual(
            len(self.messages_received),
            2,
            "❌ Did not receive enough messages. Is the timer working?"
        )

        # Check message format: "READY: Heartbeat #N at YYYY-MM-DDTHH:MM:SS..."
        pattern = r'^READY: Heartbeat #\d+ at \d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}'

        for msg in self.messages_received:
            self.assertIsNotNone(
                re.match(pattern, msg),
                f"❌ Message format incorrect: '{msg}'\n"
                f"   Expected format: 'READY: Heartbeat #N at YYYY-MM-DDTHH:MM:SS...'"
            )

        subscription.destroy()

    def test_counter_increments(self):
        """Test 3: Verify counter increments correctly."""

        def message_callback(msg):
            self.messages_received.append(msg.data)

        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            message_callback,
            10
        )

        # Collect messages for 4 seconds (should get 4-5 heartbeats)
        start_time = time.time()
        while time.time() - start_time < 4.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Extract counter values from messages
        counters = []
        for msg in self.messages_received:
            match = re.search(r'Heartbeat #(\d+)', msg)
            if match:
                counters.append(int(match.group(1)))

        # Verify counters increment by 1 each time
        for i in range(len(counters) - 1):
            self.assertEqual(
                counters[i + 1],
                counters[i] + 1,
                f"❌ Counter did not increment correctly: {counters[i]} → {counters[i+1]}"
            )

        subscription.destroy()

    def test_publish_rate(self):
        """Test 4: Verify publish rate is approximately 1Hz."""

        def message_callback(msg):
            self.messages_received.append(msg.data)
            self.receive_times.append(time.time())

        subscription = self.test_node.create_subscription(
            String,
            '/robot/ready',
            message_callback,
            10
        )

        # Collect messages for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Should have received 4-6 messages (5 expected ± 1 tolerance)
        self.assertGreaterEqual(
            len(self.messages_received),
            4,
            "❌ Too few messages received. Check timer period (should be 1.0 second)"
        )

        self.assertLessEqual(
            len(self.messages_received),
            6,
            "❌ Too many messages received. Check timer period (should be 1.0 second)"
        )

        # Calculate average time between messages
        if len(self.receive_times) >= 2:
            intervals = [
                self.receive_times[i+1] - self.receive_times[i]
                for i in range(len(self.receive_times) - 1)
            ]
            avg_interval = sum(intervals) / len(intervals)

            # Average interval should be close to 1.0 second (±0.2s tolerance)
            self.assertAlmostEqual(
                avg_interval,
                1.0,
                delta=0.2,
                msg=f"❌ Publish rate incorrect. "
                    f"Average interval: {avg_interval:.2f}s (expected ~1.0s)"
            )

        subscription.destroy()

    def test_node_name(self):
        """Test 5: Verify node name is 'heartbeat_publisher'."""
        # Get list of active nodes
        node_names = self.test_node.get_node_names()

        self.assertIn(
            'heartbeat_publisher',
            node_names,
            "❌ Node 'heartbeat_publisher' not found. "
            "Did you use the correct node name in __init__?"
        )


def run_tests():
    """
    Run all tests and print results.

    Returns:
        bool: True if all tests passed, False otherwise
    """
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestHeartbeatNode)

    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print("\n" + "="*70)
    if result.wasSuccessful():
        print("✅ ALL TESTS PASSED! Lab 1 complete!")
        print(f"   - {result.testsRun} tests run")
        print(f"   - 0 failures")
        print(f"   - 0 errors")
    else:
        print("❌ SOME TESTS FAILED")
        print(f"   - {result.testsRun} tests run")
        print(f"   - {len(result.failures)} failures")
        print(f"   - {len(result.errors)} errors")
        print("\nReview the errors above and fix your implementation.")
    print("="*70 + "\n")

    return result.wasSuccessful()


if __name__ == '__main__':
    # Run tests when executed directly
    success = run_tests()
    exit(0 if success else 1)
