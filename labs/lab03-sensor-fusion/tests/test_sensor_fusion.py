#!/usr/bin/env python3
"""
Test Suite 1: Sensor Fusion Functionality
Lab 3: Sensor Fusion (IMU + RealSense Camera)

Tests:
1. Node starts successfully
2. IMU subscriber exists
3. Depth subscriber exists
4. State publisher publishes at 50Hz
5. Orientation values are valid
6. Complementary filter produces smooth output
7. Obstacle detection triggers correctly
8. Quaternion normalization is correct
"""

import unittest
import rclpy
from rclpy.node import Node
import time
import math
import sys
import os

# Add parent directory to path to import the node
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'starter'))

try:
    from sensor_fusion_package.sensor_fusion_node import SensorFusionNode
except ImportError:
    print("⚠️  Could not import SensorFusionNode from starter package.")
    print("   Make sure you've completed the TODOs in sensor_fusion_node.py")
    sys.exit(1)

from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class TestSensorFusion(unittest.TestCase):
    """Test sensor fusion node functionality."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 and create test node."""
        rclpy.init()
        cls.test_node = Node('test_sensor_fusion')

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS 2."""
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def test_01_node_starts(self):
        """Test that SensorFusionNode starts without errors."""
        try:
            node = SensorFusionNode()
            node.destroy_node()
            print("✅ Node starts successfully")
        except Exception as e:
            self.fail(f"❌ Node failed to start: {e}")

    def test_02_imu_subscriber_exists(self):
        """Test that IMU subscriber is created on /imu/data."""
        node = SensorFusionNode()

        # Check subscribers
        subscriptions = node.get_subscriptions_info_by_topic('/imu/data')

        self.assertTrue(
            len(subscriptions) > 0,
            "❌ No subscriber found on /imu/data. Check TODO 1."
        )

        print(f"✅ IMU subscriber exists on /imu/data")
        node.destroy_node()

    def test_03_depth_subscriber_exists(self):
        """Test that depth subscriber is created."""
        node = SensorFusionNode()

        subscriptions = node.get_subscriptions_info_by_topic('/camera/depth/image_rect_raw')

        self.assertTrue(
            len(subscriptions) > 0,
            "❌ No subscriber found on /camera/depth/image_rect_raw. Check TODO 7."
        )

        print("✅ Depth subscriber exists")
        node.destroy_node()

    def test_04_state_publisher_exists(self):
        """Test that state publisher is created on /robot/state."""
        node = SensorFusionNode()

        publishers = node.get_publishers_info_by_topic('/robot/state')

        self.assertTrue(
            len(publishers) > 0,
            "❌ No publisher found on /robot/state. Check TODO 10."
        )

        print("✅ State publisher exists on /robot/state")
        node.destroy_node()

    def test_05_publish_rate(self):
        """Test that state is published at approximately 50Hz."""
        node = SensorFusionNode()

        # Create subscriber to count messages
        message_count = [0]
        def callback(msg):
            message_count[0] += 1

        subscriber = self.test_node.create_subscription(
            PoseStamped,
            '/robot/state',
            callback,
            10
        )

        # Collect messages for 1 second
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(node, timeout_sec=0.01)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        # Should receive ~50 messages in 1 second (±5 tolerance)
        self.assertGreaterEqual(
            message_count[0],
            45,
            f"❌ Expected ~50 messages/sec, got {message_count[0]}. Check timer frequency."
        )
        self.assertLessEqual(
            message_count[0],
            55,
            f"❌ Publishing too fast: {message_count[0]} messages/sec (expected ~50)"
        )

        print(f"✅ Publish rate: {message_count[0]} Hz (target: 50 Hz)")

        self.test_node.destroy_subscription(subscriber)
        node.destroy_node()

    def test_06_orientation_valid_range(self):
        """Test that orientation values are within valid range."""
        node = SensorFusionNode()

        # Simulate IMU data
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # 1g downward
        imu_msg.angular_velocity.x = 0.1
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Process several IMU callbacks
        for _ in range(10):
            node.imu_callback(imu_msg)
            time.sleep(0.01)

        # Check that roll, pitch, yaw are within [-π, π]
        self.assertGreaterEqual(node.roll, -math.pi, "❌ Roll < -π")
        self.assertLessEqual(node.roll, math.pi, "❌ Roll > π")

        self.assertGreaterEqual(node.pitch, -math.pi, "❌ Pitch < -π")
        self.assertLessEqual(node.pitch, math.pi, "❌ Pitch > π")

        self.assertGreaterEqual(node.yaw, -math.pi, "❌ Yaw < -π")
        self.assertLessEqual(node.yaw, math.pi, "❌ Yaw > π")

        print(f"✅ Orientation values valid: Roll={node.roll:.2f}, Pitch={node.pitch:.2f}, Yaw={node.yaw:.2f}")
        node.destroy_node()

    def test_07_complementary_filter_smoothness(self):
        """Test that complementary filter produces smooth output."""
        node = SensorFusionNode()

        # Simulate noisy accelerometer data
        roll_values = []

        for i in range(50):
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.5 + 0.1 * (i % 2)  # Noisy
            imu_msg.linear_acceleration.z = 9.81
            imu_msg.angular_velocity.x = 0.01
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0

            node.imu_callback(imu_msg)
            roll_values.append(node.roll)
            time.sleep(0.01)

        # Calculate variance (should be low due to filtering)
        import numpy as np
        variance = np.var(roll_values[10:])  # Ignore first 10 (settling)

        self.assertLess(
            variance,
            0.01,
            f"❌ Filter output too noisy (variance={variance:.4f}). Check TODO 5."
        )

        print(f"✅ Complementary filter smoothness: variance={variance:.6f}")
        node.destroy_node()

    def test_08_quaternion_normalized(self):
        """Test that quaternions are properly normalized."""
        node = SensorFusionNode()

        # Set some orientation
        node.roll = 0.5
        node.pitch = 0.3
        node.yaw = 0.0

        # Get quaternion
        q = node.euler_to_quaternion(node.roll, node.pitch, node.yaw)

        # Calculate magnitude
        magnitude = math.sqrt(q.w**2 + q.x**2 + q.y**2 + q.z**2)

        self.assertAlmostEqual(
            magnitude,
            1.0,
            delta=0.01,
            msg=f"❌ Quaternion not normalized (magnitude={magnitude:.4f})"
        )

        print(f"✅ Quaternion normalized: magnitude={magnitude:.6f}")
        node.destroy_node()

    def test_09_obstacle_detection(self):
        """Test that obstacle detection works correctly."""
        node = SensorFusionNode()

        # Create test depth image (close obstacle at 0.5m)
        import numpy as np
        from cv_bridge import CvBridge

        bridge = CvBridge()

        # Create 640x480 depth image with 500mm at center
        depth_array = np.full((480, 640), 500, dtype=np.uint16)  # 0.5m in mm

        depth_msg = bridge.cv2_to_imgmsg(depth_array, encoding='16UC1')

        # Track obstacle messages
        obstacle_detected = [False]

        def obstacle_callback(msg):
            if 'OBSTACLE' in msg.data:
                obstacle_detected[0] = True

        obstacle_sub = self.test_node.create_subscription(
            String,
            '/robot/obstacles',
            obstacle_callback,
            10
        )

        # Process depth callback
        node.depth_callback(depth_msg)

        # Spin briefly to receive obstacle message
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.01)
            rclpy.spin_once(self.test_node, timeout_sec=0.01)

        self.assertTrue(
            obstacle_detected[0] or node.obstacle_distance < 1.0,
            "❌ Obstacle not detected at 0.5m. Check TODO 9."
        )

        print(f"✅ Obstacle detection works (distance={node.obstacle_distance:.2f}m)")

        self.test_node.destroy_subscription(obstacle_sub)
        node.destroy_node()


def run_tests():
    """Run all tests and print summary."""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSensorFusion)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "="*70)
    if result.wasSuccessful():
        print("✅ ALL SENSOR FUSION TESTS PASSED!")
        print(f"   - {result.testsRun} tests run")
        print(f"   - 0 failures")
        print(f"   - 0 errors")
        print("\n   Your sensor fusion implementation is correct!")
        print("\n   Next steps:")
        print("   1. Compare with solution: diff starter/ solutions/")
        print("   2. Test with real hardware (RealSense D435i)")
        print("   3. Try bonus challenges (EKF, magnetometer)")
    else:
        print("❌ SOME TESTS FAILED")
        print(f"   - {result.testsRun} tests run")
        print(f"   - {len(result.failures)} failures")
        print(f"   - {len(result.errors)} errors")
        print("\n   Fix the errors above and re-run the tests.")
        print("   Hint: Check the TODO numbers mentioned in error messages.")
    print("="*70 + "\n")

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
