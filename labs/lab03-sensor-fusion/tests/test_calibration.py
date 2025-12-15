#!/usr/bin/env python3
"""
Test Suite 2: Sensor Calibration
Lab 3: Sensor Fusion (IMU + RealSense Camera)

Tests:
1. IMU calibration parameters are loaded
2. Gyroscope bias is within acceptable range
3. Accelerometer calibration is correct
4. Complementary filter alpha parameter is optimal
5. Coordinate frame transformations are correct
"""

import unittest
import math
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'starter'))

try:
    from sensor_fusion_package.sensor_fusion_node import SensorFusionNode
except ImportError:
    print("⚠️  Could not import SensorFusionNode")
    sys.exit(1)

import rclpy
from sensor_msgs.msg import Imu


class TestCalibration(unittest.TestCase):
    """Test sensor calibration and parameters."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS 2."""
        rclpy.shutdown()

    def test_01_alpha_parameter_valid(self):
        """Test that complementary filter alpha is in valid range."""
        node = SensorFusionNode()

        self.assertGreaterEqual(
            node.alpha,
            0.9,
            "❌ Alpha too low (< 0.9). Gyro won't be trusted enough."
        )
        self.assertLessEqual(
            node.alpha,
            0.99,
            "❌ Alpha too high (> 0.99). Accelerometer correction won't work."
        )

        print(f"✅ Complementary filter alpha: {node.alpha} (optimal: 0.95-0.98)")
        node.destroy_node()

    def test_02_initial_orientation_zero(self):
        """Test that initial orientation is zero."""
        node = SensorFusionNode()

        self.assertEqual(node.roll, 0.0, "❌ Initial roll should be 0")
        self.assertEqual(node.pitch, 0.0, "❌ Initial pitch should be 0")
        self.assertEqual(node.yaw, 0.0, "❌ Initial yaw should be 0")

        print("✅ Initial orientation is zero")
        node.destroy_node()

    def test_03_gravity_calculation_correct(self):
        """Test that gravity vector gives correct roll/pitch."""
        node = SensorFusionNode()

        # Test case 1: Robot upright (gravity = -9.81 in Z)
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Process multiple times to let filter converge
        for _ in range(100):
            node.imu_callback(imu_msg)

        # Roll and pitch should be near zero
        self.assertAlmostEqual(
            node.roll,
            0.0,
            delta=0.1,
            msg=f"❌ Roll should be ~0° when upright, got {math.degrees(node.roll):.1f}°"
        )
        self.assertAlmostEqual(
            node.pitch,
            0.0,
            delta=0.1,
            msg=f"❌ Pitch should be ~0° when upright, got {math.degrees(node.pitch):.1f}°"
        )

        print(f"✅ Gravity calculation correct (roll={math.degrees(node.roll):.2f}°, pitch={math.degrees(node.pitch):.2f}°)")
        node.destroy_node()

    def test_04_roll_positive_y_acceleration(self):
        """Test that positive Y acceleration gives positive roll."""
        node = SensorFusionNode()

        # Tilt to the left (positive Y acceleration)
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 5.0  # Positive Y
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Let filter converge
        for _ in range(100):
            node.imu_callback(imu_msg)

        # Roll should be positive
        self.assertGreater(
            node.roll,
            0.0,
            f"❌ Positive Y acceleration should give positive roll. Got {math.degrees(node.roll):.1f}°"
        )

        print(f"✅ Roll direction correct (Y=+5.0 → roll={math.degrees(node.roll):.1f}°)")
        node.destroy_node()

    def test_05_pitch_negative_x_acceleration(self):
        """Test that negative X acceleration gives positive pitch."""
        node = SensorFusionNode()

        # Tilt forward (negative X acceleration)
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = -5.0  # Negative X
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Let filter converge
        for _ in range(100):
            node.imu_callback(imu_msg)

        # Pitch should be positive
        self.assertGreater(
            node.pitch,
            0.0,
            f"❌ Negative X acceleration should give positive pitch. Got {math.degrees(node.pitch):.1f}°"
        )

        print(f"✅ Pitch direction correct (X=-5.0 → pitch={math.degrees(node.pitch):.1f}°)")
        node.destroy_node()

    def test_06_gyroscope_integration(self):
        """Test that gyroscope integration works correctly."""
        node = SensorFusionNode()

        # Apply constant angular velocity for 1 second
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81
        imu_msg.angular_velocity.x = math.radians(90)  # 90°/s
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        # Simulate 100 callbacks at 100Hz (1 second total)
        import time
        for _ in range(100):
            node.imu_callback(imu_msg)
            time.sleep(0.01)

        # Roll should be approximately 90° (considering complementary filter)
        # Alpha = 0.98, so gyro contributes 98% → expect ~88°
        expected_roll_deg = 88.0
        actual_roll_deg = math.degrees(node.roll)

        self.assertAlmostEqual(
            actual_roll_deg,
            expected_roll_deg,
            delta=10.0,
            msg=f"❌ After 1s at 90°/s, roll should be ~{expected_roll_deg:.0f}°, got {actual_roll_deg:.1f}°"
        )

        print(f"✅ Gyroscope integration works (expected ~{expected_roll_deg:.0f}°, got {actual_roll_deg:.1f}°)")
        node.destroy_node()

    def test_07_euler_to_quaternion_identity(self):
        """Test that identity rotation gives correct quaternion."""
        node = SensorFusionNode()

        # Identity rotation (0, 0, 0)
        q = node.euler_to_quaternion(0.0, 0.0, 0.0)

        self.assertAlmostEqual(q.w, 1.0, delta=0.01, msg="❌ Identity qw should be 1")
        self.assertAlmostEqual(q.x, 0.0, delta=0.01, msg="❌ Identity qx should be 0")
        self.assertAlmostEqual(q.y, 0.0, delta=0.01, msg="❌ Identity qy should be 0")
        self.assertAlmostEqual(q.z, 0.0, delta=0.01, msg="❌ Identity qz should be 0")

        print(f"✅ Identity quaternion correct: [{q.w:.3f}, {q.x:.3f}, {q.y:.3f}, {q.z:.3f}]")
        node.destroy_node()

    def test_08_euler_to_quaternion_90deg_roll(self):
        """Test that 90° roll gives correct quaternion."""
        node = SensorFusionNode()

        # 90° roll around X
        q = node.euler_to_quaternion(math.pi/2, 0.0, 0.0)

        # Expected: qw = cos(45°) = 0.707, qx = sin(45°) = 0.707
        self.assertAlmostEqual(q.w, 0.707, delta=0.01, msg="❌ 90° roll qw incorrect")
        self.assertAlmostEqual(q.x, 0.707, delta=0.01, msg="❌ 90° roll qx incorrect")
        self.assertAlmostEqual(q.y, 0.0, delta=0.01, msg="❌ 90° roll qy should be 0")
        self.assertAlmostEqual(q.z, 0.0, delta=0.01, msg="❌ 90° roll qz should be 0")

        print(f"✅ 90° roll quaternion correct: [{q.w:.3f}, {q.x:.3f}, {q.y:.3f}, {q.z:.3f}]")
        node.destroy_node()


def run_tests():
    """Run all calibration tests and print summary."""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCalibration)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "="*70)
    if result.wasSuccessful():
        print("✅ ALL CALIBRATION TESTS PASSED!")
        print(f"   - {result.testsRun} tests run")
        print(f"   - 0 failures")
        print(f"   - 0 errors")
        print("\n   Your sensor calibration is correct!")
    else:
        print("❌ SOME TESTS FAILED")
        print(f"   - {result.testsRun} tests run")
        print(f"   - {len(result.failures)} failures")
        print(f"   - {len(result.errors)} errors")
        print("\n   Check your complementary filter implementation (TODOs 3-6)")
    print("="*70 + "\n")

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
