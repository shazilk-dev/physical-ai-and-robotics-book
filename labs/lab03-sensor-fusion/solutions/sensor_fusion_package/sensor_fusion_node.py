#!/usr/bin/env python3
"""
Lab 3: Sensor Fusion Node (COMPLETE SOLUTION)
============================================

This node fuses IMU and RealSense camera data to estimate robot orientation.

Features:
- Complementary filter for IMU fusion
- Depth-based obstacle detection
- 50Hz state estimation
- Quaternion output for ROS compatibility

Topics:
- Input: /imu/data (sensor_msgs/Imu)
- Input: /camera/depth/image_rect_raw (sensor_msgs/Image)
- Output: /robot/state (geometry_msgs/PoseStamped)
- Output: /robot/obstacles (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import math


class SensorFusionNode(Node):
    """
    Fuses IMU and camera data for robot state estimation using complementary filter.
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # State variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_imu_time = None

        # Complementary filter parameter
        self.alpha = 0.98  # High-pass filter coefficient (trust gyro 98%)

        # Camera processing
        self.bridge = CvBridge()
        self.obstacle_distance = float('inf')

        # TODO 1: Create IMU subscriber ✅
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # TODO 7: Create depth image subscriber ✅
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # TODO 10: Create state publisher ✅
        self.state_publisher = self.create_publisher(
            PoseStamped,
            '/robot/state',
            10
        )

        self.obstacle_publisher = self.create_publisher(
            String,
            '/robot/obstacles',
            10
        )

        # Timer for state publication (50 Hz)
        self.create_timer(0.02, self.publish_state)

        self.get_logger().info('🤖 Sensor Fusion Node Started')
        self.get_logger().info('   ✓ IMU subscriber: /imu/data')
        self.get_logger().info('   ✓ Depth subscriber: /camera/depth/image_rect_raw')
        self.get_logger().info('   ✓ State publisher: /robot/state (50Hz)')
        self.get_logger().info('   ✓ Obstacle publisher: /robot/obstacles')

    def imu_callback(self, msg: Imu):
        """
        Process IMU data and update orientation estimate using complementary filter.
        """
        current_time = self.get_clock().now()

        # TODO 2: Extract IMU data ✅
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # Calculate time step
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        else:
            dt = 0.01  # Initial value (100Hz assumed)

        self.last_imu_time = current_time

        # TODO 3: Calculate roll and pitch from accelerometer ✅
        # These give absolute orientation but are noisy
        roll_accel = math.atan2(accel_y, accel_z)
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # TODO 4: Integrate gyroscope data ✅
        # These are smooth but drift over time
        roll_gyro = self.roll + gyro_x * dt
        pitch_gyro = self.pitch + gyro_y * dt

        # TODO 5: Implement complementary filter ✅
        # Combine gyro (short-term accuracy) with accel (long-term accuracy)
        roll_fused = self.alpha * roll_gyro + (1 - self.alpha) * roll_accel
        pitch_fused = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_accel

        # TODO 6: Store filtered values ✅
        self.roll = roll_fused
        self.pitch = pitch_fused

        # Log orientation periodically (~5 Hz)
        if not hasattr(self, '_imu_callback_count'):
            self._imu_callback_count = 0

        self._imu_callback_count += 1
        if self._imu_callback_count % 20 == 0:
            self.get_logger().info(
                f'Orientation: Roll={math.degrees(self.roll):+6.1f}°, '
                f'Pitch={math.degrees(self.pitch):+6.1f}°, '
                f'Yaw={math.degrees(self.yaw):+6.1f}°'
            )

    def depth_callback(self, msg: Image):
        """
        Process depth image for obstacle detection.
        """
        try:
            # TODO 8: Process depth image ✅
            # Convert ROS Image to NumPy array (16-bit depth in mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

            # Extract center region (200x200 pixels)
            height, width = depth_image.shape
            center_y, center_x = height // 2, width // 2
            roi = depth_image[center_y-100:center_y+100, center_x-100:center_x+100]

            # Calculate average depth in meters (ignore zeros)
            valid_depths = roi[roi > 0]
            if len(valid_depths) > 0:
                avg_depth_mm = np.mean(valid_depths)
                avg_depth_m = avg_depth_mm / 1000.0
            else:
                avg_depth_m = float('inf')  # No valid depth data

            # TODO 9: Detect obstacles ✅
            if avg_depth_m < 1.0:
                obstacle_msg = String()
                obstacle_msg.data = f'OBSTACLE at {avg_depth_m:.2f}m'
                self.obstacle_publisher.publish(obstacle_msg)
                self.get_logger().warn(f'⚠️  {obstacle_msg.data}')

            # Store distance for confidence calculation
            self.obstacle_distance = avg_depth_m

        except Exception as e:
            self.get_logger().error(f'Depth processing failed: {e}')

    def publish_state(self):
        """
        Publish fused robot state at 50Hz.
        """
        # TODO 11: Populate PoseStamped message ✅
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Convert Euler angles to quaternion
        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.pose.orientation = q

        # Position is zero (we're only estimating orientation)
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        # TODO 12: Add quality metrics ✅
        confidence = self.calculate_confidence()

        if confidence < 0.7:
            self.get_logger().warn(
                f'Low confidence: {confidence:.2f} '
                f'(obstacle at {self.obstacle_distance:.2f}m)'
            )

        # Publish state
        self.state_publisher.publish(msg)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles (ZYX convention) to quaternion.

        Args:
            roll: Rotation around X-axis (radians)
            pitch: Rotation around Y-axis (radians)
            yaw: Rotation around Z-axis (radians)

        Returns:
            geometry_msgs/Quaternion (normalized)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def calculate_confidence(self) -> float:
        """
        Calculate orientation confidence based on sensor quality.

        Returns:
            Confidence score (0.0 to 1.0)
        """
        # Simple heuristic based on obstacle distance
        if self.obstacle_distance < 0.5:
            return 0.5  # Low confidence - too close
        elif self.obstacle_distance < 1.0:
            return 0.8  # Medium confidence
        else:
            return 1.0  # High confidence

        # Production implementation could include:
        # - IMU data variance (higher variance = lower confidence)
        # - Complementary filter convergence rate
        # - Sensor health checks (missing data, out-of-range values)
        # - Gyroscope bias estimation quality


def main(args=None):
    """
    Main entry point for sensor fusion node.
    """
    rclpy.init(args=args)

    try:
        node = SensorFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
