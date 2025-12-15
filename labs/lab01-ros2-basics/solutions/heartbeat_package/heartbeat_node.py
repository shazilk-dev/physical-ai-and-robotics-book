#!/usr/bin/env python3
"""
Lab 1 Solution: Heartbeat Publisher Node
Physical AI & Humanoid Robotics Textbook

This node demonstrates the publisher pattern by sending periodic
heartbeat messages to announce the robot is operational.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime


class HeartbeatNode(Node):
    """
    A simple publisher node that sends heartbeat messages at 1Hz.

    This pattern is used in production robots for health monitoring
    and fault detection. If heartbeats stop, the system knows the
    node has crashed or frozen.
    """

    def __init__(self):
        """Initialize the heartbeat publisher node."""
        super().__init__('heartbeat_publisher')

        # Create publisher
        # - Topic: '/robot/ready' (standard health monitoring topic)
        # - Message type: std_msgs/String
        # - Queue size: 10 (buffer up to 10 messages if subscriber is slow)
        self.publisher_ = self.create_publisher(
            String,
            '/robot/ready',
            10
        )

        # Create timer that fires every 1.0 seconds
        # Timer callback: self.publish_heartbeat
        self.timer = self.create_timer(1.0, self.publish_heartbeat)

        # Counter tracks how many heartbeats have been sent
        self.counter = 0

        # Log startup message
        self.get_logger().info('✅ Heartbeat node started - publishing at 1Hz')

    def publish_heartbeat(self):
        """
        Timer callback - called every 1.0 seconds.

        Publishes a heartbeat message with counter and timestamp.
        """
        # Create String message
        msg = String()

        # Set message data with counter and ISO 8601 timestamp
        # Example: "READY: Heartbeat #0 at 2025-12-15T12:30:00.123456"
        msg.data = f'READY: Heartbeat #{self.counter} at {datetime.now().isoformat()}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message (helps with debugging)
        self.get_logger().info(f'Published: {msg.data}')

        # Increment counter for next message
        self.counter += 1


def main(args=None):
    """
    Main entry point for the node.

    Initializes ROS 2, creates the node, and spins until shutdown.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = HeartbeatNode()

    try:
        # Spin the node (process callbacks until shutdown)
        # This is a blocking call - the program will stay here
        # until Ctrl+C or rclpy.shutdown() is called
        rclpy.spin(node)
    except KeyboardInterrupt:
        # User pressed Ctrl+C - exit gracefully
        pass
    finally:
        # Cleanup: destroy the node and shutdown ROS 2
        node.get_logger().info('🛑 Shutting down heartbeat node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
