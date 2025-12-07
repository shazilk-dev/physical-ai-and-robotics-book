import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        
        # Create publisher for /robot/ready topic
        self.publisher_ = self.create_publisher(String, '/robot/ready', 10)
        
        # Create timer that fires every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.counter = 0
        self.get_logger().info('✅ Heartbeat node started - publishing to /robot/ready at 1Hz')
    
    def publish_heartbeat(self):
        """Timer callback - publishes heartbeat message"""
        # Create String message
        msg = String()
        
        # Set message data with counter and ISO timestamp
        msg.data = f'READY: Heartbeat #{self.counter} at {datetime.now().isoformat()}'
        
        # Publish the message to /robot/ready topic
        self.publisher_.publish(msg)
        
        # Log confirmation to console
        self.get_logger().info(f'Published: {msg.data}')
        
        # Increment counter for next heartbeat
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
