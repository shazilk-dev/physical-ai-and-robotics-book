import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_publisher')
        
        # TODO: Create publisher
        # Hint: self.publisher_ = self.create_publisher(String, '/robot/ready', 10)
        # - Topic name: '/robot/ready'
        # - Message type: String
        # - Queue size: 10
        
        # TODO: Create timer (1Hz = 1.0 seconds)
        # Hint: self.timer = self.create_timer(1.0, self.publish_heartbeat)
        
        self.counter = 0
        self.get_logger().info('Heartbeat node started - waiting for implementation...')
    
    def publish_heartbeat(self):
        """Timer callback - publishes heartbeat message"""
        # TODO: Create String message
        # Hint: msg = String()
        
        # TODO: Set message data with counter and timestamp
        # Format: f'READY: Heartbeat #{self.counter} at {datetime.now().isoformat()}'
        # Hint: msg.data = ...
        
        # TODO: Publish the message
        # Hint: self.publisher_.publish(msg)
        
        # TODO: Log the published message
        # Hint: self.get_logger().info(f'Published: {msg.data}')
        
        # TODO: Increment the counter
        # Hint: self.counter += 1
        
        pass  # Remove this line when you implement the function

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
